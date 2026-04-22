import heapq
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sar_msgs.msg import FSMEvent
from std_msgs.msg import Empty, String

GridCell = Tuple[int, int]


class AStarNavigationNode(Node):

    def __init__(self):
        super().__init__('astar_navigation_node')

        # ===== Parameters =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('uav_id', 'x1'),
                ('replan_check_rate', 0.5),
            ]
        )

        self.uav_id = self.get_parameter('uav_id').value
        replan_rate = self.get_parameter('replan_check_rate').value

        # ===== State =====
        self.current_pose = None
        self.waypoints = []       # world-coord list from coordinator
        self.waypoint_index = 0   # which waypoint we're currently heading to
        self.current_path = None  # active A* path to current waypoint
        self._cached_grid = None  # latest OccupancyGrid from world_model
        self.initial_plan_count = 0
        self.replan_count = 0
        self.path_failed_count = 0
        self._in_failure = False          # True after PATH_FAILED, until a plan succeeds
        self._consecutive_failures = 0
        self._path_failed_threshold = 3   # silent retries before PATH_FAILED (~1.5s at 2 Hz)
        self._max_replan_attempts = 20    # REPLAN_FAIL after ~10s of PATH_FAILED at 2 Hz

        qos_transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ===== Publishers =====
        self._path_pub = self.create_publisher(
            Path, f'/{self.uav_id}/nav/planned_path', qos_transient)
        self._event_pub = self.create_publisher(
            FSMEvent, f'/{self.uav_id}/fsm/event', 10)
        self._status_pub = self.create_publisher(String, '/mission/status', 10)

        # ===== Subscribers =====
        self.create_subscription(
            PoseArray,
            f'/{self.uav_id}/nav/waypoints',
            self._on_waypoints_received,
            qos_transient
        )
        self.create_subscription(
            Odometry,
            f'/{self.uav_id}/state/odom',
            self._on_pose_update,
            10
        )
        # path executor signals when the current waypoint is reached
        self.create_subscription(
            Empty,
            f'/{self.uav_id}/nav/reached_coverage_waypoint',
            self._on_waypoint_reached,
            10
        )
        self.create_subscription(
            OccupancyGrid,
            f'/{self.uav_id}/world_model/grid',
            self._on_grid_update,
            qos_transient
        )

        # ===== Replan Timer =====
        self.create_timer(1.0 / replan_rate, self._check_path_validity)

        self.get_logger().debug(f'AStarNavigationNode ready for {self.uav_id}')

    # ===== Pose Tracking =====

    def _on_pose_update(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    # ===== Waypoint Reception =====

    def _on_waypoints_received(self, msg):
        if not msg.poses:
            return

        self.waypoints = [(p.position.x, p.position.y) for p in msg.poses]
        self.waypoint_index = 0
        self.current_path = None
        if self.current_pose is not None:
            self._plan()
        # else: _check_path_validity timer retries once pose arrives

    def _on_waypoint_reached(self, msg):
        self.waypoint_index += 1
        if self.waypoint_index >= len(self.waypoints):
            self.get_logger().info(f'[{self.uav_id}] waypoint sequence complete')
            self.waypoints = []
            self.current_path = None
            return
        self.current_path = None
        self._plan()

    # ===== Grid Cache =====

    def _on_grid_update(self, msg):
        self._cached_grid = msg

    # ===== Planning =====

    def _plan(self):
        if not self.waypoints or self.current_pose is None:
            return
        if self.waypoint_index >= len(self.waypoints):
            return

        if self._cached_grid is None:
            self.get_logger().warn(f'[{self.uav_id}] no grid yet, waiting...')
            return

        width = self._cached_grid.info.width
        grid_flat = self._cached_grid.data
        origin_x = self._cached_grid.info.origin.position.x
        origin_y = self._cached_grid.info.origin.position.y
        resolution = self._cached_grid.info.resolution

        def w2g(wx, wy):
            return (
                int((wx - origin_x) / resolution),
                int((wy - origin_y) / resolution)
            )

        start = w2g(*self.current_pose)
        goal = w2g(*self.waypoints[self.waypoint_index])
        path = self._astar(start, goal, grid_flat, width)

        if path is None:
            self.path_failed_count += 1
            self._consecutive_failures += 1
            if not self._in_failure and self._consecutive_failures >= self._path_failed_threshold:
                self._in_failure = True
                self._publish_event('PATH_FAILED')
            elif self._in_failure and self._consecutive_failures >= self._max_replan_attempts:
                self._publish_event('REPLAN_FAIL')
                self._in_failure = False
                self._consecutive_failures = 0
            self._log_metrics()
            return

        self._consecutive_failures = 0
        if self._in_failure:
            self._in_failure = False
            self._publish_event('REPLAN_SUCCESS')

        if self.current_path is None:
            self.initial_plan_count += 1

        self.current_path = path
        self._path_pub.publish(self._build_path_msg(path, origin_x, origin_y, resolution))
        self._log_metrics()

    # ===== Path Validity Check =====

    def _check_path_validity(self):
        if not self.waypoints:
            return

        # Retry planning if we have waypoints but no path yet
        if self.current_path is None:
            self._plan()
            return

        if self._cached_grid is None:
            return

        width = self._cached_grid.info.width
        grid_flat = self._cached_grid.data
        height = len(grid_flat) // width

        for gx, gy in self.current_path:
            if not (0 <= gx < width and 0 <= gy < height):
                continue  # start cell can be outside grid when UAV spawns below grid boundary
            if grid_flat[gy * width + gx] > 0:  # only occupied (1) blocks; unknown (-1) is ok
                self.current_path = None
                self.replan_count += 1
                self._plan()
                return

    # ===== Core Algorithm =====

    def _astar(
        self,
        start: GridCell,
        goal: GridCell,
        grid_flat: List[int],
        width: int
    ) -> Optional[List[GridCell]]:
        height = len(grid_flat) // width

        def in_bounds(gx, gy):
            return 0 <= gx < width and 0 <= gy < height

        def is_free(gx, gy):
            # -1 (unknown) and 0 (free) are passable; 1 (occupied) is not
            return grid_flat[gy * width + gx] <= 0

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        def neighbors(cell):
            x, y = cell
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = x + dx, y + dy
                if in_bounds(nx, ny) and is_free(nx, ny):
                    yield (nx, ny)

        open_heap: List[Tuple[int, GridCell]] = []
        heapq.heappush(open_heap, (0, start))
        came_from: Dict[GridCell, GridCell] = {}
        g_score: Dict[GridCell, int] = {start: 0}
        open_set = {start}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            open_set.discard(current)

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for nb in neighbors(current):
                tentative_g = g_score[current] + 1
                if nb not in g_score or tentative_g < g_score[nb]:
                    came_from[nb] = current
                    g_score[nb] = tentative_g
                    f = tentative_g + heuristic(nb, goal)
                    if nb not in open_set:
                        heapq.heappush(open_heap, (f, nb))
                        open_set.add(nb)

        return None

    # ===== Helpers =====

    def _build_path_msg(self, cells, origin_x, origin_y, resolution):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for gx, gy in cells:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(gx * resolution + origin_x)
            pose.pose.position.y = float(gy * resolution + origin_y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

    # ===== Helpers =====

    def _publish_event(self, event: str):
        msg = FSMEvent()
        msg.uav_id = self.uav_id
        msg.event = event
        msg.timestamp = self.get_clock().now().nanoseconds / 1e9
        self._event_pub.publish(msg)

    # ===== Metrics =====

    def _log_metrics(self):
        msg = String()
        msg.data = (
            f'[{self.uav_id}] PLANS:{self.initial_plan_count} '
            f'REPLANS:{self.replan_count} '
            f'FAILURES:{self.path_failed_count}'
        )
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
