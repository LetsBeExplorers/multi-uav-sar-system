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
                ('x_min', -999.0),
                ('x_max', 999.0),
            ]
        )

        self.uav_id = self.get_parameter('uav_id').value
        replan_rate = self.get_parameter('replan_check_rate').value
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value

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
        self._path_failed_threshold = 20  # silent retries before PATH_FAILED (~10s at 2 Hz)
        self._max_replan_attempts = 40    # REPLAN_FAIL ~10s after PATH_FAILED at 2 Hz
        self.in_region = False

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

    # ===== Pose Tracking =====

    def _on_pose_update(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        # detect if UAV has entered its assigned region
        x = self.current_pose[0]
        if self.x_min <= x <= self.x_max:
            self.in_region = True

    # ===== Waypoint Reception =====

    def _on_waypoints_received(self, msg):
        if not msg.poses:
            # Clear ALL planning state
            self.current_goal = None
            self.waypoints = []
            self.waypoint_index = 0
            self.current_path = None

            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = 'map'
            self._path_pub.publish(empty_path)

            return

        # get region bounds
        bounds = msg.header.frame_id.split(',')
        if len(bounds) == 2:
            self.x_min = float(bounds[0])
            self.x_max = float(bounds[1])
            self.in_region = False

        self.waypoints = [(p.position.x, p.position.y) for p in msg.poses]
        self.waypoint_index = 0
        self.current_path = None
        if self.current_pose is not None:
            self._plan()

    def _on_waypoint_reached(self, msg):
        self.waypoint_index += 1
        if self.waypoint_index >= len(self.waypoints):
            self.waypoints = []
            self.current_path = None
            return
        self.current_path = None
        self._plan()

    # ===== Grid Cache =====

    def _on_grid_update(self, msg):
        self._cached_grid = msg

    # ===== Planning =====

    def _find_nearest_free(self, goal, grid_flat, width):
        height = len(grid_flat) // width
        gx, gy = goal

        # search outward from goal in a small radius
        for radius in range(1, 4):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = gx + dx, gy + dy

                    # stay inside section bounds
                    if 0 <= nx < width and 0 <= ny < height:

                        # convert to world x
                        origin_x = self._cached_grid.info.origin.position.x
                        resolution = self._cached_grid.info.resolution
                        wx = nx * resolution + origin_x

                        # enforce region constraint
                        if not (self.x_min <= wx <= self.x_max):
                            continue

                        if grid_flat[ny * width + nx] <= 0:
                            return (nx, ny)

        # fallback: return original goal if nothing found
        return goal

    def _can_plan(self):
        return (
            self.waypoints and
            self.current_pose is not None and
            self.waypoint_index < len(self.waypoints) and
            self._cached_grid is not None
        )

    def _plan(self):
        if not self._can_plan():
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

        # clamp goal to grid bounds — landing pads sit outside the search grid
        height = len(grid_flat) // width
        goal = (max(0, min(goal[0], width - 1)),
                max(0, min(goal[1], height - 1)))

        # adjust goal if it's blocked (e.g., another UAV is sitting there)
        idx = goal[1] * width + goal[0]

        if grid_flat[idx] > 0:   # only if actually blocked
            goal = self._find_nearest_free(goal, grid_flat, width)
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
                saved = self.current_path
                self.current_path = None
                self.replan_count += 1
                self._plan()
                if self.current_path is None:
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
                step_cost = 1
                nx, ny = nb
                for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ax, ay = nx + ddx, ny + ddy
                    if in_bounds(ax, ay) and grid_flat[ay * width + ax] > 0:
                        step_cost = 10  # neighbor touches an obstacle — keep clearance
                        break

                # soft region penalty (only after entering region)
                wx = nb[0] * self._cached_grid.info.resolution + self._cached_grid.info.origin.position.x
                margin = self._cached_grid.info.resolution  # 1 cell tolerance

                region_penalty = 0

                if self.in_region:
                    if wx < self.x_min - margin or wx > self.x_max + margin:
                        region_penalty = 15

                tentative_g = g_score[current] + step_cost + region_penalty
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
