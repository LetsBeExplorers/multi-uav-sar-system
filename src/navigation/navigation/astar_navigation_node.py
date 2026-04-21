import heapq
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sar_msgs.msg import FSMEvent
from sar_msgs.srv import GetOccupancyGrid
from std_msgs.msg import String

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
        self.current_goal = None
        self.current_path = None
        self.initial_plan_count = 0
        self.replan_count = 0
        self.path_failed_count = 0

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

        # ===== Service Client =====
        self._grid_client = self.create_client(
            GetOccupancyGrid, f'/{self.uav_id}/world_model/get_grid')

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
        if self.current_pose is None:
            return
        if not msg.poses:
            return

        # store last waypoint as the overall goal (for replan reference)
        last = msg.poses[-1]
        self.current_goal = (last.position.x, last.position.y)
        self.current_path = None

        self._plan(msg.poses)

    # ===== Planning =====

    def _get_grid(self):
        if not self._grid_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('world_model service unavailable')
            return None
        future = self._grid_client.call_async(GetOccupancyGrid.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result() if future.done() else None

    def _plan(self, poses):
        resp = self._get_grid()
        if resp is None:
            return

        width = resp.width
        grid_flat = resp.grid
        origin_x = resp.origin_x
        origin_y = resp.origin_y
        resolution = resp.resolution

        def w2g(wx, wy):
            return (
                int((wx - origin_x) / resolution),
                int((wy - origin_y) / resolution)
            )

        start = w2g(*self.current_pose)
        waypoints = [w2g(p.position.x, p.position.y) for p in poses]

        # chain A* through every waypoint
        full_path = []
        seg_start = start
        for goal in waypoints:
            segment = self._astar(seg_start, goal, grid_flat, width)
            if segment is None:
                self.path_failed_count += 1
                event = FSMEvent()
                event.uav_id = self.uav_id
                event.event = 'PATH_FAILED'
                event.timestamp = self.get_clock().now().nanoseconds / 1e9
                self._event_pub.publish(event)
                self._log_metrics()
                return
            full_path.extend(segment)
            seg_start = goal

        if self.current_path is None:
            self.initial_plan_count += 1

        self.current_path = full_path
        self._path_pub.publish(self._build_path_msg(full_path, origin_x, origin_y, resolution))
        self._log_metrics()

    # ===== Path Validity Check =====

    def _check_path_validity(self):
        if self.current_path is None or self.current_goal is None:
            return

        resp = self._get_grid()
        if resp is None:
            return

        width = resp.width
        grid_flat = resp.grid

        for gx, gy in self.current_path:
            if grid_flat[gy * width + gx] != 0:
                self.current_path = None
                self.replan_count += 1
                self._replan_to_goal(resp)
                return

    def _replan_to_goal(self, resp):
        if self.current_goal is None or self.current_pose is None:
            return

        width = resp.width
        grid_flat = resp.grid
        origin_x = resp.origin_x
        origin_y = resp.origin_y
        resolution = resp.resolution

        def w2g(wx, wy):
            return (
                int((wx - origin_x) / resolution),
                int((wy - origin_y) / resolution)
            )

        start = w2g(*self.current_pose)
        goal = w2g(*self.current_goal)
        path = self._astar(start, goal, grid_flat, width)

        if path is None:
            self.path_failed_count += 1
            event = FSMEvent()
            event.uav_id = self.uav_id
            event.event = 'PATH_FAILED'
            event.timestamp = self.get_clock().now().nanoseconds / 1e9
            self._event_pub.publish(event)
            self._log_metrics()
            return

        self.current_path = path
        self._path_pub.publish(
            self._build_path_msg(path, origin_x, origin_y, resolution))
        self._log_metrics()

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
            return grid_flat[gy * width + gx] == 0

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
