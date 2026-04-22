import time

from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sar_msgs.msg import FSMEvent, MissionCoverage, UAVState
from std_msgs.msg import Empty, String


def _lawnmower(x_start, x_end, ymin, ymax, rows):
    """Boustrophedon sweep over a rectangular region, alternating row direction."""
    poses = []
    row_spacing = (ymax - ymin) / max(rows - 1, 1)
    for row in range(rows):
        y = ymin + row * row_spacing
        xs = [x_start, x_end] if row % 2 == 0 else [x_end, x_start]
        for x in xs:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = 1.0
            p.orientation.w = 1.0
            poses.append(p)
    return poses


class SwarmCoordinator(Node):

    def __init__(self):
        super().__init__('swarm_coordinator')

        # ===== Parameters =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('uav_id', 'x1'),
                ('num_uavs', 3),
                ('area_bounds', [-10, 10, -10, 10]),
                ('rows', 3),
                ('threshold', 0.95),
            ]
        )

        self.uav_id = self.get_parameter('uav_id').value
        self.num_uavs = self.get_parameter('num_uavs').value
        self.area = self.get_parameter('area_bounds').value
        self.rows = self.get_parameter('rows').value
        self.threshold = self.get_parameter('threshold').value

        # ===== Region assignment =====
        uav_ids = [f'x{i + 1}' for i in range(self.num_uavs)]
        self.uav_index = uav_ids.index(self.uav_id)

        xmin, xmax = self.area[0], self.area[1]
        slice_width = (xmax - xmin) / self.num_uavs
        self.x_start = xmin + self.uav_index * slice_width
        self.x_end = xmin + (self.uav_index + 1) * slice_width

        # ===== State =====
        self.current_mode = 'IDLE'
        self.is_paused = False
        self.coverage_waypoints_total = 0
        self.coverage_waypoints_visited = 0
        self.coverage_map = {}          # uav_id → coverage_ratio
        self.start_time = None
        self.run_id = int(time.time())

        qos_transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ===== Publishers =====
        self._waypoint_pub = self.create_publisher(
            PoseArray, f'/{self.uav_id}/nav/waypoints', qos_transient)
        self._event_pub = self.create_publisher(
            FSMEvent, f'/{self.uav_id}/fsm/event', 10)
        self._status_pub = self.create_publisher(String, '/mission/status', 10)
        self._go_home_pub = self.create_publisher(
            Empty, f'/{self.uav_id}/nav/go_home', 10)

        # ===== Subscribers =====
        self.create_subscription(UAVState, '/uav/state', self._on_fsm_state_change, 10)
        self.create_subscription(
            Empty,
            f'/{self.uav_id}/nav/reached_coverage_waypoint',
            self._on_waypoint_reached,
            10
        )
        self.create_subscription(
            MissionCoverage, '/mission/coverage', self._on_coverage_update, 10)

        self.get_logger().info(
            f'[{self.uav_id}] SwarmCoordinator ready — '
            f'region x:[{self.x_start:.1f}, {self.x_end:.1f}]'
        )

    # ===== FSM Reaction =====

    def _on_fsm_state_change(self, msg):
        if msg.uav_id != self.uav_id:
            return

        self.current_mode = msg.state

        if self.current_mode in ('VERIFYING', 'TARGET_LOCK'):
            self.is_paused = True
            return

        # Resuming from pause or recovery — navigator still has waypoints, just unpause
        if msg.previous_state in ('VERIFYING', 'TARGET_LOCK', 'RECOVERY'):
            self.is_paused = False
            return

        self.is_paused = False

        if self.current_mode == 'SEARCHING':
            self._reset_coverage()
            self.start_time = time.time()
            self._publish_search_waypoints()

        elif self.current_mode == 'REFINING':
            self._reset_coverage()
            self._publish_refinement_waypoints()

        elif self.current_mode == 'ASSISTING':
            self._reset_coverage()
            self._publish_assistive_waypoints()

        elif self.current_mode == 'RETURNING':
            self._go_home_pub.publish(Empty())

    # ===== Waypoint Generation =====

    def _publish_search_waypoints(self):
        poses = _lawnmower(
            self.x_start, self.x_end,
            self.area[2], self.area[3],
            self.rows
        )
        self._send_waypoints(poses)
        self._publish_status(
            f'[{self.uav_id}] AREA x:[{self.x_start:.1f},{self.x_end:.1f}] rows:{self.rows}'
        )

    def _publish_refinement_waypoints(self):
        poses = _lawnmower(
            self.x_start, self.x_end,
            self.area[2], self.area[3],
            self.rows * 2
        )
        self._send_waypoints(poses)

    def _publish_assistive_waypoints(self):
        other = {uid: r for uid, r in self.coverage_map.items() if uid != self.uav_id}

        if not other or all(r >= self.threshold for r in other.values()):
            self._publish_event('ALL_DRONES_DONE')
            return

        target_id = min(other, key=other.get)
        target_index = int(target_id[1:]) - 1   # 'x1' → 0, 'x2' → 1, …

        xmin, xmax = self.area[0], self.area[1]
        slice_width = (xmax - xmin) / self.num_uavs
        tx_start = xmin + target_index * slice_width
        tx_end = xmin + (target_index + 1) * slice_width

        poses = _lawnmower(
            tx_start, tx_end,
            self.area[2], self.area[3],
            self.rows
        )
        self._send_waypoints(poses)
        self._publish_status(f'[{self.uav_id}] ASSISTING → {target_id}')

    def _send_waypoints(self, poses):
        msg = PoseArray()
        msg.header.frame_id = 'world'
        msg.poses = poses
        self.coverage_waypoints_total = len(poses)
        self._waypoint_pub.publish(msg)

    # ===== Coverage Tracking =====

    def _on_waypoint_reached(self, _msg):
        if self.is_paused:
            return

        self.coverage_waypoints_visited += 1

        if self.coverage_waypoints_visited % 10 == 0:
            self._publish_status(
                f'[{self.uav_id}] PROGRESS: '
                f'{self.coverage_waypoints_visited}/{self.coverage_waypoints_total}'
            )

        self._check_coverage_events()

    def _on_coverage_update(self, msg):
        for uid, ratio in zip(msg.uav_ids, msg.coverage_ratios):
            self.coverage_map[uid] = ratio

    def _check_coverage_events(self):
        if self.coverage_waypoints_total == 0:
            return
        if self.coverage_waypoints_visited < self.coverage_waypoints_total:
            return

        coverage = self.coverage_waypoints_visited / self.coverage_waypoints_total

        if self.current_mode == 'SEARCHING':
            self._publish_event('REGION_COMPLETE', value=coverage)

        elif self.current_mode == 'REFINING':
            self._publish_event('REFINEMENT_COMPLETE', value=coverage)

        elif self.current_mode == 'ASSISTING':
            if all(r >= self.threshold for r in self.coverage_map.values()):
                self._publish_event('ALL_DRONES_DONE')
            else:
                self._publish_event('ASSIST_COMPLETE')

    # ===== Helpers =====

    def _reset_coverage(self):
        self.coverage_waypoints_total = 0
        self.coverage_waypoints_visited = 0

    def _publish_event(self, event: str, value: float = 0.0):
        msg = FSMEvent()
        msg.uav_id = self.uav_id
        msg.event = event
        msg.value = value
        msg.timestamp = self.get_clock().now().nanoseconds / 1e9
        self._event_pub.publish(msg)
        self.get_logger().info(f'[{self.uav_id}] event → {event} (value={value:.2f})')

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCoordinator()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
