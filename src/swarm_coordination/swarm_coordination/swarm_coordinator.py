import time

from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sar_msgs.msg import FSMEvent, MissionCoverage, UAVState
from std_msgs.msg import Empty, String
from swarm_coordination.uav_state_manager import assign_helpers


def _lawnmower(x_start, x_end, ymin, ymax, rows, reverse=False):
    # Boustrophedon sweep. reverse=True starts at the top-right corner instead of bottom-left.
    poses = []
    row_spacing = (ymax - ymin) / max(rows - 1, 1)
    for i in range(rows):
        if reverse:
            y = ymax - i * row_spacing
            xs = [x_end, x_start] if i % 2 == 0 else [x_start, x_end]
        else:
            y = ymin + i * row_spacing
            xs = [x_start, x_end] if i % 2 == 0 else [x_end, x_start]
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
                ('threshold', 0.90),
                ('completion_wait_sec', 1.5),  # brief sync hover before deciding
                ('resolution', 1.0),
                ('coverage_radius', 1.0),  # sensor footprint radius (m) for cell marking
            ]
        )

        self.uav_id = self.get_parameter('uav_id').value
        self.num_uavs = self.get_parameter('num_uavs').value
        self.area = self.get_parameter('area_bounds').value
        self.rows = self.get_parameter('rows').value
        self.threshold = self.get_parameter('threshold').value
        self.completion_wait_sec = self.get_parameter('completion_wait_sec').value
        self.resolution = self.get_parameter('resolution').value
        self.coverage_radius = self.get_parameter('coverage_radius').value

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
        self.run_id = int(time.time())
        self.last_coverage = 0.0
        self.stall_count = 0
        self._completion_timer = None  # active during the post-refine sync hover
        self.home_pose = None

        # grid cells visited; persists across SEARCHING → REFINING
        self.visited_cells = set()
        self.slice_w_cells = int((self.x_end - self.x_start) / self.resolution)
        self.slice_h_cells = int((self.area[3] - self.area[2]) / self.resolution)
        self.total_cells = self.slice_w_cells * self.slice_h_cells

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

        # ===== Subscribers =====
        self.create_subscription(
            UAVState, '/uav/state', self._on_fsm_state_change, 10)
        self.create_subscription(
            Empty, f'/{self.uav_id}/nav/reached_coverage_waypoint', self._on_waypoint_reached, 10)
        self.create_subscription(
            MissionCoverage, '/mission/coverage', self._on_coverage_update, 10)
        self.create_subscription(
            FSMEvent, f'/{self.uav_id}/fsm/command', self._on_fsm_command, 10)
        self.create_subscription(
            Odometry, f'/{self.uav_id}/state/odom', self._on_own_odom, 10) # own odometry

        # subscribe to all peers' odom so cells they pass through our slice get marked
        for i in range(self.num_uavs):
            peer_id = f'x{i + 1}'
            self.create_subscription(
                Odometry, f'/{peer_id}/state/odom', self._on_odom, 10)

        # ===== Timers =====
        # 5Hz coverage heartbeat so peers see fresh values, not just per-waypoint
        self.create_timer(0.2, self._publish_coverage_status)

    # ===== Coverage Tracking (grid-based) =====

    def _on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        gx_c = int((x - self.x_start) / self.resolution)
        gy_c = int((y - self.area[2]) / self.resolution)
        # mark all cells within the sensor footprint, clipped to the assigned slice
        r = int(self.coverage_radius / self.resolution)
        r_sq = r * r
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r_sq:
                    continue
                gx, gy = gx_c + dx, gy_c + dy
                if 0 <= gx < self.slice_w_cells and 0 <= gy < self.slice_h_cells:
                    self.visited_cells.add((gx, gy))
        
    def _on_own_odom(self, msg):
        if self.home_pose is None:
            self.home_pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            )

    # ===== FSM Reaction =====

    def _on_fsm_command(self, msg):
        if msg.uav_id != self.uav_id:
            return

        # always unpause on explicit command
        self.is_paused = False

        if msg.event == 'START_SEARCH':
            self._reset_coverage()
            self.visited_cells = set()  # fresh mission
            self.last_coverage = 0.0
            self.stall_count = 0
            self._publish_search_waypoints()

        elif msg.event == 'START_REFINEMENT':
            self._reset_coverage()
            self.last_coverage = 0.0
            self.stall_count = 0
            self._publish_refinement_waypoints()

        elif msg.event == 'START_ASSIST':
            self._reset_coverage()
            self._publish_assistive_waypoints(self.threshold)

        elif msg.event == 'GO_HOME':
            self._reset_coverage()
            self._publish_return_home_waypoints()

        elif msg.event == 'REPLAN':
            if self.current_mode == 'SEARCHING':
                self._publish_search_waypoints()
            elif self.current_mode == 'REFINING':
                self._publish_refinement_waypoints()
            elif self.current_mode == 'ASSISTING':
                self._publish_assistive_waypoints(self.threshold)

        elif msg.event == 'STOP':
            self._send_waypoints([], mode="STOP")

    def _on_fsm_state_change(self, msg):
        if msg.uav_id != self.uav_id:
            return

        self.current_mode = msg.state

        # leaving REFINING mid-hover invalidates the pending decision
        if self._completion_timer is not None and self.current_mode != 'REFINING':
            self.destroy_timer(self._completion_timer)
            self._completion_timer = None

        if self.current_mode in ('VERIFYING', 'TARGET_LOCK'):
            self.is_paused = True
            return

        # Resuming from pause or recovery — navigator still has waypoints, just unpause
        if msg.previous_state in ('VERIFYING', 'TARGET_LOCK', 'RECOVERY'):
            self.is_paused = False
            return

        self.is_paused = False

    # ===== Waypoint Generation =====

    def _publish_search_waypoints(self):
        poses = _lawnmower(
            self.x_start, self.x_end,
            self.area[2], self.area[3],
            self.rows
        )
        self._send_waypoints(poses, mode="SEARCH")
        self._publish_status(
            f'[{self.uav_id}] AREA x:[{self.x_start:.1f},{self.x_end:.1f}] rows:{self.rows}'
        )

    def _publish_refinement_waypoints(self):
        # start from the top-right where SEARCHING ended, snake back down
        poses = _lawnmower(
            self.x_start, self.x_end,
            self.area[2], self.area[3],
            self.rows * 2,
            reverse=True
        )
        self._send_waypoints(poses, mode="REFINE")

    def _publish_assistive_waypoints(self, pair_threshold):
        pairings = assign_helpers(self.coverage_map, pair_threshold)
        target_id = pairings.get(self.uav_id)
        if target_id is None:
            # FSM put us in ASSISTING but no target survived re-pairing — stand down
            self._publish_event('ASSIST_COMPLETE')
            return
        target_index = int(target_id[1:]) - 1   # 'x1' → 0, 'x2' → 1, …

        xmin, xmax = self.area[0], self.area[1]
        slice_width = (xmax - xmin) / self.num_uavs
        tx_start = xmin + target_index * slice_width
        tx_end = xmin + (target_index + 1) * slice_width

        # offset rows by half spacing so they fill gaps between peer's lawnmower
        spacing = (self.area[3] - self.area[2]) / (self.rows * 2 - 1)
        poses = _lawnmower(
            tx_start, tx_end,
            self.area[2] + spacing / 2,
            self.area[3] - spacing / 2,
            self.rows * 2 - 1,
        )
        self._send_waypoints(poses, mode="ASSIST", x_start=tx_start, x_end=tx_end)
        self._publish_status(f'[{self.uav_id}] ASSISTING → {target_id}')

    def _publish_return_home_waypoints(self):
        pose = Pose()
        if self.home_pose is None:
            return  # or fallback

        pose.position.x = self.home_pose[0]
        pose.position.y = self.home_pose[1]
        pose.position.z = 1.0
        pose.orientation.w = 1.0
        self._send_waypoints([])  # clear current path
        self._send_waypoints([pose], mode="GO_HOME")

        self._publish_status(f'[{self.uav_id}] RETURNING HOME')

    def _send_waypoints(self, poses, mode="NORMAL", x_start=None, x_end=None):
        msg = PoseArray()

        xs = self.x_start if x_start is None else x_start
        xe = self.x_end if x_end is None else x_end

        msg.header.frame_id = f"{mode}|{xs},{xe}"

        msg.poses = poses
        self.coverage_waypoints_total = len(poses)
        self._waypoint_pub.publish(msg)

    # ===== Coverage Tracking =====

    def _on_waypoint_reached(self, _msg):
        if self.is_paused:
            return

        self.coverage_waypoints_visited += 1

        assigned_area = (self.x_end - self.x_start) * (self.area[3] - self.area[2])
        coverage_ratio = len(self.visited_cells) / self.total_cells if self.total_cells else 0.0
        area_covered = coverage_ratio * assigned_area

        self._publish_status(
            f'[{self.uav_id}] PROGRESS: '
            f'{self.coverage_waypoints_visited}/{self.coverage_waypoints_total} '
            f'AREA: {area_covered:.1f}/{assigned_area:.1f}'
        )

        self._check_coverage_events()

    def _on_coverage_update(self, msg):
        for uid, ratio in zip(msg.uav_ids, msg.coverage_ratios):
            self.coverage_map[uid] = ratio

    def _check_coverage_events(self):
        if self.coverage_waypoints_total == 0:
            return

        coverage = len(self.visited_cells) / self.total_cells if self.total_cells else 0.0

        # Early stop for assisting
        if self.current_mode == 'ASSISTING':
            peer = {uid: r for uid, r in self.coverage_map.items() if uid != self.uav_id}
            if peer and all(r >= self.threshold for r in peer.values()):
                self._publish_event('ASSIST_COMPLETE')
                return

        if self.coverage_waypoints_visited < self.coverage_waypoints_total:
            return

        if self.current_mode == 'SEARCHING':
            self._publish_event('REGION_COMPLETE', value=coverage)

        elif self.current_mode == 'REFINING':
            improvement = coverage - self.last_coverage

            if improvement < 0.01:  # less than 1% improvement
                self.stall_count += 1
            else:
                self.stall_count = 0

            self.last_coverage = coverage

            # only refine again if needed
            if coverage + 1e-6 < self.threshold and self.stall_count < 2:
                self.coverage_waypoints_visited = 0
                self._publish_refinement_waypoints()
                return

            # hover briefly so peer coverage settles before deciding assist vs home
            if self._completion_timer is None:
                self._completion_timer = self.create_timer(
                    self.completion_wait_sec, self._on_completion_timeout
                )

        elif self.current_mode == 'ASSISTING':
            self._publish_event('ASSIST_COMPLETE')

    def _on_completion_timeout(self):
        self.destroy_timer(self._completion_timer)
        self._completion_timer = None

        # state changed during the hover (mission stop, detection, etc) — skip
        if self.current_mode != 'REFINING':
            return

        self._publish_event('REFINEMENT_COMPLETE', value=self.last_coverage)

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

    def _publish_coverage_status(self):
        # 5Hz heartbeat so peers see fresh grid coverage between waypoint hits
        if self.total_cells == 0 or self.current_mode in ('IDLE', 'RETURNING', 'EMERGENCY_STOP'):
            return
        assigned_area = (self.x_end - self.x_start) * (self.area[3] - self.area[2])
        coverage_ratio = len(self.visited_cells) / self.total_cells
        area_covered = coverage_ratio * assigned_area
        self._publish_status(
            f'[{self.uav_id}] PROGRESS: '
            f'{self.coverage_waypoints_visited}/{max(self.coverage_waypoints_total, 1)} '
            f'AREA: {area_covered:.1f}/{assigned_area:.1f}'
        )

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
