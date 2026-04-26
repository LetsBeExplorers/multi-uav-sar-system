import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sar_msgs.msg import FSMEvent, UAVState, MissionCoverage
from std_msgs.msg import Empty, String


# States where normal search behavior is active
_SEARCH_STATES = {'SEARCHING', 'REFINING', 'ASSISTING'}

# All valid states
_ALL_STATES = _SEARCH_STATES | {'IDLE', 'VERIFYING', 'TARGET_LOCK', 'RETURNING', 'RECOVERY', 'EMERGENCY_STOP'}


class UAVStateManager(Node):

    def __init__(self):
        super().__init__('uav_state_manager')

        # ===== Parameters =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('uav_id', 'x1'),
                ('threshold', 0.95),
                ('assist_threshold', 0.80),
                ('num_uavs', 3),
            ]
        )

        self.uav_id = self.get_parameter('uav_id').value
        self.threshold = self.get_parameter('threshold').value
        self.assist_threshold = self.get_parameter('assist_threshold').value
        self.num_uavs = self.get_parameter('num_uavs').value

        # ===== State =====
        self.current_state = 'IDLE'
        self.previous_state = ''
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3
        self.coverage_map = {}

        # ===== Publishers =====
        self._state_pub = self.create_publisher(UAVState, '/uav/state', 10)
        self._target_pub = self.create_publisher(String, '/gcs/target_detected', 10)
        self._command_pub = self.create_publisher(FSMEvent, f'/{self.uav_id}/fsm/command', 10)

        # ===== Subscribers =====
        self.create_subscription(Empty, '/mission/start', self._on_mission_start, 10)
        self.create_subscription(Empty, '/mission/stop', self._on_mission_stop, 10)
        self.create_subscription(FSMEvent, f'/{self.uav_id}/fsm/event', self._on_fsm_event, 10)
        self.create_subscription(MissionCoverage, '/mission/coverage', self._on_coverage_update, 10)

        # Publish initial state so other nodes see IDLE on startup
        self._publish_state()
        
    # ===== Callbacks =====

    def _on_mission_start(self, _msg):
        self._handle_event('MISSION_START')

    def _on_mission_stop(self, _msg):
        self._handle_event('MISSION_STOP')

    def _on_fsm_event(self, msg):
        self.get_logger().info(f"RECEIVED EVENT: {msg.event}")
        self._handle_event(msg.event, value=msg.value)

    def _on_coverage_update(self, msg):
        for uid, ratio in zip(msg.uav_ids, msg.coverage_ratios):
            self.coverage_map[uid] = ratio

    # ===== Core FSM =====

    def _handle_event(self, event: str, value: float = 0.0):
        # ── Tier 1: Emergency (any state) ──────────────────────────────────
        if event in ('CRITICAL_FAILURE', 'HARD_COLLISION'):
            self._transition('EMERGENCY_STOP')
            return

        # EMERGENCY_STOP only exits on RESET
        if self.current_state == 'EMERGENCY_STOP':
            if event == 'RESET':
                self._transition('IDLE')
            return

        # ── Tier 2: Battery constraint (any non-emergency state) ───────────
        if event == 'LOW_BATTERY':
            self._transition('RETURNING')
            return

        # ── Tier 3: Mission commands ───────────────────────────────────────
        if event == 'MISSION_STOP':
            self._transition('RETURNING')
            return
        if event == 'MISSION_UPDATE':
            self._transition('SEARCHING')
            return
        if event == 'RESET':
            self._transition('IDLE')
            return

        # ── Tier 4: State-specific logic ──────────────────────────────────
        s = self.current_state

        if s == 'IDLE':
            if event == 'MISSION_START':
                self._transition('SEARCHING')
                self._publish_command('START_SEARCH')

        elif s == 'SEARCHING':
            if event == 'REGION_COMPLETE':
                self._transition('REFINING')
            elif event == 'ALL_DRONES_DONE':
                self._transition('RETURNING')
            elif event == 'DETECTION_EVENT':
                self._transition('VERIFYING')
            elif event in ('OFF_COURSE', 'PATH_FAILED', 'COLLISION_RISK', 'COMMS_LOSS'):
                self._transition('RECOVERY')

        elif s == 'REFINING':

            if event == 'REFINEMENT_COMPLETE':

                if len(self.coverage_map) < self.num_uavs:
                    return  # not enough info yet

                others = {
                    uid: r for uid, r in self.coverage_map.items()
                    if uid != self.uav_id
                }

                if all(r >= self.assist_threshold for r in others.values()):
                    self._transition('RETURNING')
                else:
                    self._transition('ASSISTING')

            elif event == 'ALL_DRONES_DONE':
                self._transition('RETURNING')

            elif event == 'DETECTION_EVENT':
                self._transition('VERIFYING')

            elif event in ('OFF_COURSE', 'PATH_FAILED', 'COLLISION_RISK', 'COMMS_LOSS'):
                self._transition('RECOVERY')

        elif s == 'ASSISTING':
            if event == 'ALL_DRONES_DONE':
                self._transition('RETURNING')

            elif event == 'ASSIST_COMPLETE':

                if len(self.coverage_map) < self.num_uavs:
                    return  # not enough info yet

                others = {
                    uid: r for uid, r in self.coverage_map.items()
                    if uid != self.uav_id
                }

                if all(r >= self.threshold for r in others.values()):
                    self._transition('RETURNING')
                else:
                    # Re-emit ASSISTING so the coordinator picks a new target region
                    self._transition('ASSISTING')

            elif event == 'DETECTION_EVENT':
                self._transition('VERIFYING')

            elif event in ('OFF_COURSE', 'PATH_FAILED', 'COLLISION_RISK', 'COMMS_LOSS'):
                self._transition('RECOVERY')

        elif s == 'VERIFYING':
            if event == 'CONFIRMED_TARGET':
                self._transition('TARGET_LOCK')
            elif event in ('FALSE_POSITIVE', 'TIMEOUT'):
                resume = self.previous_state if self.previous_state in _SEARCH_STATES else 'SEARCHING'
                self._transition(resume)

        elif s == 'TARGET_LOCK':
            if event in ('TARGET_LOST', 'COMMAND_RETURN', 'TIMEOUT'):
                self._transition('RETURNING')

        elif s == 'RETURNING':
            if event == 'HOME_REACHED':
                self._transition('IDLE')
            elif event == 'MISSION_START':
                self._transition('SEARCHING')
                self._publish_command('START_SEARCH')

        elif s == 'RECOVERY':
            if event == 'REPLAN_SUCCESS':
                resume = self.previous_state if self.previous_state in _ALL_STATES else 'SEARCHING'
                self._transition(resume)

            elif event == 'REPLAN_FAIL':
                if self.recovery_attempts < self.max_recovery_attempts:
                    self._transition('RECOVERY')  # retry
                else:
                    self._transition('RETURNING')

            elif event in ('COLLISION_RISK', 'PATH_FAILED', 'COMMS_LOSS'):
                pass  # intentionally ignored during recovery

    def _transition(self, new_state: str):
        old = self.current_state

        self.previous_state = old
        self.current_state = new_state

        self._publish_state()
        self._enter_state(new_state, old)

    def _enter_state(self, new_state: str, old_state: str):

        # recovery
        if new_state == 'RECOVERY':
            self.recovery_attempts += 1
            self._publish_command('REPLAN')

        if old_state == 'RECOVERY' and new_state != 'RECOVERY':
            self.recovery_attempts = 0

        # target lock
        if new_state == 'TARGET_LOCK':
            self._publish_target_detected()

        # returning
        if new_state == 'RETURNING':
            self._publish_command('GO_HOME')

    # ===== Publishers =====

    def _publish_state(self):
        msg = UAVState()
        msg.uav_id = self.uav_id
        msg.state = self.current_state
        msg.previous_state = self.previous_state
        msg.timestamp = self.get_clock().now().nanoseconds / 1e9
        self._state_pub.publish(msg)

    def _publish_target_detected(self):
        # stub — detection module will provide full location data when integrated
        msg = String()
        msg.data = f'{self.uav_id}:TARGET_LOCK'
        self._target_pub.publish(msg)

    def _publish_command(self, event):
        msg = FSMEvent()
        msg.uav_id = self.uav_id
        msg.event = event
        msg.timestamp = self.get_clock().now().nanoseconds / 1e9
        msg.value = 0.0
        self._command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UAVStateManager()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
