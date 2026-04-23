import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sar_msgs.msg import FSMEvent, UAVState
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
            ]
        )

        self.uav_id = self.get_parameter('uav_id').value
        self.threshold = self.get_parameter('threshold').value

        # ===== State =====
        self.current_state = 'IDLE'
        self.previous_state = ''
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3

        # ===== Publishers =====
        self._state_pub = self.create_publisher(UAVState, '/uav/state', 10)
        self._target_pub = self.create_publisher(String, '/gcs/target_detected', 10)

        # ===== Subscribers =====
        self.create_subscription(Empty, '/mission/start', self._on_mission_start, 10)
        self.create_subscription(Empty, '/mission/stop', self._on_mission_stop, 10)
        self.create_subscription(
            FSMEvent, f'/{self.uav_id}/fsm/event', self._on_fsm_event, 10)

        # Publish initial state so other nodes see IDLE on startup
        self._publish_state()
        self.get_logger().info(f'[{self.uav_id}] UAVStateManager ready — state: IDLE')

    # ===== Callbacks =====

    def _on_mission_start(self, _msg):
        self._handle_event('MISSION_START')

    def _on_mission_stop(self, _msg):
        self._handle_event('MISSION_STOP')

    def _on_fsm_event(self, msg):
        self._handle_event(msg.event, value=msg.value)

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

        elif s == 'SEARCHING':
            if event == 'REGION_COMPLETE':
                if value < self.threshold:
                    self._transition('REFINING')
                else:
                    self._transition('ASSISTING')
            elif event == 'ALL_DRONES_DONE':
                self._transition('RETURNING')
            elif event == 'DETECTION_EVENT':
                self._transition('VERIFYING')
            elif event in ('OFF_COURSE', 'PATH_FAILED', 'COLLISION_RISK', 'COMMS_LOSS'):
                self._transition('RECOVERY')

        elif s == 'REFINING':
            if event == 'REFINEMENT_COMPLETE':
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

        elif s == 'RECOVERY':
            if event == 'REPLAN_SUCCESS':
                resume = self.previous_state if self.previous_state in _ALL_STATES else 'SEARCHING'
                self._transition(resume)
            elif event == 'REPLAN_FAIL':
                if self.recovery_attempts < self.max_recovery_attempts:
                    self._transition('RECOVERY')  # retry
                else:
                    self._transition('RETURNING')

    def _transition(self, new_state: str):
        old = self.current_state

        self.previous_state = old
        self.current_state = new_state

        self._publish_state()
        self._enter_state(new_state, old)

        self.get_logger().info(f'[{self.uav_id}] {old} → {new_state}')

    def _enter_state(self, new_state: str, old_state: str):

        # recovery
        if new_state == 'RECOVERY':
            self.recovery_attempts += 1

        if old_state == 'RECOVERY' and new_state != 'RECOVERY':
            self.recovery_attempts = 0

        # target lock
        if new_state == 'TARGET_LOCK':
            self._publish_target_detected()

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
