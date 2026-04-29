import random
import rclpy
from rclpy.node import Node
from sar_msgs.msg import FSMEvent, Alert, DetectionEvent


class VerificationNode(Node):

    def __init__(self):
        super().__init__('verification_node')

        # ===== Parameters =====
        self.declare_parameters(namespace='', parameters=[
            ('uav_id', 'x1'),
            ('verify_delay_min', 2.0),
            ('verify_delay_max', 5.0),
            ('verify_confirm_probability', 0.7),
            ('verify_timeout', 10.0),
        ])

        self.uav_id = self.get_parameter('uav_id').value
        self.delay_min = self.get_parameter('verify_delay_min').value
        self.delay_max = self.get_parameter('verify_delay_max').value
        self.confirm_prob = self.get_parameter('verify_confirm_probability').value
        self.verify_timeout = self.get_parameter('verify_timeout').value

        # ===== State =====
        self._verify_timer = None
        self._timeout_timer = None
        self._deciding = False  # re-entry guard
        self._last_detection = None  # (x, y, confidence) of most recent detection

        # ===== Publishers =====
        self._fsm_pub = self.create_publisher(FSMEvent, f'/{self.uav_id}/fsm/event', 10)
        self._target_pub = self.create_publisher(DetectionEvent, '/targets/confirmed', 10)
        self._alert_pub = self.create_publisher(Alert, '/alerts', 10)

        # ===== Subscribers =====
        self.create_subscription(FSMEvent, f'/{self.uav_id}/fsm/command', self._on_command, 10)
        self.create_subscription(DetectionEvent, f'/{self.uav_id}/detection/event', self._on_detection, 10)

    # ===== Callbacks =====

    def _on_command(self, msg):
        if msg.uav_id != self.uav_id:
            return

        if msg.event == 'START_VERIFY':
            self._start_verification()

    def _on_detection(self, msg):
        # Cache the most recent detection so we can report its location on confirm.
        if msg.uav_id != self.uav_id:
            return
        self._last_detection = (msg.x, msg.y, msg.confidence)

    # ===== Core Logic =====

    def _start_verification(self):
        # Destroy any prior timers; cancel() alone leaves them on the executor.
        self._clear_timers()

        delay = random.uniform(self.delay_min, self.delay_max)
        self._verify_timer = self.create_timer(delay, self._decide)

        # Safety net so the FSM can recover if _decide never fires.
        self._timeout_timer = self.create_timer(self.verify_timeout, self._on_timeout)

    def _decide(self):
        # Re-entry guard for multi-threaded executors.
        if self._deciding:
            return
        self._deciding = True
        try:
            self._clear_timers()
            result = self._simulate_verification()
            self._handle_result(result)
        finally:
            self._deciding = False

    def _on_timeout(self):
        self._clear_timers()
        self._handle_result('TIMEOUT')

    def _clear_timers(self):
        if self._verify_timer is not None:
            self.destroy_timer(self._verify_timer)
            self._verify_timer = None
        if self._timeout_timer is not None:
            self.destroy_timer(self._timeout_timer)
            self._timeout_timer = None

    def _simulate_verification(self):
        return 'CONFIRMED_TARGET' if random.random() < self.confirm_prob else 'FALSE_POSITIVE'

    def _handle_result(self, result):
        now = self.get_clock().now().nanoseconds / 1e9

        self._publish_fsm_event(now, result)

        if result == 'CONFIRMED_TARGET':
            self._publish_confirmed_target()

        self._publish_alert(now, result)

        # ===== Publishers =====

    def _publish_fsm_event(self, timestamp, event):
        msg = FSMEvent()
        msg.uav_id = self.uav_id
        msg.event = event
        msg.timestamp = timestamp
        self._fsm_pub.publish(msg)

    def _publish_confirmed_target(self):
        if self._last_detection is None:
            return

        x, y, conf = self._last_detection

        msg = DetectionEvent()
        msg.uav_id = self.uav_id
        msg.x = x
        msg.y = y
        msg.confidence = conf

        self._target_pub.publish(msg)

    def _publish_alert(self, timestamp, result):
        msg = Alert()
        msg.uav_id = self.uav_id
        msg.timestamp = timestamp

        if result == 'CONFIRMED_TARGET':
            msg.level = 'CRITICAL'
            msg.type = 'CONFIRMATION'
            msg.message = self._format_target_message()
        elif result == 'TIMEOUT':
            msg.level = 'WARNING'
            msg.type = 'ERROR'
            msg.message = 'Verification timed out'
        else:  # FALSE_POSITIVE
            msg.level = 'INFO'
            msg.type = 'DETECTION'
            msg.message = 'False detection'

        self._alert_pub.publish(msg)

    def _format_target_message(self):
        if self._last_detection is None:
            return 'Target confirmed (location unknown)'
        x, y, conf = self._last_detection
        return f'Target confirmed at ({x:.1f}, {y:.1f}) conf={conf:.2f}'


def main(args=None):
    rclpy.init(args=args)
    node = VerificationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()