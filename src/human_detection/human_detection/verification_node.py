import random
import rclpy
from rclpy.node import Node
from sar_msgs.msg import FSMEvent, Alert


class VerificationNode(Node):

    def __init__(self):
        super().__init__('verification_node')

        # ===== Parameters =====
        self.declare_parameters(namespace='', parameters=[
            ('uav_id', 'x1'),
            ('verify_delay_min', 2.0),
            ('verify_delay_max', 5.0),
            ('verify_confirm_probability', 0.7),
        ])

        self.uav_id = self.get_parameter('uav_id').value
        self.delay_min = self.get_parameter('verify_delay_min').value
        self.delay_max = self.get_parameter('verify_delay_max').value
        self.confirm_prob = self.get_parameter('verify_confirm_probability').value

        # ===== State =====
        self._verify_timer = None

        # ===== Publishers =====
        self._fsm_pub = self.create_publisher(
            FSMEvent, f'/{self.uav_id}/fsm/event', 10)

        self._alert_pub = self.create_publisher(
            Alert, '/alerts', 10)

        # ===== Subscribers =====
        self.create_subscription(
            FSMEvent,
            f'/{self.uav_id}/fsm/command',
            self._on_command,
            10
        )

    # ===== Callbacks =====

    def _on_command(self, msg):
        if msg.uav_id != self.uav_id:
            return

        if msg.event == 'START_VERIFY':
            self._start_verification()

    # ===== Core Logic =====

    def _start_verification(self):
        delay = random.uniform(self.delay_min, self.delay_max)

        if self._verify_timer is not None:
            self._verify_timer.cancel()

        self._verify_timer = self.create_timer(delay, self._decide)

    def _decide(self):
        if self._verify_timer is not None:
            self._verify_timer.cancel()
            self._verify_timer = None

        result = self._simulate_verification()
        self._handle_result(result)

    def _simulate_verification(self):
        return 'CONFIRMED_TARGET' if random.random() < self.confirm_prob else 'FALSE_POSITIVE'

    def _handle_result(self, result):
        now = self.get_clock().now().nanoseconds / 1e9

        self._publish_fsm_event(now, result)
        self._publish_alert(now, result)

    # ===== Publishers =====

    def _publish_fsm_event(self, timestamp, event):
        msg = FSMEvent()
        msg.uav_id = self.uav_id
        msg.event = event
        msg.timestamp = timestamp
        self._fsm_pub.publish(msg)

    def _publish_alert(self, timestamp, result):
        msg = Alert()
        msg.uav_id = self.uav_id
        msg.timestamp = timestamp

        if result == 'CONFIRMED_TARGET':
            msg.level = 'CRITICAL'
            msg.type = 'CONFIRMATION'
            msg.message = 'Target confirmed'
        else:
            msg.level = 'INFO'
            msg.type = 'FALSE_POSITIVE'
            msg.message = 'False detection'

        self._alert_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VerificationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()