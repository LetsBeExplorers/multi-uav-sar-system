import random
import rclpy
from rclpy.node import Node
from sar_msgs.msg import FSMEvent


class VerificationNode(Node):
    def __init__(self):
        super().__init__('verification_node')

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

        self._fsm_pub = self.create_publisher(
            FSMEvent, f'/{self.uav_id}/fsm/event', 10)

        self.create_subscription(
            FSMEvent, f'/{self.uav_id}/fsm/command',
            self._on_command, 10)

        self._verify_timer = None

    def _on_command(self, msg):
        if msg.uav_id != self.uav_id:
            return
        if msg.event == 'START_VERIFY':
            delay = random.uniform(self.delay_min, self.delay_max)
            if self._verify_timer is not None:
                self._verify_timer.cancel()
            self._verify_timer = self.create_timer(delay, self._decide)

    def _decide(self):
        if self._verify_timer is not None:
            self._verify_timer.cancel()
            self._verify_timer = None

        evt = FSMEvent()
        evt.uav_id = self.uav_id
        evt.timestamp = self.get_clock().now().nanoseconds / 1e9
        evt.event = 'CONFIRMED_TARGET' if random.random() < self.confirm_prob else 'FALSE_POSITIVE'
        self._fsm_pub.publish(evt)


def main(args=None):
    rclpy.init(args=args)
    node = VerificationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()