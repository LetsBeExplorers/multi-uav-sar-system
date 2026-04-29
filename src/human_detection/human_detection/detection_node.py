import random
import rclpy
from rclpy.node import Node
from sar_msgs.msg import DetectionEvent, FSMEvent, Alert


class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        # ===== Parameters =====
        self.declare_parameters(namespace='', parameters=[
            ('uav_id', 'x1'),
            ('confidence_threshold', 0.5),
            ('persistence_threshold', 3),
            ('detection_rate_hz', 5.0),
            ('fake_detection_probability', 0.05),
        ])

        self.uav_id = self.get_parameter('uav_id').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.persistence_threshold = self.get_parameter('persistence_threshold').value
        self.fake_prob = self.get_parameter('fake_detection_probability').value
        rate = self.get_parameter('detection_rate_hz').value

        # ===== State =====
        self.consecutive_detections = 0
        self.last_alert_time = 0.0
        self.alert_cooldown = 2.0  # seconds

        # ===== Publishers =====
        self._detection_pub = self.create_publisher(
            DetectionEvent, f'/{self.uav_id}/detection/event', 10)

        self._fsm_pub = self.create_publisher(
            FSMEvent, f'/{self.uav_id}/fsm/event', 10)

        self._alert_pub = self.create_publisher(
            Alert, '/alerts', 10)

        # ===== Timers =====
        self.create_timer(1.0 / rate, self._tick)

    # ===== Core Loop =====

    def _tick(self):
        confidence = self._simulate_detection()

        if confidence < self.confidence_threshold:
            self.consecutive_detections = 0
            return

        self.consecutive_detections += 1
        if self.consecutive_detections < self.persistence_threshold:
            return

        self._handle_detection(confidence)
        self.consecutive_detections = 0

    # ===== Detection Logic =====

    def _simulate_detection(self):
        if random.random() < self.fake_prob:
            return random.uniform(0.3, 1.0)
        return 0.0

    def _handle_detection(self, confidence):
        now = self.get_clock().now().nanoseconds / 1e9

        self._publish_detection(now, confidence)
        self._publish_fsm_event(now)
        self._publish_alert(now, confidence)

    # ===== Publishers =====

    def _publish_detection(self, timestamp, confidence):
        msg = DetectionEvent()
        msg.uav_id = self.uav_id
        msg.x = random.uniform(-10, 10)
        msg.y = random.uniform(-10, 10)
        msg.confidence = confidence
        msg.timestamp = timestamp
        self._detection_pub.publish(msg)

    def _publish_fsm_event(self, timestamp):
        msg = FSMEvent()
        msg.uav_id = self.uav_id
        msg.event = 'DETECTION_EVENT'
        msg.timestamp = timestamp
        self._fsm_pub.publish(msg)

    def _publish_alert(self, timestamp, confidence):
        if timestamp - self.last_alert_time < self.alert_cooldown:
            return

        msg = Alert()
        msg.uav_id = self.uav_id
        msg.level = 'WARNING'
        msg.type = 'DETECTION'
        msg.message = f'Possible human detected (confidence={confidence:.2f})'
        msg.timestamp = timestamp

        self._alert_pub.publish(msg)
        self.last_alert_time = timestamp


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()