import random
import rclpy
from rclpy.node import Node
from sar_msgs.msg import DetectionEvent, FSMEvent


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

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
        rate = self.get_parameter('detection_rate_hz').value
        self.fake_prob = self.get_parameter('fake_detection_probability').value

        self.consecutive_detections = 0

        self._detection_pub = self.create_publisher(
            DetectionEvent, f'/{self.uav_id}/detection/event', 10)
        self._fsm_pub = self.create_publisher(
            FSMEvent, f'/{self.uav_id}/fsm/event', 10)

        self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        # STUB: real YOLO replaces this with model inference on a camera frame
        if random.random() < self.fake_prob:
            confidence = random.uniform(0.3, 1.0)
        else:
            confidence = 0.0

        if confidence < self.confidence_threshold:
            self.consecutive_detections = 0
            return

        self.consecutive_detections += 1
        if self.consecutive_detections < self.persistence_threshold:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        det = DetectionEvent()
        det.uav_id = self.uav_id
        det.x = random.uniform(-10, 10)
        det.y = random.uniform(-10, 10)
        det.confidence = confidence
        det.timestamp = now
        self._detection_pub.publish(det)

        evt = FSMEvent()
        evt.uav_id = self.uav_id
        evt.event = 'DETECTION_EVENT'
        evt.timestamp = now
        self._fsm_pub.publish(evt)

        self.consecutive_detections = 0


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()