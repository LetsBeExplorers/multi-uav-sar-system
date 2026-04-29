import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
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
        self.mission_active = False
        self.start_time = None  # set by _on_start; None == no mission yet
        self.warmup_duration = 5.0  # seconds
        self.detection_active = False
        self.current_position = None

        # ===== Publishers =====
        self._detection_pub = self.create_publisher(DetectionEvent, f'/{self.uav_id}/detection/event', 10)
        self._fsm_pub = self.create_publisher(FSMEvent, f'/{self.uav_id}/fsm/event', 10)
        self._alert_pub = self.create_publisher(Alert, '/alerts', 10)

        # ===== Subscribers =====
        self.create_subscription(Empty, '/mission/start', self._on_start, 10)
        self.create_subscription(Empty, '/mission/stop', self._on_stop, 10)
        self.create_subscription(Odometry, f'/{self.uav_id}/state/odom', self._on_odom, 10)

        # ===== Timers =====
        self.create_timer(1.0 / rate, self._tick)

    # ===== Core Loop =====

    def _tick(self):
        if not self.mission_active or self.start_time is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # warmup guard
        if now - self.start_time < self.warmup_duration:
            return

        confidence = self._simulate_detection()

        if confidence < self.confidence_threshold:
            self.consecutive_detections = 0
            self.detection_active = False   # reset state — allows re-trigger
            return

        self.consecutive_detections += 1

        # Recheck in case _on_stop fired during the tick (multi-threaded executor).
        if not self.mission_active:
            return

        if self.consecutive_detections < self.persistence_threshold:
            return

        # detection becomes active (edge trigger)
        if not self.detection_active:
            self._handle_detection(confidence)
            self.detection_active = True

    # ===== Callbacks =====

    def _on_start(self, _msg):
        self.mission_active = True
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.consecutive_detections = 0
        self.detection_active = False

    def _on_stop(self, _msg):
        self.mission_active = False
        self.consecutive_detections = 0
        self.detection_active = False

    def _on_odom(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    # ===== Detection Logic =====

    def _simulate_detection(self):
        if random.random() < self.fake_prob:
            return random.uniform(0.3, 1.0)
        return 0.0

    def _handle_detection(self, confidence):
        now = self.get_clock().now().nanoseconds / 1e9

        self._publish_detection(now, confidence)
        self._publish_fsm_event(now, confidence)
        self._publish_alert(now, confidence)

    # ===== Publishers =====

    def _publish_detection(self, timestamp, confidence):
        msg = DetectionEvent()
        msg.uav_id = self.uav_id
        if self.current_position is None:
            return  # don't publish until we know where we are

        px, py = self.current_position

        # small random offset around UAV
        msg.x = px + random.uniform(-1.0, 1.0)
        msg.y = py + random.uniform(-1.0, 1.0)
        msg.confidence = confidence
        msg.timestamp = timestamp
        self._detection_pub.publish(msg)

    def _publish_fsm_event(self, timestamp, confidence):
        msg = FSMEvent()
        msg.uav_id = self.uav_id
        msg.event = 'DETECTION_EVENT'
        msg.timestamp = timestamp
        msg.value = float(confidence)  # expose confidence to FSM
        self._fsm_pub.publish(msg)

    def _publish_alert(self, timestamp, confidence):
        # No cooldown needed: detection_active edge-trigger already gates repeats.
        msg = Alert()
        msg.uav_id = self.uav_id
        msg.level = 'WARNING'
        msg.type = 'DETECTION'  # matches Alert.msg vocabulary
        msg.message = f'Possible human detected (confidence={confidence:.2f})'
        msg.timestamp = timestamp
        self._alert_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()