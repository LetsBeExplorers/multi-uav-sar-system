import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sar_msgs.msg import DriverHealth, UAVState, Alert, FSMEvent


class PlatformInterface(Node):

    def __init__(self):
        super().__init__('platform_interface')

        # ===== Parameters =====
        self.declare_parameters('', [
            ('uav_id', 'x1'),
            ('max_linear_speed', 3.0),
            ('max_vertical_speed', 2.0),
            ('low_battery_threshold', 20.0),
        ])

        self.uav_id = self.get_parameter('uav_id').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_vertical = self.get_parameter('max_vertical_speed').value
        self.low_battery_thresh = self.get_parameter('low_battery_threshold').value

        # ===== State =====
        self.too_close = False

        # ===== Publishers =====
        self.cmd_pub = self.create_publisher(Twist, f'/{self.uav_id}/driver/cmd_vel', 10)
        self.event_pub = self.create_publisher(FSMEvent, f'/{self.uav_id}/fsm/event', 10)
        self.alert_pub = self.create_publisher(Alert, '/alerts', 10)

        # ===== Subscribers =====
        self.create_subscription(Twist, f'/{self.uav_id}/platform/cmd_vel', self._on_cmd, 10)
        self.create_subscription(DriverHealth, f'/{self.uav_id}/driver/health', self._on_health, 10)
        self.create_subscription(LaserScan, f'/{self.uav_id}/scan', self._on_scan, 10)


    # ===== Command Handling =====

    def _on_cmd(self, msg):
        safe = Twist()
        safe.linear.x = max(min(msg.linear.x, self.max_linear), -self.max_linear)
        safe.linear.y = max(min(msg.linear.y, self.max_linear), -self.max_linear)
        safe.linear.z = max(min(msg.linear.z, self.max_vertical), -self.max_vertical)
        safe.angular.z = msg.angular.z
        self.cmd_pub.publish(safe)


    # ===== Health =====

    def _on_health(self, msg):
        if msg.status == 'COMMS_LOSS':
            self._publish_event('COMMS_LOSS')

        if msg.battery < self.low_battery_thresh:
            self._publish_event('LOW_BATTERY')


    # ===== Collision Risk =====

    def _on_scan(self, msg):
        valid = [r for r in msg.ranges if 0.0 < r < float('inf')]
        if not valid:
            return

        min_dist = min(valid)

        if min_dist < 1.5:
            if not self.too_close:
                self._alert("COLLISION_RISK", f"Obstacle at {min_dist:.2f}m")
            self.too_close = True
        else:
            self.too_close = False

    # ===== Helpers =====

    def _alert(self, typ, msg, level="WARNING"):
        a = Alert()
        a.uav_id = self.uav_id
        a.type = typ
        a.level = level
        a.message = msg
        self.alert_pub.publish(a)

    def _publish_event(self, event):
        e = FSMEvent()
        e.uav_id = self.uav_id
        e.event = event
        e.timestamp = self.get_clock().now().nanoseconds / 1e9
        self.event_pub.publish(e)

def main(args=None):
    rclpy.init(args=args)
    node = PlatformInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
