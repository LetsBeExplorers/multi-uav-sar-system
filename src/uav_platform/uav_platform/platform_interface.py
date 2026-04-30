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
        self.current_mode = "IDLE"
        self.last_position = None
        self.last_move_time = self.get_clock().now()

        self.min_movement = 0.05
        self.stuck_timeout = 3.0

        self.is_moving_cmd = False
        self.too_close = False

        self.active_modes = {"SEARCHING", "REFINING", "ASSISTING", "RETURNING"}

        # ===== Publishers =====
        self.cmd_pub = self.create_publisher(Twist, f'/{self.uav_id}/driver/cmd_vel', 10)
        self.event_pub = self.create_publisher(FSMEvent, f'/{self.uav_id}/fsm/event', 10)
        self.alert_pub = self.create_publisher(Alert, '/alerts', 10)

        # ===== Subscribers =====
        self.create_subscription(Twist, f'/{self.uav_id}/platform/cmd_vel', self._on_cmd, 10)
        self.create_subscription(DriverHealth, f'/{self.uav_id}/driver/health', self._on_health, 10)
        self.create_subscription(LaserScan, f'/{self.uav_id}/scan', self._on_scan, 10)
        self.create_subscription(Odometry, f'/{self.uav_id}/state/odom', self._on_odom, 10)
        self.create_subscription(UAVState, '/uav/state', self._on_state, 10)


    # ===== Command Handling =====

    def _on_cmd(self, msg):
        safe = Twist()
        safe.linear.x = max(min(msg.linear.x, self.max_linear), -self.max_linear)
        safe.linear.y = max(min(msg.linear.y, self.max_linear), -self.max_linear)
        safe.linear.z = max(min(msg.linear.z, self.max_vertical), -self.max_vertical)
        safe.angular.z = msg.angular.z

        speed = math.hypot(safe.linear.x, safe.linear.y)
        vertical = abs(safe.linear.z)
        self.is_moving_cmd = (speed > 0.1 or vertical > 0.1)

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

        if min_dist < 1.0:
            if not self.too_close:
                self._alert("COLLISION_RISK", f"Obstacle at {min_dist:.2f}m")
            self.too_close = True
        else:
            self.too_close = False


    # ===== Motion Failure =====

    def _on_odom(self, msg):
        now = self.get_clock().now()
        pos = msg.pose.pose.position

        if self.last_position is None:
            self.last_position = pos
            return

        dx = pos.x - self.last_position.x
        dy = pos.y - self.last_position.y
        dist = math.hypot(dx, dy)

        if dist > self.min_movement:
            self.last_move_time = now
        else:
            dt = (now - self.last_move_time).nanoseconds / 1e9

            if (
                self.current_mode in self.active_modes and
                self.is_moving_cmd and
                dt > self.stuck_timeout
            ):
                self._alert("PATH_FAILED", "No progress toward goal")
                self.last_move_time = now

        self.last_position = pos


    # ===== FSM State =====

    def _on_state(self, msg):
        if msg.uav_id != self.uav_id:
            return
        self.current_mode = msg.state


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
