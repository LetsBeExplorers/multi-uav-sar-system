import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sar_msgs.msg import DriverHealth, FSMEvent, Alert


class PlatformInterface(Node):

    def __init__(self):
        super().__init__('platform_interface')

        # ===== Parameters =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('uav_id', 'x1'),
                ('max_linear_speed', 3.0),
                ('max_vertical_speed', 2.0),
                ('takeoff_velocity_threshold', 0.3),
                ('landing_velocity_threshold', -0.2),
                ('low_battery_threshold', 20.0),
            ]
        )

        self.uav_id = self.get_parameter('uav_id').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_vertical = self.get_parameter('max_vertical_speed').value
        self.takeoff_thresh = self.get_parameter('takeoff_velocity_threshold').value
        self.landing_thresh = self.get_parameter('landing_velocity_threshold').value
        self.low_battery_thresh = self.get_parameter('low_battery_threshold').value

        # ===== State =====
        self.flight_state = 'GROUNDED'
        self.too_close = False
        self.last_position = None
        self.last_move_time = self.get_clock().now()
        self.stuck_timeout = 3.0   # shorter = more responsive
        self.min_movement = 0.05   # smaller since odom is precise

        self.is_moving_cmd = False

        # ===== Publishers =====
        self._cmd_pub = self.create_publisher(Twist, f'/{self.uav_id}/driver/cmd_vel', 10)
        self._event_pub = self.create_publisher(FSMEvent, f'/{self.uav_id}/fsm/event', 10)
        self.alert_pub = self.create_publisher(Alert, '/alerts', 10)

        # ===== Subscribers =====
        self.create_subscription(
            Twist, f'/{self.uav_id}/platform/cmd_vel', self._process_command, 10)
        self.create_subscription(
            DriverHealth, f'/{self.uav_id}/driver/health', self._on_health_update, 10)
        self.create_subscription(
            LaserScan, f'/{self.uav_id}/scan', self._on_scan, 10)
        self.create_subscription(
            Odometry, f'/{self.uav_id}/state/odom', self._on_odom, 10)

    # ===== Command Processing =====

    def _process_command(self, msg):

        # Clamp all velocity components to safe limits before forwarding to driver
        safe = Twist()
        safe.linear.x = max(min(msg.linear.x, self.max_linear), -self.max_linear)
        safe.linear.y = max(min(msg.linear.y, self.max_linear), -self.max_linear)
        safe.linear.z = max(min(msg.linear.z, self.max_vertical), -self.max_vertical)
        safe.angular.z = msg.angular.z

        self._update_flight_state(safe.linear.z)

        # detect if UAV is trying to move
        speed = math.hypot(safe.linear.x, safe.linear.y)
        vertical = abs(safe.linear.z)

        if speed > 0.1 or vertical > 0.1:
            self.is_moving_cmd = True
        else:
            self.is_moving_cmd = False

        self._cmd_pub.publish(safe)

    # ===== Flight State =====

    def _update_flight_state(self, vz):
        # Track whether the UAV is airborne based on vertical velocity
        if vz > self.takeoff_thresh and self.flight_state != 'AIRBORNE':
            self.flight_state = 'AIRBORNE'
            self.get_logger().debug('AIRBORNE')
        elif vz < self.landing_thresh and self.flight_state != 'GROUNDED':
            self.flight_state = 'GROUNDED'
            self.get_logger().debug('GROUNDED')

    # ===== Health Monitoring =====
    # TODO: add COLLISION_RISK and PATH_FAILED events once sensor input is wired in

    def _on_health_update(self, msg):
        # Translate hardware issues into FSM safety events
        if msg.status == 'COMMS_LOSS':
            self._publish_event('COMMS_LOSS')
        if msg.battery < self.low_battery_thresh:
            self._publish_event('LOW_BATTERY')

    def _on_scan(self, msg):
        # ignore invalid readings (0 or inf)
        valid_ranges = [r for r in msg.ranges if r > 0.0 and r < float('inf')]

        if not valid_ranges:
            self.too_close = False
            return

        min_dist = min(valid_ranges)

        # threshold (tune this later)
        if min_dist < 1.0:
            self.too_close = True
        else:
            self.too_close = False

    def _publish_alert(self, alert_type, message, level="WARNING"):
        msg = Alert()
        msg.uav_id = self.uav_id
        msg.type = alert_type
        msg.level = level
        msg.message = message
        self.alert_pub.publish(msg)

    # ===== Helpers =====

    def _on_odom(self, msg):
        current_time = self.get_clock().now()
        pos = msg.pose.pose.position

        if self.last_position is not None:
            dx = pos.x - self.last_position.x
            dy = pos.y - self.last_position.y
            dist = math.hypot(dx, dy)

            if dist > self.min_movement:
                # UAV is moving → good
                self.last_move_time = current_time
            else:
                # UAV not moving
                dt = (current_time - self.last_move_time).nanoseconds / 1e9

                if self.is_moving_cmd and dt > self.stuck_timeout:
                    self._publish_alert("COLLISION", "UAV stuck while moving")
                    self.last_move_time = current_time

        self.last_position = pos

    def _publish_event(self, event):
        msg = FSMEvent()
        msg.uav_id = self.uav_id
        msg.event = event
        msg.timestamp = self.get_clock().now().nanoseconds / 1e9
        msg.value = 0.0
        self._event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlatformInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
