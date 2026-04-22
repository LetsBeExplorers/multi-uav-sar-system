from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sar_msgs.msg import DriverHealth


class GazeboDriver(Node):

    def __init__(self):
        super().__init__('gazebo_driver')

        # ===== Parameters =====
        self.declare_parameter('uav_name', 'x1')
        uav = self.get_parameter('uav_name').value

        # ===== Publishers =====
        self._cmd_pub = self.create_publisher(Twist, f'/model/{uav}/cmd_vel', 10)
        self._pose_pub = self.create_publisher(Odometry, f'/{uav}/state/odom', 10)
        self._health_pub = self.create_publisher(DriverHealth, f'/{uav}/driver/health', 10)

        # ===== Subscribers =====
        self.create_subscription(Twist, f'/{uav}/driver/cmd_vel', self._forward_command, 10)
        self.create_subscription(Odometry, f'/model/{uav}/odometry', self._forward_state, 10)

        self.get_logger().debug(f'GazeboDriver ready for {uav}')

    # ===== Command Passthrough =====

    def _forward_command(self, msg):
        # Pass velocity command from platform interface to Gazebo
        self._cmd_pub.publish(msg)

    # ===== State Reporting =====

    def _forward_state(self, msg):
        # Republish Gazebo odometry to the system state topic, then report health
        self._pose_pub.publish(msg)
        self._publish_health()

    # ===== Health Reporting =====

    def _publish_health(self):
        # Battery and status are stubbed in sim — real values come from Tello SDK on hardware
        health = DriverHealth()
        health.status = 'OK'
        health.battery = 100.0
        health.timestamp = self.get_clock().now().nanoseconds / 1e9
        self._health_pub.publish(health)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
