import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PlatformInterface(Node):

    def __init__(self):
        super().__init__('platform_interface')

        # ===== Parameters =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('uav_name', 'x1'),
                ('max_linear_speed', 3.0),
                ('max_vertical_speed', 2.0),
                ('takeoff_altitude_threshold', 0.3),
                ('landing_altitude_threshold', 0.1),
            ]
        )

        # Retrieve parameters
        uav_name = self.get_parameter('uav_name').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_vertical = self.get_parameter('max_vertical_speed').value
        self.takeoff_thresh = self.get_parameter('takeoff_altitude_threshold').value
        self.landing_thresh = self.get_parameter('landing_altitude_threshold').value

        # Topics
        self.cmd_in_topic = f'/{uav_name}/platform/cmd_vel'
        self.driver_topic = f'/{uav_name}/driver/cmd_vel'
        self.state_topic = f'/{uav_name}/state/odom'

        # Publisher to Driver
        self.cmd_pub = self.create_publisher(Twist, self.driver_topic, 10)

        # Subscriber from Navigation
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_in_topic,
            self.process_command,
            10
        )

        # Subscriber from Driver (state feedback)
        self.state_sub = self.create_subscription(
            Odometry,
            self.state_topic,
            self.update_state,
            10
        )

        self.flight_state = "GROUNDED"
        self.altitude = 0.0

        self.get_logger().info(f"PlatformInterface ready for UAV: {uav_name}")

    def update_state(self, msg):
        self.altitude = msg.pose.pose.position.z

        if self.altitude > 0.3 and self.flight_state == "GROUNDED":
            self.flight_state = "AIRBORNE"
            self.get_logger().info("Takeoff confirmed")

        elif self.altitude < 0.1 and self.flight_state == "AIRBORNE":
            self.flight_state = "GROUNDED"
            self.get_logger().info("Landing confirmed")

    def process_command(self, msg):
        safe_msg = Twist()

        # Safety clamping
        safe_msg.linear.x = max(min(msg.linear.x, self.max_linear), -self.max_linear)
        safe_msg.linear.y = max(min(msg.linear.y, self.max_linear), -self.max_linear)
        safe_msg.linear.z = max(min(msg.linear.z, self.max_vertical), -self.max_vertical)
        safe_msg.angular.z = msg.angular.z

        self.cmd_pub.publish(safe_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlatformInterface()
    rclpy.spin(node)
    rclpy.shutdown()