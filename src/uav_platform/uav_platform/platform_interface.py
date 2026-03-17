import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Safety & Interface Layer between Autonomy and Hardware
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
                ('takeoff_velocity_threshold', 0.3),
                ('landing_velocity_threshold', -0.2),
            ]
        )

        # Retrieve parameter values
        self.uav_name = self.get_parameter('uav_name').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_vertical = self.get_parameter('max_vertical_speed').value
        self.takeoff_thresh = self.get_parameter('takeoff_velocity_threshold').value
        self.landing_thresh = self.get_parameter('landing_velocity_threshold').value

        # Topics
        self.cmd_in_topic = f'/{self.uav_name}/platform/cmd_vel'
        self.driver_topic = f'/{self.uav_name}/driver/cmd_vel'

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, self.driver_topic, 10)
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_in_topic,
            self.process_command,
            10
        )

        # Optional status publisher (for future safety alerts)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # Internal state (LOW-LEVEL)
        self.state = "GROUNDED"

        self.get_logger().debug(f"PlatformInterface ready for {self.uav_name}")

    # Status publishing (unused for now)
    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    # Flight state detection (internal only)
    def update_flight_state(self, vz):

        if vz > self.takeoff_thresh and self.state != "AIRBORNE":
            self.state = "AIRBORNE"
            self.get_logger().debug("AIRBORNE")

        elif vz < self.landing_thresh and self.state != "GROUNDED":
            self.state = "GROUNDED"
            self.get_logger().debug("GROUNDED")

    # Command processing
    def process_command(self, msg):

        safe_msg = Twist()

        # Clamp horizontal velocity
        safe_msg.linear.x = max(min(msg.linear.x, self.max_linear), -self.max_linear)
        safe_msg.linear.y = max(min(msg.linear.y, self.max_linear), -self.max_linear)

        # Clamp vertical velocity
        safe_msg.linear.z = max(min(msg.linear.z, self.max_vertical), -self.max_vertical)

        # Pass through yaw
        safe_msg.angular.z = msg.angular.z

        # Update internal state
        self.update_flight_state(safe_msg.linear.z)

        # Publish safe command
        self.cmd_pub.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlatformInterface()
    rclpy.spin(node)
    rclpy.shutdown()