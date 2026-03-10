import rclpy
from rclpy.node import Node
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
        uav_name = self.get_parameter('uav_name').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_vertical = self.get_parameter('max_vertical_speed').value
        self.takeoff_vel_thresh = self.get_parameter('takeoff_velocity_threshold').value
        self.landing_vel_thresh = self.get_parameter('landing_velocity_threshold').value

        # Topics
        self.cmd_in_topic = f'/{uav_name}/platform/cmd_vel'
        self.driver_topic = f'/{uav_name}/driver/cmd_vel'

        # Publish information to the drivers
        self.cmd_pub = self.create_publisher(Twist, self.driver_topic, 10)

        # Subscriber: receives high-level velocity commands
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_in_topic,
            self.process_command,
            10
        )

        # Set starting state
        self.flight_state = "GROUNDED"
        self.get_logger().info(f"PlatformInterface ready for UAV: {uav_name}")

    # ===== State Machine Based on Commanded Motion =====
    def update_flight_state(self, vertical_velocity):

        # Detect takeoff
        if vertical_velocity > self.takeoff_vel_thresh:
            if self.flight_state != "AIRBORNE":
                self.flight_state = "AIRBORNE"
                self.get_logger().info("Takeoff detected → AIRBORNE")

        # Detect landing
        elif vertical_velocity < self.landing_vel_thresh:
            if self.flight_state != "GROUNDED":
                self.flight_state = "GROUNDED"
                self.get_logger().info("Landing detected → GROUNDED")

    # ===== Command Processing =====
    def process_command(self, msg):
        safe_msg = Twist()

        # Clamp horizontal velocities to prevent unsafe speeds
        safe_msg.linear.x = max(min(msg.linear.x, self.max_linear), -self.max_linear)
        safe_msg.linear.y = max(min(msg.linear.y, self.max_linear), -self.max_linear)

        # Clamp vertical velocity separately (typically stricter limits)
        safe_msg.linear.z = max(min(msg.linear.z, self.max_vertical), -self.max_vertical)

        # Allow yaw rotation without modification
        safe_msg.angular.z = msg.angular.z

        # Update flight state based on commanded vertical motion
        self.update_flight_state(safe_msg.linear.z)

        # Publish the safe command to the UAV driver
        self.cmd_pub.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlatformInterface()
    rclpy.spin(node)
    rclpy.shutdown()