import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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

        uav_name = self.get_parameter('uav_name').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_vertical = self.get_parameter('max_vertical_speed').value
        self.takeoff_vel_thresh = self.get_parameter('takeoff_velocity_threshold').value
        self.landing_vel_thresh = self.get_parameter('landing_velocity_threshold').value

        # Topics
        self.cmd_in_topic = f'/{uav_name}/platform/cmd_vel'
        self.driver_topic = f'/{uav_name}/driver/cmd_vel'

        self.cmd_pub = self.create_publisher(Twist, self.driver_topic, 10)

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_in_topic,
            self.process_command,
            10
        )

        self.flight_state = "GROUNDED"
        self.get_logger().info(f"PlatformInterface ready for UAV: {uav_name}")

    # ===== State Machine Based on Commanded Motion =====
    def update_flight_state(self, vertical_velocity):

        if vertical_velocity > self.takeoff_vel_thresh:
            if self.flight_state != "AIRBORNE":
                self.flight_state = "AIRBORNE"
                self.get_logger().info("Takeoff detected → AIRBORNE")

        elif vertical_velocity < self.landing_vel_thresh:
            if self.flight_state != "GROUNDED":
                self.flight_state = "GROUNDED"
                self.get_logger().info("Landing detected → GROUNDED")

    # ===== Command Processing =====
    def process_command(self, msg):
        safe_msg = Twist()

        safe_msg.linear.x = max(min(msg.linear.x, self.max_linear), -self.max_linear)
        safe_msg.linear.y = max(min(msg.linear.y, self.max_linear), -self.max_linear)
        safe_msg.linear.z = max(min(msg.linear.z, self.max_vertical), -self.max_vertical)
        safe_msg.angular.z = msg.angular.z

        # Update state from commanded vertical motion
        self.update_flight_state(safe_msg.linear.z)

        self.cmd_pub.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlatformInterface()
    rclpy.spin(node)
    rclpy.shutdown()