import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GazeboDriver(Node):

    def __init__(self):
        super().__init__('gazebo_driver')

        # ===== Parameters =====
        self.declare_parameter('uav_name', 'x3')
        self.declare_parameter('max_linear_speed', 3.0)
        self.declare_parameter('max_vertical_speed', 2.0)

        uav_name = self.get_parameter('uav_name').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_vertical = self.get_parameter('max_vertical_speed').value

        self.cmd_topic = f'/model/{uav_name}/cmd_vel'
        self.input_topic = f'/{uav_name}/cmd_vel'

        # ===== Publisher to Gazebo =====
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)

        # ===== Subscriber from Navigation Layer =====
        self.sub = self.create_subscription(
            Twist,
            self.input_topic,
            self.cmd_callback,
            10
        )

        # ===== Flight State =====
        self.flight_state = "GROUNDED"   # GROUNDED | AIRBORNE

        self.get_logger().info(f"GazeboDriver ready for UAV: {uav_name}")

    # ===== Velocity Command Callback =====
    def cmd_callback(self, msg: Twist):

        safe_msg = Twist()

        # --- Safety Clamping ---
        safe_msg.linear.x = max(min(msg.linear.x, self.max_linear), -self.max_linear)
        safe_msg.linear.y = max(min(msg.linear.y, self.max_linear), -self.max_linear)
        safe_msg.linear.z = max(min(msg.linear.z, self.max_vertical), -self.max_vertical)

        safe_msg.angular.z = msg.angular.z

        # --- Basic Flight State Logic ---
        if self.flight_state == "GROUNDED":
            if safe_msg.linear.z > 0.2:
                self.flight_state = "AIRBORNE"
                self.get_logger().info("Takeoff detected → AIRBORNE")

        elif self.flight_state == "AIRBORNE":
            if abs(safe_msg.linear.z) < 0.05 and \
               abs(safe_msg.linear.x) < 0.05 and \
               abs(safe_msg.linear.y) < 0.05:
                self.get_logger().debug("Hovering")

        # Publish safe command to simulator
        self.pub.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboDriver()
    rclpy.spin(node)
    rclpy.shutdown()