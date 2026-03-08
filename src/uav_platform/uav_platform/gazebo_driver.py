import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class GazeboDriver(Node):

    def __init__(self):
        # Initialize ROS node
        super().__init__('gazebo_driver')

        # Publisher that sends motor speed commands to Gazebo (through bridge)
        self.motor_pub = self.create_publisher(
            Float64MultiArray,
            '/X3/gazebo/command/motor_speed',
            10
        )

        # Current motor speed
        self.motor_speed = 0.0

        # Approximate motor speed needed to hover
        self.hover_speed = 560.0

        # Phase controls flight sequence:
        # 0 = takeoff, 1 = hover, 2 = landing, 3 = finished
        self.phase = 0

        # Counter used to stay hovering for a short time
        self.counter = 0

        # Timer runs control loop every 0.5 seconds
        self.timer = self.create_timer(0.5, self.loop)

        self.get_logger().info("Simple Gazebo Driver Started")

    def loop(self):

        # =========================
        # Phase 0 — TAKEOFF
        # Gradually increase motor speed
        # =========================
        if self.phase == 0:
            self.motor_speed += 30.0
            self.get_logger().info(f"Taking off: {self.motor_speed}")

            # Once hover speed reached, switch to hover phase
            if self.motor_speed >= self.hover_speed:
                self.phase = 1

        # =========================
        # Phase 1 — HOVER
        # Stay at same motor speed briefly
        # =========================
        elif self.phase == 1:
            self.counter += 1
            self.get_logger().info("Hovering...")

            # After a few cycles, begin landing
            if self.counter > 6:
                self.phase = 2

        # =========================
        # Phase 2 — LANDING
        # Gradually reduce motor speed
        # =========================
        elif self.phase == 2:
            self.motor_speed -= 30.0
            self.get_logger().info(f"Landing: {self.motor_speed}")

            # Stop motors completely once speed reaches zero
            if self.motor_speed <= 0:
                self.motor_speed = 0.0
                self.phase = 3
                self.get_logger().info("Landed")

        # =========================
        # Publish motor speeds to Gazebo
        # =========================
        msg = Float64MultiArray()
        msg.data = [self.motor_speed] * 4
        self.motor_pub.publish(msg)


def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)

    # Create driver node
    node = GazeboDriver()

    # Keep node running
    rclpy.spin(node)

    # Shutdown cleanly
    rclpy.shutdown()