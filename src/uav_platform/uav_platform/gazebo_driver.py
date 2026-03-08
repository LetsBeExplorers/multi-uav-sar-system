import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GazeboDriver(Node):

    def __init__(self):
        super().__init__('gazebo_driver')

        self.pub = self.create_publisher(
            Twist,
            '/model/x3/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.2, self.loop)
        self.counter = 0

        self.get_logger().info("Gazebo Driver Started")

    def loop(self):
        msg = Twist()

        # First 3 seconds: go up
        if self.counter < 15:
            msg.linear.z = 2.0
            self.get_logger().info("Rising")

        # Next 3 seconds: hover
        elif self.counter < 30:
            msg.linear.z = 0.0
            self.get_logger().info("Hovering")

        # Then descend
        else:
            msg.linear.z = -2.0
            self.get_logger().info("Descending")

        self.pub.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = GazeboDriver()
    rclpy.spin(node)
    rclpy.shutdown()