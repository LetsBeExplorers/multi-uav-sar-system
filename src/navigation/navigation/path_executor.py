import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
import time

class PathExecutor(Node):

    def __init__(self):
        super().__init__('path_executor')

        # UAV name
        self.declare_parameter('uav_name', 'x1')
        uav = self.get_parameter('uav_name').value

        # Publishes velocity commands
        self.cmd_pub = self.create_publisher(
            Twist, f'/{uav}/platform/cmd_vel', 10)

        # Receives waypoint list
        self.create_subscription(
            PoseArray,
            f'/{uav}/nav/waypoints',
            self.waypoint_callback,
            10
        )

        self.get_logger().info(f"Path Executor ready for {uav}")

    def waypoint_callback(self, msg):

        if msg.poses:
            curr_x = msg.poses[0].position.x
            curr_y = msg.poses[0].position.y

        for i, pose in enumerate(msg.poses):

            x, y = pose.position.x, pose.position.y
            dx, dy = x - curr_x, y - curr_y

            self.get_logger().info(f"Waypoint {i+1}: ({x:.1f}, {y:.1f})")

            cmd = Twist()

            # Move along dominant axis
            if abs(dx) > abs(dy):
                cmd.linear.x = 1.0 if dx > 0 else -1.0
                distance = abs(dx)
            else:
                cmd.linear.y = 1.0 if dy > 0 else -1.0
                distance = abs(dy)

            move_time = distance / 1.0  # speed = 1 m/s

            # Move
            self.cmd_pub.publish(cmd)
            time.sleep(move_time)

            # Stop (prevents drift)
            stop = Twist()
            for _ in range(10):
                self.cmd_pub.publish(stop)
                time.sleep(0.1)

            curr_x, curr_y = x, y
            time.sleep(1.0)

        self.get_logger().info("Finished waypoints")

def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    rclpy.spin(node)
    rclpy.shutdown()