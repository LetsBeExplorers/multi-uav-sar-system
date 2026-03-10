import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
import time


class PathExecutor(Node):

    def __init__(self):
        super().__init__('path_executor')

        # --- Parameter ---
        self.declare_parameter('uav_name', 'x1')
        uav_name = self.get_parameter('uav_name').value

        # --- Topics ---
        self.cmd_topic = f'/{uav_name}/platform/cmd_vel'
        self.waypoint_topic = f'/{uav_name}/nav/waypoints'

        # --- Publisher ---
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        # --- Subscriber ---
        self.create_subscription(
            PoseArray,
            self.waypoint_topic,
            self.waypoint_callback,
            10
        )

        self.get_logger().info(f"Path Executor ready for {uav_name}")

    # ---------------------------------------------------
    # Waypoint callback
    # ---------------------------------------------------
    def waypoint_callback(self, msg):
        self.get_logger().info(f"Received {len(msg.poses)} waypoints")

        for i, pose in enumerate(msg.poses):
            x = pose.position.x
            y = pose.position.y

            self.get_logger().info(f"Moving to waypoint {i+1}: ({x:.1f}, {y:.1f})")

            # Decide movement direction (simple grid logic)
            cmd = Twist()

            if abs(x) > abs(y):
                if x > 0:
                    cmd.linear.x = 1.0   # forward
                else:
                    cmd.linear.x = -1.0  # backward
            else:
                if y > 0:
                    cmd.linear.y = 1.0   # left
                else:
                    cmd.linear.y = -1.0  # right

            # Move
            self.cmd_pub.publish(cmd)
            time.sleep(2.0)

            # Stop
            self.cmd_pub.publish(Twist())
            time.sleep(1.0)

        self.get_logger().info("Finished all waypoints")


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    rclpy.spin(node)
    rclpy.shutdown()