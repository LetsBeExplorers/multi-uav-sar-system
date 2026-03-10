import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
import time
import math


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

        # Track where we *think* the drone is
        curr_x = 0.0
        curr_y = 0.0

        for i, pose in enumerate(msg.poses):
            x = pose.position.x
            y = pose.position.y

            # Relative movement needed
            dx = x - curr_x
            dy = y - curr_y

            self.get_logger().info(
                f"Moving to waypoint {i+1}: ({x:.1f}, {y:.1f})"
            )

            cmd = Twist()

            # Decide movement axis
            if abs(dx) > abs(dy):
                cmd.linear.x = 1.0 if dx > 0 else -1.0
            else:
                cmd.linear.y = 1.0 if dy > 0 else -1.0

            # Distance along chosen axis
            distance = abs(dx) if abs(dx) > abs(dy) else abs(dy)

            speed = 1.0
            move_time = distance / speed

            self.get_logger().info(
                f"Distance: {distance:.2f} m, moving {move_time:.2f} s"
            )

            # --- Move ---
            self.cmd_pub.publish(cmd)
            time.sleep(move_time)

            # --- Strong Stop ---
            stop_cmd = Twist()
            for _ in range(15):
                self.cmd_pub.publish(stop_cmd)
                time.sleep(0.1)

            time.sleep(1.0)

            # Update assumed position
            curr_x = x
            curr_y = y

        self.get_logger().info("Finished all waypoints")


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    rclpy.spin(node)
    rclpy.shutdown()