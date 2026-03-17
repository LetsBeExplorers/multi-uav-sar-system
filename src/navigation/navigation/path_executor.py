#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
import time
import math

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

        # Odometry subscription
        self.create_subscription(
            Odometry,
            f'/{uav}/odom',
            self.odom_callback,
            10
        )

        # Internal state
        self.waypoints = []
        self.current_index = 0

        # Initialize UAV position at its pad
        self.uav_x, self.uav_y = self.get_starting_pad(uav)
        self.speed = 2.0  # meters per second (used for dynamic sleep)
        self.timer = self.create_timer(0.1, self.move_step)

        self.get_logger().info(f"Path Executor ready for {uav}")

    # Hacky WARNING - dont try this at home
    def get_starting_pad(self, uav):
        if uav == 'x1':
            return -3.0, -11.0
        elif uav == 'x2':
            return 0.0, -11.0
        elif uav == 'x3':
            return 3.0, -11.0
        else:
            return 0.0, 0.0  # fallback

    # Update position from odometry
    def odom_callback(self, msg):
        self.uav_x = msg.pose.pose.position.x
        self.uav_y = msg.pose.pose.position.y

    # Store incoming waypoints from coordinator
    def waypoint_callback(self, msg):
        if msg.poses:
            self.waypoints = list(msg.poses)

            # Reset index if we got a new list
            if self.current_index >= len(self.waypoints):
                self.current_index = 0

            self.get_logger().info(f"Received {len(self.waypoints)} waypoints")

    # Move toward current waypoint using odometry feedback
    def move_step(self):

        if not self.waypoints:
            return

        if self.current_index < len(self.waypoints):

            target = self.waypoints[self.current_index]
            x, y = target.position.x, target.position.y

            dx = x - self.uav_x
            dy = y - self.uav_y

            cmd = Twist()

            # Row-first lawn-mower logic: always move along X first
            if abs(x - self.uav_x) >= 0.1:  # still need threshold check
                cmd.linear.x = 1.0 if x > self.uav_x else -1.0
                cmd.linear.y = 0.0
            else:
                # reached X, now move along Y
                cmd.linear.x = 0.0
                if abs(y - self.uav_y) >= 0.1:
                    cmd.linear.y = 1.0 if y > self.uav_y else -1.0

            # Publish velocity
            self.cmd_pub.publish(cmd)

            # Check if we've reached the waypoint (using real position now)
            if abs(self.uav_x - x) < 0.2 and abs(self.uav_y - y) < 0.2:

                # Stop to prevent drift
                stop = Twist()
                self.cmd_pub.publish(stop)

                # Update current index
                self.current_index += 1

                # Log the waypoint
                self.get_logger().info(
                    f"Waypoint {self.current_index}: ({x:.2f}, {y:.2f})"
                )

        if self.current_index >= len(self.waypoints) and self.waypoints:
            self.get_logger().info("Finished all waypoints")

def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()