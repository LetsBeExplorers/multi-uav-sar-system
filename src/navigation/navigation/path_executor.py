#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
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

        # QoS so we don't miss the waypoint message if it was sent before this node started
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Receives waypoint list
        self.create_subscription(
            PoseArray,
            f'/{uav}/nav/waypoints',
            self.waypoint_callback,
            qos
        )

        # Odometry subscription
        self.create_subscription(
            Odometry,
            f'/{uav}/state/odom',
            self.odom_callback,
            10
        )

        # Internal state
        self.waypoints = []
        self.current_index = 0
        self.finished = False
        self.state = "FOLLOW"

        # Initial fallback position (overwritten once odometry is received)
        self.uav_x = 0.0
        self.uav_y = 0.0
        self.home_x = None
        self.home_y = None
        self.timer = self.create_timer(0.1, self.move_step)

        self.get_logger().debug(f"Path Executor ready for {uav}")

    # Update position from odometry
    def odom_callback(self, msg):
        self.uav_x = msg.pose.pose.position.x
        self.uav_y = msg.pose.pose.position.y

        # Capture home position once
        if self.home_x is None:
            self.home_x = self.uav_x
            self.home_y = self.uav_y

    # Store incoming waypoints from coordinator
    def waypoint_callback(self, msg):

        # Ignore if we already have waypoints (prevents restarting mission)
        if self.waypoints:
            return

        if msg.poses:
            self.waypoints = list(msg.poses)
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
            if abs(dx) >= 0.2:
                cmd.linear.x = 1.0 if dx > 0 else -1.0
                cmd.linear.y = 0.0
            else:
                # reached X, now move along Y
                cmd.linear.x = 0.0
                if abs(dy) >= 0.2:
                    cmd.linear.y = 1.0 if dy > 0 else -1.0

            # Publish velocity
            self.cmd_pub.publish(cmd)

            # Check if we've reached the waypoint
            if abs(dx) < 0.2 and abs(dy) < 0.2:

                # Stop to prevent drift
                stop = Twist()
                self.cmd_pub.publish(stop)

                # Update current index
                self.current_index += 1

                # Log the waypoint
                self.get_logger().debug(
                    f"Waypoint {self.current_index}: ({x:.2f}, {y:.2f})"
                )

        # sends the drone home when its done
        if not self.finished and self.current_index >= len(self.waypoints):

            # in case we failed to set a home
            if self.home_x is None:
                return

            self.get_logger().info("Returning to landing pad")

            home_pose = Pose()
            home_pose.position.x = self.home_x
            home_pose.position.y = self.home_y
            home_pose.position.z = 1.0

            self.waypoints.append(home_pose)

            # ensure we actually go to the new waypoint
            self.current_index = len(self.waypoints) - 1

            self.finished = True
            self.state = "RETURN"

def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()