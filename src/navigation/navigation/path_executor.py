#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import math

class PathExecutor(Node):

    def __init__(self):
        super().__init__('path_executor')

        # UAV name
        self.declare_parameter('uav_name', 'x1')
        uav = self.get_parameter('uav_name').value
        self.uav_name = uav

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, f'/{uav}/platform/cmd_vel', 10)
        self.wp_pub = self.create_publisher(Empty, f'/{self.uav_name}/nav/reached_waypoint', 10)

        # QoS so we don't miss the waypoint message if it was sent before this node started
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Receives waypoint list
        self.create_subscription(
            Path,
            f'/{uav}/nav/planned_path',
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
        self.state = "IDLE"
        self.just_reached = False

        # Initial fallback position (overwritten once odometry is received)
        self.uav_x = 0.0
        self.uav_y = 0.0
        self.home_x = None
        self.home_y = None
        self.timer = self.create_timer(0.1, self.move_step)

        # Debug message and status to/from mission manager
        self.get_logger().debug(f"Path Executor ready for {uav}")
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.create_subscription(Empty, '/mission/stop', self.stop_cb, 10)

    # Publishes node/drone status
    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    # Sets the drone state and sends it to be published in the proper format
    def set_state(self, new_state):
        if self.state == new_state:
            return  # prevent spam

        self.state = new_state
        self.publish_status(f"[{self.uav_name}] {self.state}")

    # Emergency stop
    def stop_cb(self, msg):
        self.waypoints = []
        self.current_index = 0
        self.finished = False

        stop = Twist()
        self.cmd_pub.publish(stop)

        self.set_state("IDLE")

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

        # Ignore if currently running a mission
        if self.state == "EXECUTING":
            return

        if msg.poses:
            self.waypoints = [pose.pose for pose in msg.poses]
            self.current_index = 0

            # Set state and send debug message
            self.state = "EXECUTING"
            self.get_logger().debug(f"Received {len(self.waypoints)} waypoints")
            self.get_logger().info(f"New path received: {len(self.waypoints)} points")

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

            if abs(dx) < 0.2 and abs(dy) < 0.2:

                if not self.just_reached:
                    self.just_reached = True

                    # Stop to prevent drift
                    stop = Twist()
                    self.cmd_pub.publish(stop)

                    # Update current index
                    self.current_index += 1

                    msg = Empty()
                    self.wp_pub.publish(msg)
                    self.get_logger().debug(f"Waypoint {self.current_index}: ({x:.2f}, {y:.2f})")

            else:
                self.just_reached = False

        # Trigger return-to-home ONCE
        if self.current_index >= len(self.waypoints) and not self.finished:

            if self.home_x is None:
                return

            self.set_state("RETURNING")

            home_pose = Pose()
            home_pose.position.x = self.home_x
            home_pose.position.y = self.home_y
            home_pose.position.z = 1.0

            self.waypoints.append(home_pose)
            self.current_index = len(self.waypoints) - 1

            self.finished = True


        # Final completion AFTER returning home
        elif self.finished and self.current_index >= len(self.waypoints):
            self.set_state("DONE")

def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()