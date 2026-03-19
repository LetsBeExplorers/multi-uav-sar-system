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

        # QoS so we don't miss the waypoint message if it was sent before this node started
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
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
        self.state = "IDLE"
        self.latest_path_msg = None

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

    def start_path(self, msg):
        # Initialize internal waypoint tracking
        if msg.poses:
            self.waypoints = [pose.pose for pose in msg.poses]
            self.current_index = 0
            self.state = "EXECUTING"
            self.get_logger().debug(f"Starting path with {len(self.waypoints)} waypoints")

    # Store incoming waypoints from A*
    def waypoint_callback(self, msg):
        self.latest_path_msg = msg
        self.start_path(msg)

    def go_home(self):
        if self.home_x is None:
            return

        self.set_state("RETURNING")

        home_pose = Pose()
        home_pose.position.x = self.home_x
        home_pose.position.y = self.home_y

        self.waypoints = [home_pose]
        self.current_index = 0

    # Move toward current waypoint using odometry feedback
    def move_step(self):
        if not self.waypoints:
            # Finished previous path, check if a new one is waiting
            if self.latest_path_msg:
                self.start_path(self.latest_path_msg)
                self.latest_path_msg = None
            else:
                return  # nothing to do

        if self.current_index < len(self.waypoints):
            lookahead = 3 
            target_index = min(self.current_index + lookahead, len(self.waypoints) - 1)
            target = self.waypoints[target_index]
            x, y = target.position.x, target.position.y

            dx = x - self.uav_x
            dy = y - self.uav_y

            cmd = Twist()

            dist = math.hypot(dx, dy)

            if dist > 0:
                cmd.linear.x = dx / dist * 1.0
                cmd.linear.y = dy / dist * 1.0

            self.cmd_pub.publish(cmd)

            # Check if waypoint reached
            if abs(dx) < 0.2 and abs(dy) < 0.2:
                self.current_index += 1

            # Reset once all waypoints are completed
            if self.current_index >= len(self.waypoints):

                # If returning, we're done for real
                if self.state == "RETURNING":
                    self.cmd_pub.publish(Twist())
                    self.state = "IDLE"
                    self.waypoints = []
                    self.latest_path_msg = None
                    self.current_index = 0
                    return

                # Otherwise, mission done → go home
                self.publish_status(f"[{self.uav_name}] DONE")
                self.go_home()
                return

def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()