#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray

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

        # Internal state
        self.waypoints = []
        self.current_index = 0

        # Placeholder for UAV position (replace with odometry in real system)
        self.uav_x = 0.0
        self.uav_y = 0.0

        # timer callback for stepping through waypoints
        self.timer = self.create_timer(0.1, self.execute_waypoint)  # 10 Hz

        self.get_logger().info(f"Path Executor ready for {uav}")

    # store incoming waypoints from coordinator
    def waypoint_callback(self, msg):
        if msg.poses:
            self.waypoints = list(msg.poses)
            if self.current_index >= len(self.waypoints):
                self.current_index = 0
            self.get_logger().info(f"Received {len(self.waypoints)} waypoints")

    # move UAV toward the current waypoint one step at a time
    def execute_waypoint(self):
        if self.current_index >= len(self.waypoints):
            return  # nothing to do

        target = self.waypoints[self.current_index]
        x, y = target.position.x, target.position.y

        dx = x - self.uav_x
        dy = y - self.uav_y

        cmd = Twist()

        # move along row first
        cmd.linear.x = 1.0 if x > self.uav_x else -1.0
        cmd.linear.y = 0.0

        # move up to next row
        if abs(self.uav_x - x) < 0.1:
            cmd.linear.x = 0.0
            cmd.linear.y = 1.0 if y > self.uav_y else -1.0

        self.cmd_pub.publish(cmd)

        # simple “reached waypoint” check (distance threshold)
        if abs(dx) < 0.1 and abs(dy) < 0.1:
            self.current_index += 1

        # in a real system, self.uav_x / self.uav_y would be updated from odometry

def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()