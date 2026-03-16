#!/usr/bin/env python3
# Minimal Swarm Coordinator for 3 UAVs

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray

class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')

        # Parameters
        self.declare_parameter('uav_list', ['x1', 'x2', 'x3'])
        self.declare_parameter('area_bounds', [-5.0, 5.0, -5.0, 5.0])  # [xmin, xmax, ymin, ymax]
        self.declare_parameter('rows', 3)

        self.uav_list = self.get_parameter('uav_list').value
        self.area = self.get_parameter('area_bounds').value
        self.rows = self.get_parameter('rows').value

        # Create publishers for each UAV
        self.publishers = {uav: self.create_publisher(PoseArray, f'/{uav}/nav/waypoints', 10)
                           for uav in self.uav_list}

        self.get_logger().info(f"SwarmCoordinator ready for UAVs: {self.uav_list}")
        self.publish_waypoints()  # send initial waypoints

    def publish_waypoints(self):
        xmin, xmax, ymin, ymax = self.area
        width = xmax - xmin
        height = ymax - ymin
        row_height = height / self.rows

        for i, uav in enumerate(self.uav_list):
            poses = PoseArray()
            poses.header.frame_id = 'world'

            # Each UAV gets a vertical slice
            x_start = xmin + i * width / len(self.uav_list)
            x_end = xmin + (i + 1) * width / len(self.uav_list)

            # Lawn-mower pattern
            for row in range(self.rows):
                y = ymin + row * row_height
                x_positions = [x_start, x_end] if row % 2 == 0 else [x_end, x_start]
                for x in x_positions:
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = 1.0
                    poses.poses.append(pose)

            self.publishers[uav].publish(poses)
            self.get_logger().info(f"Published {len(poses.poses)} waypoints to {uav}")


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()