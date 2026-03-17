#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')

        # Parameters
        self.declare_parameter('uav_id', 'x1')
        self.declare_parameter('num_uavs', 3)
        self.declare_parameter('area_bounds', [-10,10,-10,10])
        self.declare_parameter('rows', 3)

        self.uav_id = self.get_parameter('uav_id').value
        self.num_uavs = self.get_parameter('num_uavs').value
        self.area = self.get_parameter('area_bounds').value
        self.rows = self.get_parameter('rows').value

        # Map UAV ID to index for slice assignment
        uav_ids = [f"x{i+1}" for i in range(self.num_uavs)]
        self.uav_index = uav_ids.index(str(self.uav_id))  # ensure it's a string

        # Publisher for this UAV’s waypoints
        topic = f'/{self.uav_id}/nav/waypoints'

        self.get_logger().info(f"Coordinator ready for {self.uav_id}")

        # QoS profile so late subscribers still receive the last waypoint message
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create publisher with QoS instead of default queue size
        self.publisher = self.create_publisher(PoseArray, topic, qos)

        # Publish waypoints once at startup
        self.publish_waypoints()
    

    def publish_waypoints(self):
        xmin, xmax, ymin, ymax = self.area
        width = xmax - xmin
        height = ymax - ymin
        row_height = height / (self.rows - 1)

        # Slice for this UAV
        slice_width = width / self.num_uavs
        x_start = xmin + self.uav_index * slice_width
        x_end = xmin + (self.uav_index + 1) * slice_width

        poses = PoseArray()
        poses.header.frame_id = 'world'

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

        self.publisher.publish(poses)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()