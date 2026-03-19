#!/usr/bin/env python3

import time
import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose, PoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from datetime import datetime


# Generates waypoints in coverage pattern
def generate_lawnmower_waypoints(xmin, xmax, ymin, ymax, rows, x_start, x_end):
    poses = []

    height = ymax - ymin
    row_height = height / (rows - 1)

    for row in range(rows):
        y = ymin + row * row_height
        x_positions = [x_start, x_end] if row % 2 == 0 else [x_end, x_start]

        for x in x_positions:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 1.0
            poses.append(pose)

    return poses


class SwarmCoordinator(Node):
    def __init__(self):

        # ==============================
        # Initialization
        # ==============================

        super().__init__('swarm_coordinator')
        self.init_parameters()
        self.init_state()
        self.init_uav_mapping()
        self.init_ros_interfaces()
        self.init_logging()

    # ==============================
    # Initialization Helpers
    # ==============================

    def init_parameters(self):
        self.declare_parameter('uav_id', 'x1')
        self.declare_parameter('num_uavs', 3)
        self.declare_parameter('area_bounds', [-10,10,-10,10])
        self.declare_parameter('rows', 3)

        self.uav_id = self.get_parameter('uav_id').value
        self.num_uavs = self.get_parameter('num_uavs').value
        self.area = self.get_parameter('area_bounds').value
        self.rows = self.get_parameter('rows').value

        # listeners for mission management
        self.state = "IDLE"
        self.create_subscription(Empty, '/mission/start', self.start_cb, 10)
        self.create_subscription(Empty, '/mission/stop', self.stop_cb, 10)

        # Map UAV ID to index for slice assignment
        uav_ids = [f"x{i+1}" for i in range(self.num_uavs)]
        self.uav_index = uav_ids.index(str(self.uav_id))  # ensure it's a string

        # Publisher for this UAV’s waypoints
        topic = f'/{self.uav_id}/nav/waypoints'

        # Debug message and status to mission manager
        self.get_logger().debug(f"Coordinator ready for {self.uav_id}")
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # QoS profile so late subscribers still receive the last waypoint message
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriber to know when waypoints are reached
        self.create_subscription(
            Empty,
            f'/{self.uav_id}/nav/reached_waypoint',
            self.wp_cb,
            10
        )

        # Create publisher with QoS instead of default queue size
        self.publisher = self.create_publisher(PoseArray, topic, qos)

        # Metrics for logging
        self.visited_waypoints = 0
        self.num_waypoints = 0
        self.x_start = 0.0
        self.x_end = 0.0
        self.start_time = None

        # For logging/testing
        self.run_id = int(time.time())
        self.results_file = f"{self.uav_id}_coordination-results.csv"
        self.timer = self.create_timer(2.0, self.log_metrics)

        # Only create header if file does NOT exist
        if not os.path.exists(self.results_file):
            with open(self.results_file, "w") as f:
                f.write("run_id,timestamp,elapsed,state,num_waypoints,x_start,x_end,coverage\n")

    # ==============================
    # Callbacks
    # ==============================

    # Starts the callback loop when it recieves commands
    def start_cb(self, msg):
        # Don't start if we are already started
        if self.state != "IDLE":
            return

        # Reset metrics
        self.visited_waypoints = 0
        self.start_time = time.time()

        # Update state and send debug message        
        self.set_state("SEARCHING")
        self.get_logger().debug("Mission START → publishing waypoints")

        # Generate and send waypoints
        self.publish_waypoints()

    # Sets state when emergency stop is initiated
    def stop_cb(self, msg):
        self.state = "IDLE"

    # Measures # of waypoints reached
    def wp_cb(self, msg):
        self.visited_waypoints += 1

    # ==============================
    # Core Logic
    # ==============================

    # Defines the search area and sends the waypoints to navigation
    def publish_waypoints(self):
        xmin, xmax, ymin, ymax = self.area

        slice_width = (xmax - xmin) / self.num_uavs

        self.x_start = xmin + self.uav_index * slice_width
        self.x_end = xmin + (self.uav_index + 1) * slice_width

        poses = generate_lawnmower_waypoints(
            xmin, xmax, ymin, ymax,
            self.rows,
            self.x_start, self.x_end
        )

        msg = PoseArray()
        msg.header.frame_id = 'world'
        msg.poses = poses

        self.num_waypoints = len(poses)
        self.publisher.publish(msg)

    # ==============================
    # State / Messaging Helpers
    # ==============================

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
        self.publish_status(f"[{self.uav_id}] {self.state}")

    # ==============================
    # Logging / Metrics
    # ==============================

    # Logs data
    def log_metrics(self):
        if self.start_time is None:
            return

        # How long the run took
        elapsed = time.time() - self.start_time
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Measures coverage
        coverage = (
            min(self.visited_waypoints / self.num_waypoints, 1.0)
            if self.num_waypoints > 0 else 0.0
        )

        with open(self.results_file, "a") as f:
            f.write(
                f"{self.run_id},{timestamp},{elapsed:.2f},{self.state},{self.num_waypoints},{self.x_start},{self.x_end},{coverage:.2f}\n"
            )

def main(args=None):
    rclpy.init(args=args)
    node = SwarmCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()