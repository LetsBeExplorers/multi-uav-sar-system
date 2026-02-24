import rclpy
from rclpy.node import Node


class SimulatedUAV(Node):

    def __init__(self):
        # Register this node with ROS
        super().__init__('simulated_uav')

        # Startup message
        self.get_logger().info("Simulated UAV platform started")


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    node = SimulatedUAV()

    try:
        # Keep the node alive and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow clean exit on Ctrl+C
        pass