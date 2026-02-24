import rclpy
from rclpy.node import Node


class SimulatedUAV(Node):

    def __init__(self):
        # Register this node with ROS
        super().__init__('simulated_uav')

        # Startup message
        self.get_logger().info("Simulated UAV platform started")


def main(args=None):
    # Start ROS
    rclpy.init(args=args)

    # Create node
    node = SimulatedUAV()

    # Keep it running
    rclpy.spin(node)

    # Clean shutdown
    rclpy.shutdown()