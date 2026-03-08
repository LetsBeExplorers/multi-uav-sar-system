import rclpy
from rclpy.node import Node

class SimulatedUAV(Node):

    def __init__(self):
        # Register this node with ROS
        super().__init__('simulated_uav')

        # Startup message
        self.get_logger().info("Simulated UAV platform started")

        # Create a heartbeat that runs every 1 second
        self.timer = self.create_timer(1.0, self.heartbeat)

    def heartbeat(self):
        # Periodic status message to indicate the node is running
        self.get_logger().info("Sim UAV alive")

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