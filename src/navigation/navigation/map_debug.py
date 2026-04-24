import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt


class MapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer')

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/x1/world_model/map',  # <-- adjust if needed
            self.callback,
            10
        )

        plt.ion()
        self.fig, self.ax = plt.subplots()

    def callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        self.ax.clear()

        # Plot map
        self.ax.imshow(data, cmap='gray', origin='lower')

        self.ax.set_title("Occupancy Grid")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")

        plt.draw()
        plt.pause(0.001)


def main():
    rclpy.init()
    node = MapViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()