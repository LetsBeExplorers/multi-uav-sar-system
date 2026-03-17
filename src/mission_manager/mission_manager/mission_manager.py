#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
import threading


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # Publishers (commands)
        self.start_pub = self.create_publisher(Empty, '/mission/start', 10)
        self.stop_pub = self.create_publisher(Empty, '/mission/stop', 10)

        # Subscriber (status/logs)
        self.create_subscription(
            String,
            '/mission/status',
            self.status_callback,
            10
        )

        self.get_logger().info("Mission Manager ready")

    # Print incoming status messages
    def status_callback(self, msg):
        print(f"[STATUS] {msg.data}")

    # Send start command
    def send_start(self):
        self.start_pub.publish(Empty())
        self.get_logger().info("START sent")

    # Send stop command
    def send_stop(self):
        self.stop_pub.publish(Empty())
        self.get_logger().info("STOP sent")


def main():
    rclpy.init()
    node = MissionManager()

    print("Commands: start, stop, exit")

    try:
        while rclpy.ok():
            cmd = input(">> ").strip().lower()

            if cmd == "start":
                node.send_start()

            elif cmd == "stop":
                node.send_stop()

            elif cmd == "exit":
                break

            else:
                print("Unknown command")

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()