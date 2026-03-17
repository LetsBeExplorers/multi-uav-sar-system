#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # Publishers (commands)
        self.start_pub = self.create_publisher(String, '/mission/start', 10)
        self.stop_pub = self.create_publisher(String, '/mission/stop', 10)

        # Subscriber (status/logs)
        self.status_sub = self.create_subscription(
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
        msg = String()
        msg.data = "start"
        self.start_pub.publish(msg)
        self.get_logger().info("START sent")

    # Send stop command
    def send_stop(self):
        msg = String()
        msg.data = "stop"
        self.stop_pub.publish(msg)
        self.get_logger().info("STOP sent")


def main():
    rclpy.init()
    node = MissionManager()

    # Spin ROS in background so CLI still works
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    print("Commands: start, stop, exit")

    while True:
        try:
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
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()