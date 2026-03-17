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

        # Sets initial state and where to send status updates
        self.state = "IDLE"
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # Subscriber (status/logs)
        self.create_subscription(
            String,
            '/mission/status',
            self.status_callback,
            10
        )

        self.get_logger().debug("Mission Manager ready")

    # Publishes state
    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    # Print incoming status messages
    def status_callback(self, msg):
        print(msg.data)

    # Send start command
    def send_start(self):
        # Check to see if mission is already running
        if self.state == "RUNNING":
            print("Mission already running")
            return
        # If not, set state
        self.state = "RUNNING"

        # Send status
        self.start_pub.publish(Empty())
        self.publish_status("[MISSION] STARTED")
        self.get_logger().debug("START sent")

    # Send stop command
    def send_stop(self):
        # Check to see if mission is running
        if self.state == "IDLE":
            print("Mission not running")
            return
        # If not, set state
        self.state = "STOPPED"
        
        # Send status
        self.stop_pub.publish(Empty())
        self.publish_status("[MISSION] STOPPED")
        self.get_logger().debug("STOP sent")


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