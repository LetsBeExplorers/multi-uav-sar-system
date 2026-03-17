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

        # Initialize completion variables and send debug message
        self.done_uavs = set()
        self.total_uavs = 3   # match your system
        self.mission_complete = False
        self.get_logger().debug("Mission Manager ready")

    # Publishes state
    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    # Print incoming status messages
    def status_callback(self, msg):
        text = msg.data
        print(text)

        # Detect DONE messages
        if "DONE" in text:
            # Extract UAV name (e.g., [x1])
            uav = text.split("]")[0] + "]"
            self.done_uavs.add(uav)

            # Check if all UAVs finished
            if len(self.done_uavs) == self.total_uavs and not self.mission_complete:
                self.publish_status("[MISSION] COMPLETE")
                self.mission_complete = True

    # Send start command
    def send_start(self):
        # Check to see if mission is already running
        if self.state == "RUNNING":
            print("Mission already running")
            return
        # If not, set state and clear completion status
        self.state = "RUNNING"
        self.done_uavs.clear()
        self.mission_complete = False

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

    # Start ROS spinning in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

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