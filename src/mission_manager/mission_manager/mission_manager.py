#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Empty, String
import threading
from rclpy.executors import ExternalShutdownException


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # Publishers (commands)
        self.start_pub = self.create_publisher(Empty, '/mission/start', 10)
        self.stop_pub = self.create_publisher(Empty, '/mission/stop', 10)

        # Sets initial state and where to send status updates
        self.state = "IDLE"
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # Dashboard Logic
        self.uav_data = {}
        self.mission_state = "[MISSION] IDLE"

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

        # Determine if mission is complete
        if "DONE" in text:
            uav = text.split("]")[0] + "]"
            self.done_uavs.add(uav)

            if len(self.done_uavs) == self.total_uavs and not self.mission_complete:
                self.mission_complete = True
                self.mission_state = "[MISSION] COMPLETE"

        # Dashboard data parsing
        if text.startswith("[x"):
            uav = text.split("]")[0][1:]  # "x1"
            self.uav_data.setdefault(uav, {})

            if "SEARCHING" in text:
                self.uav_data[uav]["state"] = "SEARCHING"

            elif "RETURNING" in text:
                self.uav_data[uav]["state"] = "RETURNING"

            elif "DONE" in text:
                self.uav_data[uav]["state"] = "DONE"

            elif "PROGRESS" in text:
                prog = text.split("PROGRESS:")[1].strip()
                self.uav_data[uav]["progress"] = prog

            elif "WAYPOINTS" in text:
                count = text.split(":")[-1].strip()
                self.uav_data[uav]["waypoints"] = count

            elif "AREA" in text:
                self.uav_data[uav]["area"] = text.split("] ")[1]

        self.print_dashboard()

    def print_dashboard(self):
        print("\033[H\033[J", end="")
        print("=== MISSION STATUS ===")
        print(self.mission_state)

        # If mission hasn't started yet
        if self.state != "RUNNING":
            print("\nWaiting for start command...")
            return

        # If running but no data yet
        if not self.uav_data:
            print("\nInitializing UAVs...")
            return

        for uav in sorted(self.uav_data.keys()):
            data = self.uav_data[uav]

            state = data.get("state", "-")
            prog = data.get("progress", "-")
            wp = data.get("waypoints", "-")
            area = data.get("area", "")

            print(f"{uav} | {state:<10} | {prog:<10} | waypoints:{wp}")

            if area:
                print(f"   ↳ {area}")

        print("\nCommands: start | stop | exit")

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
        self.mission_state = "[MISSION] RUNNING"
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
        self.mission_state = "[MISSION] STOPPED"
        self.get_logger().debug("STOP sent")

# Background spin thread with exception handling to prevent shutdown errors
def safe_spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass

def main():
    rclpy.init()
    node = MissionManager()

    # Start ROS spinning in background
    spin_thread = threading.Thread(target=safe_spin, args=(node,), daemon=True)
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