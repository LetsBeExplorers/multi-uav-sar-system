#!/bin/bash

echo "Cleaning up ROS 2 processes..."

# Kill ROS nodes (covers most cases)
pkill -f ros2 2>/dev/null

# Kill your specific nodes (extra safety)
pkill -f platform_interface 2>/dev/null
pkill -f gazebo_driver 2>/dev/null
pkill -f swarm_coordinator 2>/dev/null
pkill -f path_executor 2>/dev/null
pkill -f mission_manager 2>/dev/null

# Kill DDS middleware (rarely needed, but safe)
pkill -f fastdds 2>/dev/null
pkill -f cyclonedds 2>/dev/null

echo "Cleanup complete."