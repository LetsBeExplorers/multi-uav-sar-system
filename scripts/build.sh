#!/bin/bash

# Move to workspace root (parent of scripts)
cd "$(dirname "$0")/.."

stop_uavs() {
    echo "Stopping UAV system..."

    # Stop UAV nodes
    pkill -f platform_interface
    pkill -f gazebo_driver
    pkill -f swarm_coordination
    pkill -f mission_manager
    pkill -f navigation

    # Kill stuck ROS runners
    pkill -f "ros2 run"

    # Reset DDS
    pkill -f fastdds 2>/dev/null
    pkill -f cyclonedds 2>/dev/null

    echo "UAV system stopped."
}

trap stop_uavs SIGINT

echo "Cleaning old UAV processes..."

pkill -f platform_interface 2>/dev/null
pkill -f gazebo_driver 2>/dev/null
pkill -f swarm_coordination 2>/dev/null
pkill -f mission_manager 2>/dev/null
pkill -f navigation 2>/dev/null

pkill -f "ros2 run" 2>/dev/null
pkill -f fastdds 2>/dev/null
pkill -f cyclonedds 2>/dev/null

sleep 1

echo "Sourcing ROS 2..."
source /opt/ros/jazzy/setup.bash

echo "Building workspace..."
colcon build --symlink-install

echo "Sourcing workspace..."
source install/setup.bash

sleep 2

echo "Launching UAV platform..."

# Platform Interface (safety + vehicle logic)
ros2 run uav_platform platform_interface \
  --ros-args --params-file "$(pwd)/src/uav_platform/config/platform.yaml" \
  -p uav_name:=x1 &

# Gazebo Driver (simulator adapter)
ros2 run uav_platform gazebo_driver \
  --ros-args --params-file "$(pwd)/src/uav_platform/config/platform.yaml" \
  -p uav_name:=x1 &

sleep 3

echo "Testing platform control..."

# Send command INTO platform layer
ros2 topic pub -1 /x1/platform/cmd_vel geometry_msgs/Twist "{linear: {z: 1.0}}"
sleep 2
ros2 topic pub -1 /x1/platform/cmd_vel geometry_msgs/Twist "{linear: {z: 0.0}}"

echo "UAV platform running. Press Ctrl+C to stop."
wait