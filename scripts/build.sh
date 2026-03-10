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
    pkill -f path_executor

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
pkill -f path_executor 2>/dev/null
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

UAV_NAME=x1

echo "Launching UAV platform..."

# Platform Interface
ros2 run uav_platform platform_interface \
  --ros-args --params-file "$(pwd)/src/uav_platform/config/platform.yaml" \
  -p uav_name:=$UAV_NAME &

# Gazebo Driver
ros2 run uav_platform gazebo_driver \
  --ros-args --params-file "$(pwd)/src/uav_platform/config/platform.yaml" \
  -p uav_name:=$UAV_NAME &

sleep 2

# Path Executor
ros2 run navigation path_executor \
  --ros-args -p uav_name:=$UAV_NAME &

sleep 3

echo "Sending test waypoints..."

ros2 topic pub --once /$UAV_NAME/nav/waypoints geometry_msgs/PoseArray "
poses:
- position: {x: 3.0, y: 0.0, z: 0.0}
- position: {x: 3.0, y: 3.0, z: 0.0}
- position: {x: 0.0, y: 3.0, z: 0.0}
- position: {x: 0.0, y: 0.0, z: 0.0}
"

wait