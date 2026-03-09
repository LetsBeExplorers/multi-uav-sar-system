#!/bin/bash

# Move to workspace root (parent of scripts)
cd "$(dirname "$0")/.."

stop_uavs() {
    echo "Stopping UAV system..."

    # Stop UAV nodes
    pkill -f gazebo_driver
    pkill -f swarm_coordination
    pkill -f mission_manager
    pkill -f navigation

    # Kill stuck ROS runners
    pkill -f "ros2 run"

    # Reset DDS (safe)
    pkill -f fastdds 2>/dev/null
    pkill -f cyclonedds 2>/dev/null

    echo "UAV system stopped."
}

trap stop_uavs SIGINT

echo "Cleaning old UAV processes..."

pkill -f gazebo_driver 2>/dev/null
pkill -f swarm_coordination 2>/dev/null
pkill -f mission_manager 2>/dev/null
pkill -f navigation 2>/dev/null

pkill -f fastdds 2>/dev/null
pkill -f cyclonedds 2>/dev/null

sleep 1

echo "Sourcing ROS 2..."
source /opt/ros/jazzy/setup.bash

echo "Building workspace..."
colcon build --symlink-install

echo "Sourcing workspace..."
source install/setup.bash

sleep 4

echo "Launching UAV platform drivers..."
ros2 run uav_platform gazebo_driver --ros-args -p uav_name:=x3 &

sleep 3

echo "Testing abstraction with sample commands..."
# Rise
ros2 topic pub -1 /x3/cmd_vel geometry_msgs/Twist "{linear: {z: 1.0}}"
sleep 2

# Stop
ros2 topic pub -1 /x3/cmd_vel geometry_msgs/Twist "{linear: {z: 0.0}}"

# Swarm brain (when ready)
# ros2 run swarm_coordination swarm_node &

# Mission manager (when ready)
# ros2 run mission_manager mission_node &

# Navigation stack (future)
# ros2 run navigation nav_node &

echo "UAV simulation running. Press Ctrl+C to stop."

wait