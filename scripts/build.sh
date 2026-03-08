#!/bin/bash

stop_uavs() {
    echo "Stopping UAV system..."

    # Stop UAV nodes
    pkill -f sim_platform
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

pkill -f sim_platform 2>/dev/null
pkill -f swarm_coordination 2>/dev/null
pkill -f mission_manager 2>/dev/null
pkill -f navigation 2>/dev/null

pkill -f fastdds 2>/dev/null
pkill -f cyclonedds 2>/dev/null

sleep 1

echo "Sourcing ROS..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

sleep 2

echo "Launching UAV nodes..."

# Platform layer
ros2 run uav_platform sim_platform &

# Swarm brain (when ready)
# ros2 run swarm_coordination swarm_node &

# Mission manager (when ready)
# ros2 run mission_manager mission_node &

# Navigation stack (future)
# ros2 run navigation nav_node &

echo "UAV simulation running. Press Ctrl+C to stop."

wait