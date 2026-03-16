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

# config 
UAV_LIST=(x1 x2 x3)
PARAM_FILE="$PWD/src/uav_platform/config/platform.yaml"

# node → package mapping
declare -A NODE_PACKAGE=(
    [platform_interface]=uav_platform
    [gazebo_driver]=uav_platform
    [path_executor]=navigation       # skip for now if not ready
    [swarm_coordinator]=swarm_coordination  # skip for now
)

NODE_LIST=(platform_interface gazebo_driver)

# launch
for UAV in "${UAV_LIST[@]}"; do
    echo "Launching nodes for UAV $UAV..."

    for NODE in "${NODE_LIST[@]}"; do
        PKG=${NODE_PACKAGE[$NODE]}
        ros2 run $PKG $NODE \
            --ros-args --params-file "$PARAM_FILE" \
            -p uav_name:=$UAV \
            -r __node:=${NODE}_$UAV &
    done
done

sleep 3
echo "All current UAV nodes launched for all UAVs."

wait