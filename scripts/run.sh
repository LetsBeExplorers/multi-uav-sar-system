#!/bin/bash

# Kill previous system
echo "Cleaning previous system..."
pkill -9 -f "__node:=.*_x[0-9]" > /dev/null 2>&1
pkill -f mission_manager > /dev/null 2>&1
pkill -f ros2 > /dev/null 2>&1
sleep 2

# Source environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting UAV system..."

# Start launch in background
ros2 launch sar_system system.launch.py 2>&1 | grep -v "process has died" &
sleep 2

echo "Starting Mission Manager..."

# Run mission manager (foreground)
ros2 run mission_manager mission_manager --ros-args -p num_uavs:=2

# When MissionManager exits → clean shutdown
echo "Shutting down system..."

pkill -9 -f "__node:=.*_x[0-9]" > /dev/null 2>&1
pkill -f mission_manager > /dev/null 2>&1
pkill -f ros2 > /dev/null 2>&1
sleep 1