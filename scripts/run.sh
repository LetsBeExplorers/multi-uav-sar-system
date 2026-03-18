#!/bin/bash

# Kill previous system
echo "Cleaning previous system..."
pkill -f multi-uav-sar-system > /dev/null 2>&1
pkill -f ros2 > /dev/null 2>&1
sleep 2

# Source environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting UAV system..."

# Start launch in background AND track it
ros2 launch sar_system system.launch.py &
LAUNCH_PID=$!

sleep 2

echo "Starting Mission Manager..."

# Run mission manager (foreground)
ros2 run mission_manager mission_manager

# When MissionManager exits → clean shutdown
echo "Shutting down system..."

kill $LAUNCH_PID
pkill -f ros2
sleep 1