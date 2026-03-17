#!/bin/bash

# Source environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting UAV system..."

# Launch all non-interactive nodes
ros2 launch sar_system system.launch.py &

# Give system time to start
sleep 2

echo "Starting Mission Manager..."

# Run mission manager in foreground (so you can type)
ros2 run mission_manager mission_manager