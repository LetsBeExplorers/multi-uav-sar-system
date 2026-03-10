#!/bin/bash
set -e

# Move to workspace root (parent of scripts)
cd "$(dirname "$0")/.."

echo "Cleaning old simulator processes..."

pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f gazebo 2>/dev/null || true
pkill -9 -f ros_gz_bridge 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true

sleep 1

echo "Sourcing ROS..."
source /opt/ros/jazzy/setup.bash

echo "Launching Gazebo simulation..."
WORLD_FILE=${1:-quadcopter.sdf}
gz sim "$(pwd)/sim/worlds/$WORLD_FILE" &

sleep 3

echo "Starting ROS-Gazebo bridge..."
NUM_UAVS=1

for i in $(seq 1 $NUM_UAVS)
do
  UAV="x$i"
  ros2 run ros_gz_bridge parameter_bridge \
  /model/$UAV/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist &
done