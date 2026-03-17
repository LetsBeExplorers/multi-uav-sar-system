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

# Single bridge node mapping all UAV cmd_vel topics
ros2 run ros_gz_bridge parameter_bridge \
  /model/x1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /model/x2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /model/x3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /model/x1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /model/x2/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /model/x3/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry &