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
gz sim sim/worlds/quadcopter.sdf &

sleep 3

echo "Starting ROS-Gazebo bridge..."
ros2 run ros_gz_bridge parameter_bridge \
/model/x3/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist &