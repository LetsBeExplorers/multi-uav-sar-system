#!/bin/bash
set -e

echo "Cleaning old simulator processes..."

pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f gazebo 2>/dev/null || true
pkill -9 -f ros_gz_bridge 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true

sleep 1

echo "Sourcing ROS..."
source /opt/ros/jazzy/setup.bash

echo "Launching Gazebo simulation..."

gz sim ../sim/worlds/quadcopter.sdf