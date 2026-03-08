#!/bin/bash
set -e

echo "Sourcing ROS 2..."
source /opt/ros/jazzy/setup.bash

echo "Cleaning old build..."
rm -rf build install log

echo "Building UAV workspace..."
colcon build --symlink-install

echo "Build complete."