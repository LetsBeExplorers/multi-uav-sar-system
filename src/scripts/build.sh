#!/bin/bash

# Stop on error
set -e

echo "Sourcing ROS..."
source /opt/ros/jazzy/setup.bash

echo "Building workspace..."
colcon build

echo "Done."