#!/bin/bash

source /opt/ros/jazzy/setup.bash

echo "Building workspace..."
colcon build --symlink-install

echo "Sourcing workspace..."
source install/setup.bash