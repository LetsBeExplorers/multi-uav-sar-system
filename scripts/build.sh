#!/bin/bash

cd "$(dirname "$0")/.."

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash