#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run uav_platform sim_platform
