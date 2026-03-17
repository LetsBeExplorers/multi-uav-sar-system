#!/bin/bash

cd "$(dirname "$0")/.."

echo "Building workspace..."
colcon build --symlink-install

echo "Sourcing workspace..."
source install/setup.bash