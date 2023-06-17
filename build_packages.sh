#!/bin/bash

echo "Building packages..."

source /opt/ros/humble/setup.bash

cd /home/ros/moveit_debug 

colcon --log-base ../log build --build-base ../build --install-base ../install --symlink-install

source ../install/setup.bash