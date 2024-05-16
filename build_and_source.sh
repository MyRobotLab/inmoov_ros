#!/bin/bash
# build_and_source.sh

cd ~/mrl/inmoov_ros
colcon build --packages-select inmoov_bringup
source install/setup.bash
ros2 launch inmoov_bringup bringup.launch.py
