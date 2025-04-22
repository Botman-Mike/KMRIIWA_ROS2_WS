#!/bin/bash

# Source ROS environment
source /opt/ros/foxy/setup.bash
source /home/ros2-control/KMRIIWA_ROS2_WS/install/setup.bash

# Run the diagnostic script
python3 /home/ros2-control/KMRIIWA_ROS2_WS/src/kmr_communication/script/diagnose_teleop.py "$@"
