#!/bin/bash

# Source ROS 2 setup files
source /opt/ros/foxy/setup.bash
source /home/ros2-control/KMRIIWA_ROS2_WS/install/setup.bash

# Directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Run the Python script with the proper environment
python3 ${SCRIPT_DIR}/send_test_move.py "$@"
