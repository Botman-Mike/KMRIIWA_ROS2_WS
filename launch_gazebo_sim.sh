#!/bin/bash

# Set display environment variables
export DISPLAY=:1
export GAZEBO_GUI=true
export GZ_GUI=1

# Print information
echo "Launching Gazebo simulation with display: $DISPLAY"

# Launch the simulation
cd /home/ros2-control/kmriiwa_ws_devel
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kmr_simulation gazebo.launch.py

# Keep the terminal open if the command fails
if [ $? -ne 0 ]; then
    echo "Launch failed. Press Enter to close."
    read
fi
