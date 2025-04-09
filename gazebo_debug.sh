#!/bin/bash
export DISPLAY=:1
export GAZEBO_GUI=true
export GZ_GUI=1
export QT_X11_NO_MITSHM=1
export NVIDIA_DRIVER_CAPABILITIES=all

echo "Starting Gazebo with display: $DISPLAY"
gazebo --verbose
