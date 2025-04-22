#!/bin/bash

# Create backup directory with timestamp
BACKUP_DIR="/home/ros2-control/ros_backups/$(date +%Y%m%d_%H%M%S)"
mkdir -p $BACKUP_DIR

echo "Creating backup of current state in $BACKUP_DIR"

# Copy modified files
cp -r /home/ros2-control/KMRIIWA_ROS2_WS/src/kmr_communication/launch $BACKUP_DIR/
cp -r /home/ros2-control/KMRIIWA_ROS2_WS/src/kmr_communication/nodes $BACKUP_DIR/
cp -r /home/ros2-control/KMRIIWA_ROS2_WS/src/kmr_communication/script $BACKUP_DIR/

echo "Backup complete"
