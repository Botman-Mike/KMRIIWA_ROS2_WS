#!/bin/bash

# Change to the repository directory
cd /home/ros2-control/KMRIIWA_ROS2_WS/src/kmr_communication

# View recent commit history to identify the last working commit
echo "Recent commit history:"
git log --oneline -10

# Uncomment and modify the line below with your last known good commit hash
# git reset --hard COMMIT_HASH

echo ""
echo "INSTRUCTIONS:"
echo "1. From the commit log above, identify the last working commit"
echo "2. Edit this script and replace COMMIT_HASH with that commit's hash"
echo "3. Remove the # from the git reset line"
echo "4. Run this script again to restore that version"
echo ""
echo "After restoring, rebuild with:"
echo "cd /home/ros2-control/KMRIIWA_ROS2_WS"
echo "colcon build --packages-select kmr_communication"
echo "source install/setup.bash"
