#!/bin/bash

# Script to launch KUKA KMR communication with better waiting and coordination

echo "Starting KUKA KMR communication..."
echo "Waiting for network connectivity to robot..."

# Function to check if robot is reachable
function check_robot_ping() {
    ping -c 1 -W 1 $1 > /dev/null 2>&1
    return $?
}

# Get robot IP from args or use default
ROBOT_IP=${1:-"172.31.1.206"}
RETRY_COUNT=0
MAX_RETRIES=30

# Try to ping the robot before launching
while ! check_robot_ping $ROBOT_IP; do
    if [ $RETRY_COUNT -ge $MAX_RETRIES ]; then
        echo "Warning: Robot at $ROBOT_IP not responding to ping after $MAX_RETRIES attempts."
        echo "Continuing anyway, nodes will retry connection as needed..."
        break
    fi
    
    echo "Waiting for robot at $ROBOT_IP to respond... (attempt $RETRY_COUNT/$MAX_RETRIES)"
    RETRY_COUNT=$((RETRY_COUNT+1))
    sleep 2
done

if [ $RETRY_COUNT -lt $MAX_RETRIES ]; then
    echo "Robot is responsive at $ROBOT_IP"
fi

echo "Launching ROS2 communication nodes..."
echo "Note: Nodes will wait up to 30 seconds before reporting connection issues."

# Launch with specified IP
ros2 launch kmr_communication sunrise_communication.launch.py robot_ip:=$ROBOT_IP

# Exit with the exit code from the ros2 launch command
exit $?
