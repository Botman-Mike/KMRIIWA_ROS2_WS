#!/bin/bash

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}=== Checking Teleop Command Flow ===${NC}"

# Check what topic the keyboard publishes to
echo -e "\n${CYAN}1. Checking twist_keyboard.py publisher topic:${NC}"
ros2 node info /teleop_twist_keyboard --show-topics 2>/dev/null

# Check for cmd_vel subscribers
echo -e "\n${CYAN}2. Checking subscribers to /cmd_vel:${NC}"
ros2 topic info /cmd_vel -v 2>/dev/null

# Check the commands node is running
echo -e "\n${CYAN}3. Verifying command nodes are running:${NC}"
ros2 node list | grep commands_node

# Echo the cmd_vel topic to see if commands are published
echo -e "\n${CYAN}4. Monitoring cmd_vel messages (press Ctrl+C after using teleop):${NC}"
timeout 10 ros2 topic echo /cmd_vel --once 2>/dev/null || echo -e "${YELLOW}No message received in 10 seconds. Is teleop running?${NC}"

# Check socket connection for command node
echo -e "\n${CYAN}5. Checking TCP socket for command node:${NC}"
PID=$(pgrep -f "kmp_commands_node.py")
if [ -n "$PID" ]; then
    echo -e "${GREEN}KMP commands node running with PID: $PID${NC}"
    echo "Socket connections:"
    lsof -p $PID -a -i -n | grep TCP
    echo "Socket stats:"
    ss -tap | grep $PID
else
    echo -e "${RED}KMP commands node not running${NC}"
fi

# Check if we have diagnostic information
echo -e "\n${CYAN}6. Checking diagnostic info:${NC}"
ros2 topic list | grep diagnostics

echo -e "\n${YELLOW}If teleop isn't causing movement, try the following:${NC}"
echo "1. Verify the robot simulation/hardware is expecting commands on the correct topic"
echo "2. Check if the command_node is properly transforming and forwarding the commands"
echo "3. Run 'ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"linear: {x: 0.1}\"' to test"
echo "4. Examine any error logs in the command node terminal"
