#!/bin/bash

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Define the ports to check
PORT_LIST="30001 30002 30003 30004 30005 30006 30007"
ROBOT_IP=${1:-"172.31.1.10"}

echo -e "${CYAN}=== Robot Communication Firewall Check ===${NC}"
echo -e "Checking connectivity to robot at $ROBOT_IP"

# Test basic connectivity
echo -e "\n${CYAN}1. Testing basic connectivity:${NC}"
if ping -c 1 -W 2 $ROBOT_IP >/dev/null 2>&1; then
  echo -e "${GREEN}Robot is reachable via ping${NC}"
else
  echo -e "${RED}Robot is NOT reachable via ping${NC}"
  echo "This might indicate a basic network connectivity issue or ICMP is blocked"
fi

# Check both directions - outbound and inbound
echo -e "\n${CYAN}2. Testing outbound connections (this machine → robot):${NC}"
for port in $PORT_LIST; do
  # Determine protocol based on port
  if [[ "$port" == "30003" || "$port" == "30004" || "$port" == "30007" ]]; then
    PROTO="udp"
  else
    PROTO="tcp"
  fi
  
  # Test outbound connectivity
  if timeout 3 bash -c ">/dev/$PROTO/$ROBOT_IP/$port" 2>/dev/null; then
    echo -e "${GREEN}Outbound $PROTO connection to port $port: SUCCESS${NC}"
  else
    echo -e "${RED}Outbound $PROTO connection to port $port: FAILED${NC}"
  fi
done

echo -e "\n${CYAN}3. Checking if robot's Java application is actively using ports:${NC}"
if command -v nmap >/dev/null 2>&1; then
  # Check with nmap for more detailed port state
  nmap -p 30001-30007 $ROBOT_IP -Pn | grep -E "PORT|30001|30002|30003|30004|30005|30006|30007"
else
  echo -e "${YELLOW}nmap not installed. Install with: sudo apt install nmap${NC}"
fi

echo -e "\n${CYAN}4. Firewall Recommendations:${NC}"
echo -e "${YELLOW}You may need to configure both inbound AND outbound rules on the robot:${NC}"
echo "1. Inbound rules: Allow incoming connections TO the robot on ports 30001-30007"
echo "2. Outbound rules: Allow outgoing connections FROM the robot to your ROS2 machine"
echo -e "${CYAN}For the robot firewall (likely Windows), ensure:${NC}"
echo " - TCP ports 30001, 30002, 30005, 30006 are allowed inbound AND outbound"
echo " - UDP ports 30003, 30004, 30007 are allowed inbound AND outbound"
echo " - The Java application is authorized to use the network"
