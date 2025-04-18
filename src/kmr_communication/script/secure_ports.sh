#!/bin/bash

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}=== Secure ROS2 Communication Ports ===${NC}"
echo -e "This script protects ROS2 communication ports from WiFi access"

# Check for root privileges
if [ "$EUID" -ne 0 ]; then
  echo -e "${RED}Please run as root:${NC} sudo $0"
  exit 1
fi

# Check if UFW is installed
if ! command -v ufw >/dev/null 2>&1; then
  echo -e "${YELLOW}UFW not found. Installing...${NC}"
  apt update && apt install -y ufw
fi

# Define ports to protect
PORTS=(30001 30002 30003 30004 30005 30006 30007)
PROTOCOLS=("tcp" "tcp" "udp" "udp" "tcp" "tcp" "udp")

# Detect network interfaces
WIFI_INTERFACE=""
ETHERNET_INTERFACE=""

echo -e "\n${CYAN}Detecting network interfaces...${NC}"
ip -br addr | grep -v "lo\|docker\|veth" | while read line; do
  IFACE=$(echo $line | awk '{print $1}')
  STATUS=$(echo $line | awk '{print $2}')
  
  # Check if interface is wireless
  if [ -d "/sys/class/net/$IFACE/wireless" ] || [ -L "/sys/class/net/$IFACE/phy80211" ]; then
    echo -e "WiFi interface detected: ${YELLOW}$IFACE${NC}"
    WIFI_INTERFACE="$IFACE"
  else
    # Assume non-wireless interfaces with status UP are Ethernet
    if [ "$STATUS" == "UP" ]; then
      echo -e "Ethernet interface detected: ${GREEN}$IFACE${NC}"
      ETHERNET_INTERFACE="$IFACE"
    fi
  fi
done

# Validate detected interfaces
if [ -z "$ETHERNET_INTERFACE" ]; then
  echo -e "${RED}Error:${NC} Could not detect Ethernet interface."
  echo "Please enter your Ethernet interface name (e.g., eth0, eno1):"
  read ETHERNET_INTERFACE
fi

if [ -z "$WIFI_INTERFACE" ]; then
  echo -e "${YELLOW}Warning:${NC} No WiFi interface detected."
  echo "If you have WiFi, please enter the interface name (e.g., wlan0):"
  read WIFI_INTERFACE
  # If still empty, user doesn't have WiFi or doesn't want to specify
  if [ -z "$WIFI_INTERFACE" ]; then
    echo -e "${GREEN}No WiFi interface specified. Only configuring Ethernet rules.${NC}"
  fi
fi

# Make sure UFW is enabled
echo "y" | ufw enable > /dev/null 2>&1

# Configure UFW rules
echo -e "\n${CYAN}Setting up firewall rules...${NC}"

# First, add specific rules to allow from Ethernet
for i in "${!PORTS[@]}"; do
  PORT=${PORTS[$i]}
  PROTO=${PROTOCOLS[$i]}
  echo -e "Allowing $PORT/$PROTO from Ethernet interface ($ETHERNET_INTERFACE)..."
  ufw allow in on $ETHERNET_INTERFACE proto $PROTO to any port $PORT
done

# Then, if WiFi is detected, deny from WiFi
if [ -n "$WIFI_INTERFACE" ]; then
  for i in "${!PORTS[@]}"; do
    PORT=${PORTS[$i]}
    PROTO=${PROTOCOLS[$i]}
    echo -e "Blocking $PORT/$PROTO from WiFi interface ($WIFI_INTERFACE)..."
    ufw deny in on $WIFI_INTERFACE proto $PROTO to any port $PORT
  done
fi

# Reload UFW to apply changes
ufw reload > /dev/null 2>&1

# Check rules were applied correctly
echo -e "\n${CYAN}Verifying firewall rules:${NC}"
ufw status | grep -E "30001|30002|30003|30004|30005|30006|30007"

echo -e "\n${GREEN}Configuration complete!${NC}"
echo -e "Your ROS2 communication ports are now:"
echo -e " ✓ ${GREEN}Accessible from Ethernet${NC}"
if [ -n "$WIFI_INTERFACE" ]; then
  echo -e " ✓ ${GREEN}Protected from WiFi access${NC}"
fi

echo -e "\nTo bind your ROS2 nodes with maximum compatibility, use:"
echo -e "  ${CYAN}robot_ip = LaunchConfiguration('robot_ip', default='0.0.0.0')${NC}"
echo -e "This allows binding to all interfaces, while the firewall provides protection."
