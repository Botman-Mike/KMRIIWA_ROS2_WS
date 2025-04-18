#!/bin/bash

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Define the ports needed for KMR communication
PORT_LIST="30001:tcp 30002:tcp 30003:udp 30004:udp 30005:tcp 30006:tcp 30007:udp"

echo -e "${CYAN}=== KUKA KMR Firewall Configuration ===${NC}"
echo -e "This script will configure your firewall to allow the necessary ports for KMR communication."

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo -e "${YELLOW}Please run this script with sudo:${NC}"
  echo "sudo $0"
  exit 1
fi

# Check which firewall is in use
HAS_UFW=false
HAS_FIREWALLD=false

if command -v ufw >/dev/null 2>&1; then
  HAS_UFW=true
  echo -e "Detected ${CYAN}UFW${NC} (Uncomplicated Firewall)"
fi

if command -v firewall-cmd >/dev/null 2>&1; then
  HAS_FIREWALLD=true
  echo -e "Detected ${CYAN}firewalld${NC}"
fi

if ! $HAS_UFW && ! $HAS_FIREWALLD; then
  echo -e "${YELLOW}No supported firewall detected.${NC}"
  echo "Would you like to install UFW (Uncomplicated Firewall)? (y/n)"
  read -r install_fw
  if [[ $install_fw == "y" || $install_fw == "Y" ]]; then
    echo "Installing UFW..."
    apt update
    apt install -y ufw
    HAS_UFW=true
  else
    echo "No firewall will be configured. Exiting."
    exit 0
  fi
fi

# Configure UFW if present
if $HAS_UFW; then
  echo -e "\n${CYAN}Configuring UFW...${NC}"
  
  # Check if UFW is enabled
  if ! ufw status | grep -q "Status: active"; then
    echo "UFW is not enabled. Enabling UFW..."
    echo "y" | ufw enable
  fi
  
  # Add rules for each port
  for port_info in $PORT_LIST; do
    port=${port_info%%:*}
    proto=${port_info#*:}
    
    echo -e "Adding rule for port ${CYAN}$port/$proto${NC}..."
    ufw allow $port/$proto comment "KUKA KMR Communication"
  done
  
  # Check status after adding rules
  echo -e "\n${CYAN}UFW Status:${NC}"
  ufw status verbose | grep -E "Status:|$port/(tcp|udp)"
fi

# Configure firewalld if present
if $HAS_FIREWALLD; then
  echo -e "\n${CYAN}Configuring firewalld...${NC}"
  
  # Check if firewalld is running
  if ! systemctl is-active --quiet firewalld; then
    echo "firewalld is not running. Starting firewalld..."
    systemctl start firewalld
  fi
  
  # Get active zone
  ZONE=$(firewall-cmd --get-default-zone)
  echo -e "Using firewall zone: ${CYAN}$ZONE${NC}"
  
  # Add rules for each port
  for port_info in $PORT_LIST; do
    port=${port_info%%:*}
    proto=${port_info#*:}
    
    echo -e "Adding rule for port ${CYAN}$port/$proto${NC}..."
    firewall-cmd --zone=$ZONE --add-port=$port/$proto --permanent
  done
  
  # Reload firewall to apply changes
  echo "Reloading firewall rules..."
  firewall-cmd --reload
  
  # Check status after adding rules
  echo -e "\n${CYAN}firewalld Status:${NC}"
  firewall-cmd --list-ports | grep -E "30001|30002|30003|30004|30005|30006|30007"
fi

# Check if networking is configured to accept incoming connections
echo -e "\n${CYAN}Checking iptables policies...${NC}"
if iptables -L INPUT | grep -q "policy DROP"; then
  echo -e "${YELLOW}Warning: Default input policy is set to DROP.${NC}"
  echo "This means incoming connections are blocked by default."
  echo "The firewall rules we added should override this, but if you still have"
  echo "connection issues, you might need to check your iptables configuration."
fi

# Verify port access with netcat if available
if command -v nc >/dev/null 2>&1; then
  echo -e "\n${CYAN}Testing port access...${NC}"
  echo "Note: This will only test if the ports can be bound locally."
  echo "It doesn't verify external accessibility."
  
  for port_info in $PORT_LIST; do
    port=${port_info%%:*}
    proto=${port_info#*:}
    
    if [ "$proto" = "tcp" ]; then
      # Try to start a listener (will fail if port is in use)
      nc -l -p $port -w 1 > /dev/null 2>&1 &
      NC_PID=$!
      sleep 1
      
      # Check if netcat is still running
      if kill -0 $NC_PID 2>/dev/null; then
        echo -e "Port ${GREEN}$port/tcp${NC} is ${GREEN}accessible${NC}"
        kill $NC_PID 2>/dev/null
      else
        echo -e "Port ${RED}$port/tcp${NC} might be ${RED}in use${NC} or ${RED}blocked${NC}"
      fi
    elif [ "$proto" = "udp" ]; then
      # UDP tests are more complex and less reliable, so we'll skip for now
      echo -e "Port ${YELLOW}$port/udp${NC} - UDP port testing not implemented"
    fi
  done
fi

echo -e "\n${GREEN}Firewall configuration complete.${NC}"
echo "If you still encounter connection issues, consider temporarily disabling"
echo "your firewall completely to determine if it's a firewall-related problem:"
if $HAS_UFW; then
  echo "  sudo ufw disable"
  echo "  # To re-enable later:"
  echo "  sudo ufw enable"
fi
if $HAS_FIREWALLD; then
  echo "  sudo systemctl stop firewalld"
  echo "  # To re-enable later:"
  echo "  sudo systemctl start firewalld"
fi
