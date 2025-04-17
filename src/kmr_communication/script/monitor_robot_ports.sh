#!/bin/bash

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Define the ports to monitor
PORT_LIST="30001 30002 30003 30004 30005 30006 30007"
PORT_NAMES=(
  "30001:KMP Status (TCP)" 
  "30002:KMP Command (TCP)" 
  "30003:KMP Laser (UDP)" 
  "30004:KMP Odometry (UDP)" 
  "30005:LBR Command (TCP)" 
  "30006:LBR Status (TCP)" 
  "30007:LBR Sensors (UDP)"
)

# Function to get port name from number
get_port_name() {
  local port=$1
  for entry in "${PORT_NAMES[@]}"; do
    if [[ $entry == "$port:"* ]]; then
      echo "${entry#*:}"
      return
    fi
  done
  echo "Unknown"
}

# Function to check if running in a terminal
is_terminal() {
  [ -t 0 ]
}

# Function to monitor a specific port
check_port() {
  port=$1
  # Check if the port is in use with netstat
  if netstat -tuln | grep -q ":$port "; then
    # Get the protocol (TCP/UDP)
    proto=$(netstat -tuln | grep ":$port " | awk '{print $1}')
    # Get the state if available
    state=$(netstat -tuln | grep ":$port " | awk '{print $6}')
    # Get the PID using the port
    pid=$(lsof -i :$port -t 2>/dev/null)
    
    if [ -n "$pid" ]; then
      # Get process name using the PID
      pname=$(ps -p $pid -o comm= 2>/dev/null)
      echo -e "${GREEN}Port $port ($(get_port_name $port))${NC}: ACTIVE - $proto ${YELLOW}PID:$pid${NC} ($pname)"
    else
      echo -e "${GREEN}Port $port ($(get_port_name $port))${NC}: ACTIVE - $proto"
    fi
  else
    echo -e "${RED}Port $port ($(get_port_name $port))${NC}: INACTIVE"
  fi
}

# Function to check robot connectivity
check_robot_connection() {
  robot_ip=$1
  if ping -c 1 -W 1 $robot_ip >/dev/null 2>&1; then
    echo -e "${GREEN}Robot at $robot_ip is reachable${NC}"
  else
    echo -e "${RED}Robot at $robot_ip is NOT reachable${NC}"
  fi
}

show_usage() {
  echo "Usage: $0 [options]"
  echo "Options:"
  echo "  -h, --help          Show this help message"
  echo "  -m, --monitor       Monitor ports continuously (updates every 2 seconds)"
  echo "  -i, --ip IP         Specify robot IP (default: 172.31.1.206)"
  echo "  -c, --check-robot   Check if robot is reachable via ping"
  echo "  -v, --verbose       Show more detailed information"
  echo "  -p, --port PORT     Check a specific port"
  echo ""
  echo "Example: $0 -m -i 172.31.1.206 -c"
}

# Parse command line arguments
MONITOR=false
ROBOT_IP="172.31.1.206"
CHECK_ROBOT=false
VERBOSE=false
SPECIFIC_PORT=""

while [[ $# -gt 0 ]]; do
  case $1 in
    -h|--help)
      show_usage
      exit 0
      ;;
    -m|--monitor)
      MONITOR=true
      shift
      ;;
    -i|--ip)
      ROBOT_IP="$2"
      shift 2
      ;;
    -c|--check-robot)
      CHECK_ROBOT=true
      shift
      ;;
    -v|--verbose)
      VERBOSE=true
      shift
      ;;
    -p|--port)
      SPECIFIC_PORT="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      show_usage
      exit 1
      ;;
  esac
done

# Main function to run the checks
run_checks() {
  clear
  echo -e "${CYAN}=== KUKA KMR Port Monitor ===${NC}"
  echo -e "Time: $(date)"
  
  if [ "$CHECK_ROBOT" = true ]; then
    echo -e "\n${CYAN}Robot Connection:${NC}"
    check_robot_connection $ROBOT_IP
  fi
  
  echo -e "\n${CYAN}Port Status:${NC}"
  if [ -n "$SPECIFIC_PORT" ]; then
    check_port $SPECIFIC_PORT
  else
    for port in $PORT_LIST; do
      check_port $port
    done
  fi
  
  if [ "$VERBOSE" = true ] && [ -z "$SPECIFIC_PORT" ]; then
    echo -e "\n${CYAN}Full netstat output:${NC}"
    netstat -tuln | grep -E "State|:3000[1-7] "
    
    echo -e "\n${CYAN}Active connections (established):${NC}"
    netstat -tn | grep ESTABLISHED | grep -E "3000[1-7]"
    
    echo -e "\n${CYAN}Process details:${NC}"
    for port in $PORT_LIST; do
      pid=$(lsof -i :$port -t 2>/dev/null)
      if [ -n "$pid" ]; then
        echo -e "Port $port: ${YELLOW}PID $pid${NC}"
        ps -p $pid -o pid,ppid,cmd 2>/dev/null
      fi
    done
  fi
}

# Main execution
if [ "$MONITOR" = true ] && is_terminal; then
  # Continuous monitoring
  while true; do
    run_checks
    echo -e "\n${YELLOW}Monitoring... Press Ctrl+C to exit${NC}"
    sleep 2
  done
else
  # Single run
  run_checks
fi
