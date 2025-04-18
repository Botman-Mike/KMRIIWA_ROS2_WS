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

# Function to check firewall status
check_firewall_status() {
  echo -e "\n${CYAN}Firewall Status:${NC}"
  
  # Check if ufw is installed and active
  if ! command -v ufw >/dev/null 2>&1; then
    echo -e "${YELLOW}UFW (Uncomplicated Firewall) is not installed.${NC}"
    echo -e "Install with: sudo apt install ufw"
    return
  fi
  
  # Check if firewall is active
  if sudo ufw status | grep -q "Status: active"; then
    echo -e "${GREEN}Firewall is active.${NC} Checking port rules..."
    
    for port in $PORT_LIST; do
      protocol=""
      for entry in "${PORT_NAMES[@]}"; do
        if [[ $entry == "$port:"* ]]; then
          if [[ $entry == *"(TCP)"* ]]; then
            protocol="tcp"
          elif [[ $entry == *"(UDP)"* ]]; then
            protocol="udp"
          fi
          break
        fi
      done
      
      if [ -n "$protocol" ]; then
        if sudo ufw status | grep -q "$port/$protocol"; then
          echo -e "Port ${GREEN}$port/$protocol${NC} is ${GREEN}allowed${NC} in the firewall"
        else
          echo -e "Port ${RED}$port/$protocol${NC} is ${RED}NOT allowed${NC} in the firewall"
        fi
      fi
    done
  else
    echo -e "${YELLOW}Firewall is not active.${NC} All ports should be accessible."
  fi
}

# Function to configure the firewall for KMR ports
configure_firewall() {
  echo -e "\n${CYAN}Configuring Firewall for KMR Communication:${NC}"
  
  # Check if ufw is installed
  if ! command -v ufw >/dev/null 2>&1; then
    echo -e "${YELLOW}UFW (Uncomplicated Firewall) is not installed.${NC}"
    read -p "Do you want to install UFW? (y/n): " install_ufw
    if [[ $install_ufw == "y" || $install_ufw == "Y" ]]; then
      echo "Installing UFW..."
      sudo apt update
      sudo apt install -y ufw
    else
      echo "Aborting firewall configuration."
      return 1
    fi
  fi
  
  # Ask for confirmation before modifying firewall rules
  read -p "This will add rules to your firewall to allow KMR communication ports. Continue? (y/n): " confirm
  if [[ $confirm != "y" && $confirm != "Y" ]]; then
    echo "Aborting firewall configuration."
    return 1
  fi
  
  # Add rules for each port
  for port in $PORT_LIST; do
    protocol=""
    for entry in "${PORT_NAMES[@]}"; do
      if [[ $entry == "$port:"* ]]; then
        if [[ $entry == *"(TCP)"* ]]; then
          protocol="tcp"
        elif [[ $entry == *"(UDP)"* ]]; then
          protocol="udp"
        fi
        break
      fi
    done
    
    if [ -n "$protocol" ]; then
      echo "Adding rule for port $port/$protocol..."
      sudo ufw allow $port/$protocol
    fi
  done
  
  # Enable firewall if not already enabled
  if ! sudo ufw status | grep -q "Status: active"; then
    echo "Enabling firewall..."
    echo "y" | sudo ufw enable
  fi
  
  echo -e "${GREEN}Firewall rules added successfully.${NC}"
  sudo ufw status
}

# Function to check robot connectivity
check_robot_connection() {
  robot_ip=$1
  # Increase ping timeout to 10 minutes
  if ping -c 1 -W 600 $robot_ip >/dev/null 2>&1; then
    echo -e "${GREEN}Robot at $robot_ip is reachable${NC}"
  else
    echo -e "${RED}Robot at $robot_ip is NOT reachable${NC}"
  fi
}

# Function to perform detailed port scan using nmap if available
scan_ports() {
  target_ip=$1
  
  echo -e "\n${CYAN}Port Scan Results for $target_ip:${NC}"
  
  if ! command -v nmap >/dev/null 2>&1; then
    echo -e "${YELLOW}nmap is not installed. Install with:${NC} sudo apt install nmap"
    return
  fi
  
  echo "Scanning ports 30001-30007..."
  nmap_output=$(nmap -p 30001-30007 $target_ip)
  echo -e "$nmap_output" | grep -E "PORT|30001|30002|30003|30004|30005|30006|30007"
  
  # Extract and interpret results
  echo -e "\n${CYAN}Connectivity Analysis:${NC}"
  
  # Check for filtered ports which indicate firewall issues
  if echo "$nmap_output" | grep -q "filtered"; then
    echo -e "${YELLOW}WARNING:${NC} Some ports are filtered, indicating a potential firewall issue."
    echo "          This may prevent proper communication with the robot."
    filtered_ports=$(echo "$nmap_output" | grep "filtered" | awk '{print $1}')
    echo "          Filtered ports: $filtered_ports"
  fi
  
  # If all ports on robot are closed
  if echo "$nmap_output" | grep -v "PORT" | grep -q "closed" && ! echo "$nmap_output" | grep -v "PORT" | grep -q "open"; then
    echo -e "${RED}ERROR:${NC} All ports on $target_ip appear to be closed."
    echo "       This suggests the robot services are not running."
    echo "       Please check if the robot software has been started."
  fi
  
  # UDP scan is also important for our protocol
  echo -e "\n${CYAN}UDP Port Scan (may take longer):${NC}"
  echo "Scanning UDP ports 30001-30007..."
  sudo nmap -sU -p 30001-30007 --version-intensity 0 $target_ip | grep -E "PORT|30001|30002|30003|30004|30005|30006|30007"
}

# Function to check network interfaces and IP configuration
check_network_config() {
  echo -e "\n${CYAN}Network Configuration:${NC}"
  
  # Get network interfaces and their IP addresses
  echo -e "${CYAN}Network Interfaces:${NC}"
  ip -br addr show | grep -v "lo" | while read line; do
    interface=$(echo $line | awk '{print $1}')
    status=$(echo $line | awk '{print $2}')
    ip_addr=$(echo $line | awk '{print $3}')
    
    if [[ "$status" == "UP" ]]; then
      echo -e "${GREEN}$interface:${NC} $ip_addr - ${GREEN}UP${NC}"
    else
      echo -e "${RED}$interface:${NC} $ip_addr - ${RED}DOWN${NC}"
    fi
  done
  
  # Check if we're on the same subnet as the robot
  robot_subnet=$(echo $ROBOT_IP | cut -d. -f1-3)
  host_has_matching_subnet=false
  
  ip -br addr show | grep -v "lo" | grep -o '[0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}\.[0-9]\{1,3\}/[0-9]\{1,2\}' | while read ip_cidr; do
    ip_addr=$(echo $ip_cidr | cut -d/ -f1)
    host_subnet=$(echo $ip_addr | cut -d. -f1-3)
    
    if [[ "$host_subnet" == "$robot_subnet" ]]; then
      host_has_matching_subnet=true
      echo -e "${GREEN}Correct subnet:${NC} This machine has an IP on the same subnet as the robot ($robot_subnet.x)"
      break
    fi
  done
  
  if ! $host_has_matching_subnet; then
    echo -e "${RED}Subnet mismatch:${NC} This machine does not have an IP address on the robot subnet ($robot_subnet.x)"
    echo "This could prevent communication with the robot."
    echo "Consider adding an IP address on the $robot_subnet.x subnet."
  fi
  
  # Check routing
  echo -e "\n${CYAN}Route to Robot:${NC}"
  ip route get $ROBOT_IP
}

# Function to test both TCP and UDP connections to all relevant ports
test_connections() {
  target_ip=$1
  
  echo -e "\n${CYAN}Connection Tests to $target_ip:${NC}"
  
  # Test TCP connections with increased timeout
  echo -e "\n${CYAN}TCP Connection Tests:${NC}"
  for port in 30001 30002 30005 30006; do
    desc=""
    for entry in "${PORT_NAMES[@]}"; do
      if [[ $entry == "$port:"* ]]; then
        desc=${entry#*:}
        break
      fi
    done
    
    # Use timeout with increased value (10 minutes = 600 seconds)
    if timeout 600 bash -c "</dev/tcp/$target_ip/$port" 2>/dev/null; then
      echo -e "${GREEN}✓${NC} Port $port ($desc) is ${GREEN}reachable${NC}"
    else
      echo -e "${RED}✗${NC} Port $port ($desc) is ${RED}not reachable${NC}"
    fi
  done
  
  # Test UDP connections with increased timeout
  echo -e "\n${CYAN}UDP Connection Test Attempts:${NC}"
  for port in 30003 30004 30007; do
    desc=""
    for entry in "${PORT_NAMES[@]}"; do
      if [[ $entry == "$port:"* ]]; then
        desc=${entry#*:}
        break
      fi
    done
    
    # For UDP, we can use netcat to try to send a packet and see if we get a response or error
    # Increase timeout to 10 minutes = 600 seconds
    echo "Testing UDP port $port ($desc)..."
    timeout 600 nc -u -z -v $target_ip $port 2>&1 | grep -q "succeeded" && \
      echo -e "${GREEN}✓${NC} Port $port ($desc) appears ${GREEN}open${NC}" || \
      echo -e "${YELLOW}?${NC} Port $port ($desc) status ${YELLOW}uncertain${NC}"
  done
}

# Function to suggest fixes based on detected issues
suggest_fixes() {
  echo -e "\n${CYAN}Troubleshooting Suggestions:${NC}"
  
  # Check if we have nmap results
  if [ -n "$nmap_output" ]; then
    # If we see filtered ports, suggest firewall fix
    if echo "$nmap_output" | grep -q "filtered"; then
      echo -e "1. ${YELLOW}Firewall issues detected:${NC}"
      echo "   - Configure firewall on both machines to allow ports 30001-30007 (TCP/UDP)"
      echo "   - Run: $0 -F to configure firewall on this machine"
      echo "   - On the robot, check KUKA firewall settings"
    fi
    
    # If all ports are closed on robot, suggest starting services
    if echo "$nmap_output" | grep -v "PORT" | grep -q "closed" && ! echo "$nmap_output" | grep -v "PORT" | grep -q "open"; then
      echo -e "2. ${YELLOW}Robot services may not be running:${NC}"
      echo "   - Ensure the robot control software is started"
      echo "   - Check if the KUKA Java application is running"
      echo "   - Restart the robot services if necessary"
    fi
  fi
  
  # Check local port binding
  local_binding_issues=false
  for port in $PORT_LIST; do
    if ! netstat -tuln | grep -q ":$port "; then
      local_binding_issues=true
      break
    fi
  done
  
  if $local_binding_issues; then
    echo -e "3. ${YELLOW}Local port binding issues:${NC}"
    echo "   - Ensure ROS2 nodes are running and binding to the correct ports"
    echo "   - Check if another process is using the required ports"
    echo "   - Try restarting your ROS2 application"
  fi
  
  # General network suggestions
  echo -e "4. ${YELLOW}General network troubleshooting:${NC}"
  echo "   - Verify both machines are on the same subnet"
  echo "   - Check physical network connections"
  echo "   - Restart networking on both systems"
  echo "   - Try disabling firewalls temporarily to test connectivity"
  
  # Robot-specific suggestions
  echo -e "5. ${YELLOW}Robot-specific checks:${NC}"
  echo "   - Restart the Sunrise Workbench application"
  echo "   - Check robot logs for any connection errors"
  echo "   - Verify the robot's IP configuration matches what you're using"
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
  echo "  -f, --firewall      Check firewall status for KMR ports"
  echo "  -F, --configure-fw  Configure firewall to allow KMR ports"
  echo "  -s, --scan          Perform port scan of the robot"
  echo "  -n, --network       Check network configuration"
  echo "  -t, --test          Test direct connections to ports"
  echo "  -d, --diagnose      Run comprehensive diagnostics and suggest fixes"
  echo ""
  echo "Example: $0 -m -i 172.31.1.206 -c -f"
  echo "Full diagnostic: $0 -d -i 172.31.1.206"
}

# Parse command line arguments
MONITOR=false
ROBOT_IP="172.31.1.206"
CHECK_ROBOT=false
VERBOSE=false
SPECIFIC_PORT=""
CHECK_FIREWALL=false
CONFIGURE_FIREWALL=false
SCAN_PORTS=false
CHECK_NETWORK=false
TEST_CONNECTIONS=false
DIAGNOSE=false

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
    -f|--firewall)
      CHECK_FIREWALL=true
      shift
      ;;
    -F|--configure-fw)
      CONFIGURE_FIREWALL=true
      shift
      ;;
    -s|--scan)
      SCAN_PORTS=true
      shift
      ;;
    -n|--network)
      CHECK_NETWORK=true
      shift
      ;;
    -t|--test)
      TEST_CONNECTIONS=true
      shift
      ;;
    -d|--diagnose)
      DIAGNOSE=true
      CHECK_ROBOT=true
      SCAN_PORTS=true
      CHECK_NETWORK=true
      TEST_CONNECTIONS=true
      CHECK_FIREWALL=true
      shift
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
  
  if [ "$CHECK_FIREWALL" = true ]; then
    check_firewall_status
  fi
  
  if [ "$CHECK_NETWORK" = true ]; then
    check_network_config
  fi
  
  if [ "$SCAN_PORTS" = true ]; then
    scan_ports $ROBOT_IP
  fi
  
  if [ "$TEST_CONNECTIONS" = true ]; then
    test_connections $ROBOT_IP
  fi
  
  if [ "$DIAGNOSE" = true ]; then
    suggest_fixes
  fi
}

# Main execution
if [ "$CONFIGURE_FIREWALL" = true ]; then
  configure_firewall
  exit $?
fi

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
