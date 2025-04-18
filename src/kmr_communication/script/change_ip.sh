#!/bin/bash

# No need to change IP - just verify current configuration
echo "Current network configuration:"
ip addr show eno1ip addr show eno1

# Ensure network connectivity with simulatornflict
echo "Testing connectivity to simulator:"sudo ip addr add 172.31.1.207/16 dev eno1
ping -c 3 172.31.1.205
 configuration:"
# Check if ports are open and listeningip addr show eno1
echo "Checking if ports are properly bound and listening:"
netstat -tuln | grep -E "30001|30002|30003|30004|30005|30006|30007"instead of 172.31.1.206"
echo "Your ROS2 nodes are still binding to 0.0.0.0 (all interfaces)"


echo "IMPORTANT: Ensure simulator is configured to connect to 172.31.1.206"