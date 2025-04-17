#!/usr/bin/env python3

import socket
import subprocess
import argparse
import time
import os
import sys

# Define colors for terminal output
def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'
def cl_yellow(msge): return '\033[93m' + msge + '\033[0m'
def cl_cyan(msge): return '\033[36m' + msge + '\033[0m'

def check_port_listening(ip, port, udp=False):
    """Check if a port is open and listening"""
    sock = None
    try:
        if udp:
            # For UDP sockets, we can only check binding capability
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1)
            try:
                sock.bind((ip, port))
                # If we can bind, it means no one is listening
                return False
            except socket.error:
                # If we can't bind, someone might be listening
                return True
        else:
            # For TCP, we can check connection
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex((ip, port))
            return result == 0
    except Exception as e:
        print(f"Error checking {'UDP' if udp else 'TCP'} port {port}: {e}")
        return False
    finally:
        if sock:
            sock.close()

def check_ros2_topics(port_to_topic_map):
    """Check if ROS2 topics are active"""
    try:
        # Run ros2 topic list
        result = subprocess.run(['ros2', 'topic', 'list'], 
                               stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE, 
                               text=True)
        
        if result.returncode != 0:
            print(cl_red("Error running ros2 topic list"))
            return {}
            
        topics = result.stdout.strip().split('\n')
        
        # Check which topics from our map are active
        topic_status = {}
        for port, topic in port_to_topic_map.items():
            topic_status[port] = any(t == topic or t.startswith(topic + '/') for t in topics)
            
        return topic_status
    except Exception as e:
        print(cl_red(f"Error checking ROS2 topics: {e}"))
        return {}

def check_ros2_nodes():
    """Get list of running ROS2 nodes"""
    try:
        result = subprocess.run(['ros2', 'node', 'list'], 
                               stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE, 
                               text=True)
        
        if result.returncode != 0:
            print(cl_red("Error running ros2 node list"))
            return []
            
        return result.stdout.strip().split('\n')
    except Exception as e:
        print(cl_red(f"Error checking ROS2 nodes: {e}"))
        return []

def check_netstat(ports):
    """Check netstat for port usage"""
    try:
        # Run netstat command
        result = subprocess.run(['netstat', '-tuln'], 
                               stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE, 
                               text=True)
        
        if result.returncode != 0:
            print(cl_red("Error running netstat"))
            return {}
            
        lines = result.stdout.strip().split('\n')
        
        # Process each line
        port_status = {p: {'active': False, 'protocol': None, 'pid': None} for p in ports}
        for line in lines:
            parts = line.split()
            if len(parts) >= 4:  # Netstat should have at least 4 columns
                local_address = parts[3]
                protocol = parts[0]
                
                for port in ports:
                    if f":{port}" in local_address:
                        port_status[port]['active'] = True
                        port_status[port]['protocol'] = protocol
                        
                        # Try to get PID
                        try:
                            # Get PID using lsof
                            lsof_result = subprocess.run(
                                ['lsof', '-i', f':{port}'],
                                stdout=subprocess.PIPE, 
                                stderr=subprocess.PIPE, 
                                text=True)
                            
                            if lsof_result.returncode == 0:
                                lsof_lines = lsof_result.stdout.strip().split('\n')
                                if len(lsof_lines) > 1:  # Header + at least one process
                                    # Format is: COMMAND PID USER FD TYPE DEVICE SIZE/OFF NODE NAME
                                    parts = lsof_lines[1].split()
                                    if len(parts) > 1:
                                        port_status[port]['pid'] = parts[1]
                        except:
                            pass
                        
        return port_status
    except Exception as e:
        print(cl_red(f"Error checking netstat: {e}"))
        return {}

def main():
    parser = argparse.ArgumentParser(description='Check KUKA KMR port connections')
    parser.add_argument('--robot-ip', default='172.31.1.206', help='Robot IP address')
    parser.add_argument('--local-ip', default='0.0.0.0', help='Local listening IP address')
    args = parser.parse_args()
    
    # Connection information
    port_info = {
        30001: {'desc': 'KMP Status', 'protocol': 'TCP', 'topic': '/kmp_statusdata'},
        30002: {'desc': 'KMP Command', 'protocol': 'TCP', 'topic': '/kmp_commands'},
        30003: {'desc': 'KMP Laser', 'protocol': 'UDP', 'topic': '/scan'},
        30004: {'desc': 'KMP Odometry', 'protocol': 'UDP', 'topic': '/odom'},
        30005: {'desc': 'LBR Command', 'protocol': 'TCP', 'topic': '/lbr_commands'},
        30006: {'desc': 'LBR Status', 'protocol': 'TCP', 'topic': '/lbr_statusdata'},
        30007: {'desc': 'LBR Sensors', 'protocol': 'UDP', 'topic': '/joint_states'}
    }
    
    print(cl_cyan("=== KUKA KMR Connection Check ==="))
    print(f"Robot IP: {args.robot_ip}")
    print(f"Local IP: {args.local_ip}")
    print()
    
    # Create map of ports to topics for ROS checking
    port_to_topic_map = {port: info['topic'] for port, info in port_info.items()}
    
    # Check netstat
    print(cl_cyan("Checking local port bindings..."))
    port_status = check_netstat(port_info.keys())
    
    # Check robot connectivity
    print(cl_cyan(f"Checking connectivity to robot at {args.robot_ip}..."))
    robot_online = check_port_listening(args.robot_ip, 22)  # SSH port check
    print(f"Robot online: {cl_green('YES') if robot_online else cl_red('NO')}")
    
    # Check ROS2 nodes running
    print(cl_cyan("Checking ROS2 nodes..."))
    ros2_nodes = check_ros2_nodes()
    expected_nodes = ['kmp_commands_node', 'kmp_laserscan_node', 'kmp_odometry_node', 
                     'kmp_statusdata_node', 'lbr_commands_node', 'lbr_statusdata_node', 
                     'lbr_sensordata_node']
    
    # Check which expected nodes are running
    for node in expected_nodes:
        node_running = any(n.strip('/') == node for n in ros2_nodes)
        print(f"Node {node}: {cl_green('RUNNING') if node_running else cl_red('NOT RUNNING')}")
    
    # Check ROS2 topics
    print(cl_cyan("\nChecking ROS2 topics..."))
    topic_status = check_ros2_topics(port_to_topic_map)
    
    # Print full connection table
    print(cl_cyan("\n=== Port Connection Status ==="))
    print(f"{'Port':<8} {'Description':<15} {'Protocol':<10} {'Status':<15} {'Topic Status':<15} {'PID':<8}")
    print("-" * 80)
    
    for port, info in port_info.items():
        protocol = info['protocol']
        description = info['desc']
        topic = info['topic']
        
        # Port binding status
        if port in port_status and port_status[port]['active']:
            port_binding = cl_green("LISTENING")
            pid = port_status[port]['pid'] or 'N/A'
        else:
            port_binding = cl_red("NOT BOUND")
            pid = 'N/A'
        
        # Topic status
        if port in topic_status:
            topic_active = cl_green("ACTIVE") if topic_status[port] else cl_red("INACTIVE")
        else:
            topic_active = cl_yellow("UNKNOWN")
            
        print(f"{port:<8} {description:<15} {protocol:<10} {port_binding:<15} {topic_active:<15} {pid:<8}")
    
    print("\nFor more details on specific connections, try:")
    print("  ros2 topic echo /kmp_statusdata/connection_status")
    print("  ros2 topic echo /kmp_statusdata/connection_details")

if __name__ == '__main__':
    main()
