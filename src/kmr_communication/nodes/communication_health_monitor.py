#!/usr/bin/env python3

# Copyright 2023
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from kmr_msgs.msg import KmpStatusdata, LbrStatusdata
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
import time
from datetime import datetime


def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'
def cl_yellow(msge): return '\033[93m' + msge + '\033[0m'
def cl_cyan(msge): return '\033[36m' + msge + '\033[0m'


class CommunicationHealthMonitor(Node):
    """
    Node to monitor the health of all communication channels with the robot.
    """
    def __init__(self):
        super().__init__('communication_health_monitor')
        
        # Parameters
        self.declare_parameter('check_interval', 1.0)  # How often to check communication health (seconds)
        self.declare_parameter('timeout_threshold', 600.0)  # INCREASED TO 10 MINUTES FOR TROUBLESHOOTING
        
        self.check_interval = self.get_parameter('check_interval').value
        self.timeout_threshold = self.get_parameter('timeout_threshold').value
        
        # Track the last received message time for each topic
        self.last_received = {
            'kmp_statusdata': 0.0,
            'lbr_statusdata': 0.0, 
            'odom': 0.0,
            'scan': 0.0,
            'scan_2': 0.0,
            'joint_states': 0.0,
            'robot_connected': False,
            'manipulator_connected': False
        }
        
        # Create subscribers for all relevant topics
        self.sub_kmp_status = self.create_subscription(
            KmpStatusdata, 'kmp_statusdata', self.kmp_status_callback, 10)
        self.sub_lbr_status = self.create_subscription(
            LbrStatusdata, 'lbr_statusdata', self.lbr_status_callback, 10)
        self.sub_odom = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)
        self.sub_scan = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.sub_scan2 = self.create_subscription(
            LaserScan, 'scan_2', self.scan2_callback, qos_profile_sensor_data)
        self.sub_joint_states = self.create_subscription(
            JointState, 'joint_states', self.joint_states_callback, 20)
        self.sub_robot_connected = self.create_subscription(
            Bool, 'robot_connected', self.robot_connected_callback, 10)
        self.sub_manipulator_connected = self.create_subscription(
            Bool, 'manipulator_connected', self.manipulator_connected_callback, 10)
        
        # Create publisher for health status
        self.pub_health_status = self.create_publisher(String, 'communication_health', 10)
        
        # Timer to periodically check communication health
        self.timer = self.create_timer(self.check_interval, self.check_health)
        
        self.get_logger().info('Communication Health Monitor started')
    
    def kmp_status_callback(self, msg):
        self.last_received['kmp_statusdata'] = time.time()
    
    def lbr_status_callback(self, msg):
        self.last_received['lbr_statusdata'] = time.time()
    
    def odom_callback(self, msg):
        self.last_received['odom'] = time.time()
    
    def scan_callback(self, msg):
        self.last_received['scan'] = time.time()
    
    def scan2_callback(self, msg):
        self.last_received['scan_2'] = time.time()
    
    def joint_states_callback(self, msg):
        self.last_received['joint_states'] = time.time()
    
    def robot_connected_callback(self, msg):
        self.last_received['robot_connected'] = msg.data
    
    def manipulator_connected_callback(self, msg):
        self.last_received['manipulator_connected'] = msg.data
    
    def check_health(self):
        """Check the health of all communication channels"""
        now = time.time()
        status_msg = String()
        status_lines = []
        
        # Check connection status
        robot_connected = self.last_received['robot_connected']
        manipulator_connected = self.last_received['manipulator_connected']
        
        # Add connection status
        status_lines.append(f"Robot connection: {'CONNECTED' if robot_connected else 'DISCONNECTED'}")
        status_lines.append(f"Manipulator connection: {'CONNECTED' if manipulator_connected else 'DISCONNECTED'}")
        
        # Check data streams if connections are up
        all_healthy = True
        
        if robot_connected:
            # Check KMP related topics
            kmp_status_age = now - self.last_received['kmp_statusdata']
            odom_age = now - self.last_received['odom']
            scan_age = now - self.last_received['scan']
            scan2_age = now - self.last_received['scan_2']
            
            kmp_status_ok = kmp_status_age < self.timeout_threshold
            odom_ok = odom_age < self.timeout_threshold
            scan_ok = scan_age < self.timeout_threshold
            scan2_ok = scan2_age < self.timeout_threshold
            
            status_lines.append(f"KMP Status: {'OK' if kmp_status_ok else 'TIMEOUT'} ({kmp_status_age:.1f}s)")
            status_lines.append(f"Odometry: {'OK' if odom_ok else 'TIMEOUT'} ({odom_age:.1f}s)")
            status_lines.append(f"Laser scan 1: {'OK' if scan_ok else 'TIMEOUT'} ({scan_age:.1f}s)")
            status_lines.append(f"Laser scan 2: {'OK' if scan2_ok else 'TIMEOUT'} ({scan2_age:.1f}s)")
            
            if not (kmp_status_ok and odom_ok and (scan_ok or scan2_ok)):
                all_healthy = False
        
        if manipulator_connected:
            # Check LBR related topics
            lbr_status_age = now - self.last_received['lbr_statusdata']
            joint_states_age = now - self.last_received['joint_states']
            
            lbr_status_ok = lbr_status_age < self.timeout_threshold
            joint_states_ok = joint_states_age < self.timeout_threshold
            
            status_lines.append(f"LBR Status: {'OK' if lbr_status_ok else 'TIMEOUT'} ({lbr_status_age:.1f}s)")
            status_lines.append(f"Joint states: {'OK' if joint_states_ok else 'TIMEOUT'} ({joint_states_age:.1f}s)")
            
            if not (lbr_status_ok and joint_states_ok):
                all_healthy = False
        
        # Add overall health status
        if not robot_connected and not manipulator_connected:
            status_lines.insert(0, "CRITICAL: All connections down")
            self.get_logger().error("All connections down")
        elif not all_healthy:
            status_lines.insert(0, "WARNING: Some data streams not available")
            self.get_logger().warn("Some data streams not available")
        else:
            status_lines.insert(0, "HEALTHY: All systems operational")
            self.get_logger().debug("All systems operational")
        
        # Add timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        status_lines.append(f"Last checked: {timestamp}")
        
        # Format and publish status message
        status_msg.data = "\n".join(status_lines)
        self.pub_health_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        health_monitor = CommunicationHealthMonitor()
        rclpy.spin(health_monitor)
    except KeyboardInterrupt:
        print(cl_yellow('Keyboard interrupt received, shutting down...'))
    except Exception as e:
        print(cl_red(f'Error: {e}'))
    finally:
        if 'health_monitor' in locals():
            health_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
