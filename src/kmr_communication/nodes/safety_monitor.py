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
import time

class SafetyMonitor(Node):
    """
    Node to monitor robot safety conditions and issue emergency stops when necessary.
    This acts as the central authority for determining if commands should be sent to the robot.
    """
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Parameters
        self.declare_parameter('check_interval', 0.1)  # How often to check safety status (seconds)
        self.declare_parameter('auto_reset', False)    # Whether to auto-reset safety stop
        self.declare_parameter('auto_reset_delay', 5.0)  # Delay before auto-reset (seconds)
        
        self.check_interval = self.get_parameter('check_interval').value
        self.auto_reset = self.get_parameter('auto_reset').value
        self.auto_reset_delay = self.get_parameter('auto_reset_delay').value
        
        # Track safety status
        self.safety_status = {
            'kmp_safetystop': False,
            'lbr_safetystop': False,
            'kmp_ready': True,
            'lbr_ready': True,
            'protection_field_clear': True,
            'warning_field_clear': True,
            'manual_estop': False,
            'last_safetystop_time': 0.0
        }
        
        # Create subscribers for robot status
        self.sub_kmp_status = self.create_subscription(
            KmpStatusdata, 'kmp_statusdata', self.kmp_status_callback, 10)
        self.sub_lbr_status = self.create_subscription(
            LbrStatusdata, 'lbr_statusdata', self.lbr_status_callback, 10)
        self.sub_estop = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_stop_callback, 10)
        
        # Create publishers for safety status
        self.pub_kmp_safe = self.create_publisher(Bool, 'kmp_safe_to_move', 10)
        self.pub_lbr_safe = self.create_publisher(Bool, 'lbr_safe_to_move', 10)
        self.pub_estop = self.create_publisher(Bool, 'emergency_stop_active', 10)
        self.pub_safety_status = self.create_publisher(String, 'robot_safety_status', 10)
        self.pub_reset = self.create_publisher(Bool, 'reset_safety_stop', 10)
        
        # Timer to periodically check and publish safety status
        self.timer = self.create_timer(self.check_interval, self.check_safety)
        
        self.get_logger().info('Safety Monitor started')
    
    def kmp_status_callback(self, msg):
        """Process KMP status data"""
        self.safety_status['kmp_safetystop'] = msg.kmp_safetystop
        self.safety_status['kmp_ready'] = msg.ready_to_move
        self.safety_status['protection_field_clear'] = msg.protection_field_clear
        self.safety_status['warning_field_clear'] = msg.warning_field_clear
        
        # Record time of safety stop for potential auto-reset
        if msg.kmp_safetystop and not self.safety_status.get('prev_kmp_safetystop', False):
            self.safety_status['last_safetystop_time'] = time.time()
            self.get_logger().warn("KMP Safety Stop activated!")
            
        self.safety_status['prev_kmp_safetystop'] = msg.kmp_safetystop
    
    def lbr_status_callback(self, msg):
        """Process LBR status data"""
        self.safety_status['lbr_safetystop'] = msg.lbr_safetystop
        self.safety_status['lbr_ready'] = msg.ready_to_move
        
        # Record time of safety stop for potential auto-reset
        if msg.lbr_safetystop and not self.safety_status.get('prev_lbr_safetystop', False):
            self.safety_status['last_safetystop_time'] = time.time()
            self.get_logger().warn("LBR Safety Stop activated!")
            
        self.safety_status['prev_lbr_safetystop'] = msg.lbr_safetystop
    
    def emergency_stop_callback(self, msg):
        """Process external emergency stop command"""
        if msg.data and not self.safety_status['manual_estop']:
            self.get_logger().error("External Emergency Stop activated!")
            
        self.safety_status['manual_estop'] = msg.data
        if msg.data:
            self.safety_status['last_safetystop_time'] = time.time()
    
    def reset_safety_stop(self):
        """Publish message to reset safety stops if applicable"""
        reset_msg = Bool()
        reset_msg.data = True
        self.pub_reset.publish(reset_msg)
        self.get_logger().info("Attempting to reset safety stops")
    
    def check_safety(self):
        """Check safety conditions and publish status"""
        now = time.time()
        
        # Determine if KMP is safe to move
        kmp_safe = (
            not self.safety_status['kmp_safetystop'] and 
            self.safety_status['kmp_ready'] and 
            self.safety_status['protection_field_clear'] and 
            not self.safety_status['manual_estop']
        )
        
        # Determine if LBR is safe to move
        lbr_safe = (
            not self.safety_status['lbr_safetystop'] and 
            self.safety_status['lbr_ready'] and 
            not self.safety_status['manual_estop']
        )
        
        # Consider auto-reset for safety stops if configured
        if self.auto_reset and (self.safety_status['kmp_safetystop'] or self.safety_status['lbr_safetystop']):
            if now - self.safety_status['last_safetystop_time'] > self.auto_reset_delay:
                self.reset_safety_stop()
                # Wait for the reset to take effect, don't immediately consider it safe
        
        # Publish safety statuses
        kmp_msg = Bool()
        kmp_msg.data = kmp_safe
        self.pub_kmp_safe.publish(kmp_msg)
        
        lbr_msg = Bool()
        lbr_msg.data = lbr_safe
        self.pub_lbr_safe.publish(lbr_msg)
        
        estop_msg = Bool()
        estop_msg.data = self.safety_status['manual_estop']
        self.pub_estop.publish(estop_msg)
        
        # Publish detailed safety status
        status_msg = String()
        status_lines = [
            "--- Robot Safety Status ---",
            f"KMP Safety Stop: {'ACTIVE' if self.safety_status['kmp_safetystop'] else 'Inactive'}",
            f"LBR Safety Stop: {'ACTIVE' if self.safety_status['lbr_safetystop'] else 'Inactive'}",
            f"KMP Ready: {'Yes' if self.safety_status['kmp_ready'] else 'No'}",
            f"LBR Ready: {'Yes' if self.safety_status['lbr_ready'] else 'No'}",
            f"Protection Field: {'Clear' if self.safety_status['protection_field_clear'] else 'OCCUPIED'}",
            f"Warning Field: {'Clear' if self.safety_status['warning_field_clear'] else 'OCCUPIED'}",
            f"Emergency Stop: {'ACTIVE' if self.safety_status['manual_estop'] else 'Inactive'}",
            f"KMP Safe to Move: {'Yes' if kmp_safe else 'No'}",
            f"LBR Safe to Move: {'Yes' if lbr_safe else 'No'}"
        ]
        status_msg.data = "\n".join(status_lines)
        self.pub_safety_status.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        safety_monitor = SafetyMonitor()
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'safety_monitor' in locals():
            safety_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
