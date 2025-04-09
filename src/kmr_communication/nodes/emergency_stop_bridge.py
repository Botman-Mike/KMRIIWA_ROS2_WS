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
from std_msgs.msg import Bool
import time

class EmergencyStopBridge(Node):
    """
    Bridge node that monitors external emergency stop triggers 
    and publishes to the internal emergency stop topic.
    
    This can be extended to interface with hardware e-stop buttons,
    process watchdogs, or other external safety systems.
    """
    def __init__(self):
        super().__init__('emergency_stop_bridge')
        
        # Parameters
        self.declare_parameter('check_interval', 0.1)  # How often to check safety status (seconds)
        self.check_interval = self.get_parameter('check_interval').value
        
        # Create publisher for e-stop status
        self.pub_estop = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # Subscribe to potential external e-stop sources
        # This is just an example - you would adapt these to your specific system
        self.sub_hardware_estop = self.create_subscription(
            Bool, 'hardware_estop_button', self.hardware_estop_callback, 10)
        self.sub_watchdog = self.create_subscription(
            Bool, 'system_watchdog_fault', self.watchdog_callback, 10)
        self.sub_manual_estop = self.create_subscription(
            Bool, 'manual_estop_trigger', self.manual_estop_callback, 10)
        
        # Track e-stop sources
        self.estop_sources = {
            'hardware': False,
            'watchdog': False,
            'manual': False,
            'command_line': False
        }
        
        # Command-line interface for triggering e-stop
        self.timer = self.create_timer(self.check_interval, self.check_status)
        
        self.get_logger().info('E-Stop Bridge started - system is running')
        
    def hardware_estop_callback(self, msg):
        """Process hardware emergency stop trigger"""
        self.estop_sources['hardware'] = msg.data
        if msg.data:
            self.get_logger().error("Hardware Emergency Stop button pressed!")
        
    def watchdog_callback(self, msg):
        """Process system watchdog fault"""
        self.estop_sources['watchdog'] = msg.data
        if msg.data:
            self.get_logger().error("System Watchdog fault detected!")
            
    def manual_estop_callback(self, msg):
        """Process manual emergency stop trigger from UI or other source"""
        self.estop_sources['manual'] = msg.data
        if msg.data:
            self.get_logger().error("Manual Emergency Stop triggered!")
    
    def trigger_estop(self):
        """Trigger an emergency stop from command line"""
        self.estop_sources['command_line'] = True
        self.get_logger().error("Command-line Emergency Stop triggered!")
        
    def reset_estop(self):
        """Reset all emergency stops"""
        for key in self.estop_sources:
            self.estop_sources[key] = False
        self.get_logger().info("Emergency Stop reset - verify safe conditions before resuming operation")
    
    def check_status(self):
        """Check overall e-stop status and publish"""
        # Any active e-stop source triggers the overall e-stop
        is_estop_active = any(self.estop_sources.values())
        
        # Publish the status
        msg = Bool()
        msg.data = is_estop_active
        self.pub_estop.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        estop_bridge = EmergencyStopBridge()
        
        # Simple command-line interface for testing
        import threading
        def cli_thread():
            while rclpy.ok():
                try:
                    cmd = input("\nE-Stop Commands (e: trigger e-stop, r: reset, q: quit): ")
                    if cmd.lower() == 'e':
                        estop_bridge.trigger_estop()
                    elif cmd.lower() == 'r':
                        estop_bridge.reset_estop()
                    elif cmd.lower() == 'q':
                        estop_bridge.get_logger().info("Shutting down...")
                        rclpy.shutdown()
                        break
                except Exception as e:
                    print(f"Command error: {e}")
                    
        # Start CLI in a separate thread
        cli_thread = threading.Thread(target=cli_thread, daemon=True)
        cli_thread.start()
        
        rclpy.spin(estop_bridge)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'estop_bridge' in locals():
            estop_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
