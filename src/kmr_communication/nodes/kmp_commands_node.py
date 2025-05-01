#!/usr/bin/env python3

# Copyright 2019 Nina Marie Wahl and Charlotte Heggem.
# Copyright 2019 Norwegian University of Science and Technology.
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

import sys
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import argparse

from tcpSocket import TCPSocket
from udpSocket import UDPSocket


def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class KmpCommandsNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('kmp_commands_node')
        self.name = 'kmp_commands_node'
        self.declare_parameter('port', 30002)
        port = int(self.get_parameter('port').value)
        if robot == 'KMR1':
            self.declare_parameter('KMR1/ip', '172.31.1.206')
            ip = str(self.get_parameter('KMR1/ip').value)
        elif robot == 'KMR2':
            self.declare_parameter('KMR2/ip', '172.31.1.206')
            ip = str(self.get_parameter('KMR2/ip').value)
        else:
            ip=None

        if connection_type == 'TCP':
            self.soc = TCPSocket(ip,port,self.name)
            # Register socket close on shutdown, with fallback
            try:
                self.add_on_shutdown_callback(self.soc.close)
            except AttributeError:
                rclpy.get_default_context().on_shutdown(self.soc.close)
        elif connection_type == 'UDP':
            self.soc = UDPSocket(ip,port,self.name)
            # Register socket close on shutdown, with fallback
            try:
                self.add_on_shutdown_callback(self.soc.close)
            except AttributeError:
                rclpy.get_default_context().on_shutdown(self.soc.close)
        else:
            self.soc = None
            rclpy.shutdown()
            return

        # Create publishers for connection status
        self.connection_status_pub = self.create_publisher(Bool, 'robot_connected', 10)
        # Connection status publisher (for diagnostics, like LBR node)
        self.connection_status_pub = self.create_publisher(Bool, 'kmp_connected', 10)
        self.create_timer(1.0, self.publish_connection_status)
        
        # Track robot safety status
        self.safe_to_move = True
        self.emergency_stop_active = False
        
        # Subscribe to safety monitor topics
        self.sub_safety = self.create_subscription(
            Bool, 'kmp_safe_to_move', self.safety_callback, 10)
        self.sub_estop = self.create_subscription(
            Bool, 'emergency_stop_active', self.estop_callback, 10)
        self.sub_reset = self.create_subscription(
            Bool, 'reset_safety_stop', self.reset_callback, 10)
        
        # Setup command queue and lock
        self.command_queue_lock = threading.Lock()
        self.command_queue = []  # Queue for commands to ensure they're sent in order
        
        # Create a worker thread to process command queue
        self.command_worker_thread = threading.Thread(target=self.process_command_queue, daemon=True)
        self.command_worker_thread.start()
        
        # Setup subscribers for commands
        self.sub_twist = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.sub_pose = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.sub_shutdown = self.create_subscription(String, 'shutdown', self.shutdown_callback, 10)
        
        # Setup a timer for connection monitoring
        self.timer = self.create_timer(1.0, self.connection_check_callback)
        
        # Wait for initial connection
        connection_timeout = 600.0  # INCREASED TO 10 MINUTES FOR TROUBLESHOOTING
        
        start_time = time.time()
        self.get_logger().info('Waiting for initial connection...')
        while not self.soc.isconnected and time.time() - start_time < connection_timeout:
            time.sleep(0.1)
            
        if self.soc.isconnected:
            self.get_logger().info('Connected to robot successfully')
        else:
            self.get_logger().warn('Initial connection timed out, will keep trying in background')

    def safety_callback(self, msg):
        """Handle safe-to-move status updates"""
        self.safe_to_move = msg.data
        self.get_logger().debug(f'Safe to move status updated: {self.safe_to_move}')

    def estop_callback(self, msg):
        """Handle emergency stop status updates"""
        self.emergency_stop_active = msg.data
        self.get_logger().debug(f'Emergency stop status updated: {self.emergency_stop_active}')
        
    def reset_callback(self, msg):
        """Handle safety reset requests"""
        if msg.data:
            self.safe_to_move = True
            self.get_logger().info('Safety system reset requested')
    
    def process_command_queue(self):
        """Process commands in the queue and send to the robot"""
        while rclpy.ok():
            try:
                # Process commands only when connected and safe
                if self.soc.isconnected and self.safe_to_move and not self.emergency_stop_active:
                    # Get next command from queue if available
                    cmd = None
                    with self.command_queue_lock:
                        if self.command_queue:
                            cmd = self.command_queue.pop(0)
                    
                    # Send command if we got one
                    if cmd:
                        self.get_logger().debug(f'Sending command: {cmd}')
                        try:
                            self.soc.send(cmd)
                        except Exception as e:
                            self.get_logger().error(f'Failed to send command: {e}')
                            # Put command back in queue for retry
                            with self.command_queue_lock:
                                self.command_queue.insert(0, cmd)
                
                # Don't hog CPU
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().error(f'Command processing error: {e}')
                time.sleep(0.1)  # Prevent rapid error loops

    def connection_check_callback(self):
        """Periodically check connection status and publish it"""
        status_msg = Bool()
        status_msg.data = self.soc.isconnected
        self.connection_status_pub.publish(status_msg)
        
        # Log connection status periodically
        if self.soc.isconnected:
            self.get_logger().debug('Robot connection is active')
        else:
            self.get_logger().warn('Robot connection is down')

    def publish_connection_status(self):
        msg = Bool()
        msg.data = self.soc.isconnected if self.soc else False
        self.connection_status_pub.publish(msg)

    def shutdown_callback(self, data):
        """Handle shutdown requests"""
        self.get_logger().info(f'Shutdown command received: {data.data}')
        msg = "shutdown"
        # Add to command queue
        with self.command_queue_lock:
            self.command_queue.append(msg)

    def twist_callback(self, data):
        """Handle twist commands"""
        self.get_logger().debug(f'Received twist command: {data}')
        msg = 'setTwist ' + str(data.linear.x) + " " + str(data.linear.y) + " " + str(data.angular.z)
        # Add to command queue instead of sending directly
        with self.command_queue_lock:
            self.command_queue.append(msg)

    def pose_callback(self, data):
        """Handle pose commands"""
        self.get_logger().debug(f'Received pose command: {data}')
        msg = 'setPose ' + str(data.position.x) + " " + str(data.position.y) + " " + str(data.orientation.z)
        # Add to command queue instead of sending directly
        with self.command_queue_lock:
            self.command_queue.append(msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    kmp_commands_node = KmpCommandsNode(args.connection,args.robot)

    rclpy.spin(kmp_commands_node)
    # Ensure socket closed before shutdown
    try:
        kmp_commands_node.soc.close()
    except Exception:
        pass

    try:
        kmp_commands_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
