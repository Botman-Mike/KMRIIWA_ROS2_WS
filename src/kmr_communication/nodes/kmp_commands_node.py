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
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import argparse
import time
import threading

from tcpSocket import TCPSocket
from udpSocket import UDPSocket


def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'
def cl_yellow(msge): return '\033[93m' + msge + '\033[0m'


class KmpCommandsNode(Node):
    def __init__(self, connection_type, robot):
        super().__init__('kmp_commands_node')
        self.name = 'kmp_commands_node'
        self.declare_parameter('port', 30002)
        self.declare_parameter('reconnect_interval', 5.0)  # Seconds between reconnection attempts
        self.declare_parameter('respect_safety', True)  # Whether to respect safety signals
        
        port = int(self.get_parameter('port').value)
        reconnect_interval = self.get_parameter('reconnect_interval').value
        self.respect_safety = self.get_parameter('respect_safety').value
        
        if robot == 'KMR1':
            self.declare_parameter('KMR1/ip', '172.31.1.206')
            ip = str(self.get_parameter('KMR1/ip').value)
        elif robot == 'KMR2':
            self.declare_parameter('KMR2/ip', '172.31.1.206')
            ip = str(self.get_parameter('KMR2/ip').value)
        else:
            ip = None

        # Create socket based on connection type
        if connection_type == 'TCP':
            self.soc = TCPSocket(ip, port, self.name)
        elif connection_type == 'UDP':
            self.soc = UDPSocket(ip, port, self.name)
        else:
            self.get_logger().error(f"Invalid connection type: {connection_type}")
            self.soc = None
            rclpy.shutdown()
            return

        # Create publishers for connection status
        self.connection_status_pub = self.create_publisher(Bool, 'robot_connected', 10)
        
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
        connection_timeout = 30.0  # 30 seconds timeout
        start_time = time.time()
        
        self.get_logger().info('Waiting for initial connection...')
        while not self.soc.isconnected and time.time() - start_time < connection_timeout:
            time.sleep(0.1)
            
        if self.soc.isconnected:
            self.get_logger().info('Connected to robot successfully')
        else:
            self.get_logger().warn('Initial connection timed out, will keep trying in background')

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

    def shutdown_callback(self, data):
        self.get_logger().info(f"Shutdown requested: {data.data}")
        if self.soc.isconnected:
            msg = "shutdown"
            if self.soc.send(msg):
                self.get_logger().info("Shutdown command sent successfully")
            else:
                self.get_logger().error("Failed to send shutdown command")
        
        # Close the socket connection
        self.soc.close()

    def safety_callback(self, msg):
        """Process safety status updates"""
        was_safe = self.safe_to_move
        self.safe_to_move = msg.data
        
        if was_safe and not self.safe_to_move:
            self.get_logger().warn("KMP has entered unsafe state - commands will be blocked")
            # Clear command queue when entering unsafe state
            with self.command_queue_lock:
                self.command_queue.clear()
        elif not was_safe and self.safe_to_move:
            self.get_logger().info("KMP has returned to safe state - commands will be accepted")

    def estop_callback(self, msg):
        """Process emergency stop status"""
        was_stopped = self.emergency_stop_active
        self.emergency_stop_active = msg.data
        
        if not was_stopped and self.emergency_stop_active:
            self.get_logger().error("Emergency stop activated - all commands blocked")
            # Clear command queue when e-stop activated
            with self.command_queue_lock:
                self.command_queue.clear()
                # Send stop command to robot to halt any current movement
                self.soc.send("stop")
        elif was_stopped and not self.emergency_stop_active:
            self.get_logger().info("Emergency stop cleared")

    def reset_callback(self, msg):
        """Handle safety reset requests"""
        if msg.data and self.soc.isconnected:
            self.soc.send("resetSafetyStop")
            self.get_logger().info("Safety stop reset command sent to robot")

    def process_command_queue(self):
        """Worker thread to process command queue"""
        while rclpy.ok():
            if (self.soc.isconnected and self.command_queue and 
                (self.safe_to_move or not self.respect_safety) and 
                not self.emergency_stop_active):
                
                with self.command_queue_lock:
                    if self.command_queue:
                        cmd = self.command_queue.pop(0)
                        if not self.soc.send(cmd):
                            self.get_logger().error(f"Failed to send command: {cmd}")
            time.sleep(0.01)  # Small sleep to prevent CPU hogging

    def queue_command(self, cmd):
        """Add command to queue for sending"""
        with self.command_queue_lock:
            self.command_queue.append(cmd)
        return True

    def twist_callback(self, data):
        if not self.soc.isconnected:
            self.get_logger().warn("Cannot send twist command, robot not connected")
            return
            
        # Check safety if configured to do so
        if self.respect_safety and not self.safe_to_move:
            self.get_logger().warn("Ignoring twist command, robot is in unsafe state")
            return
            
        if self.emergency_stop_active:
            self.get_logger().warn("Ignoring twist command, emergency stop is active")
            return
            
        msg = f'setTwist {data.linear.x} {data.linear.y} {data.angular.z}'
        self.queue_command(msg)

    def pose_callback(self, data):
        if not self.soc.isconnected:
            self.get_logger().warn("Cannot send pose command, robot not connected")
            return
            
        # Check safety if configured to do so
        if self.respect_safety and not self.safe_to_move:
            self.get_logger().warn("Ignoring pose command, robot is in unsafe state")
            return
            
        if self.emergency_stop_active:
            self.get_logger().warn("Ignoring pose command, emergency stop is active")
            return
            
        msg = f'setPose {data.position.x} {data.position.y} {data.orientation.z}'
        self.queue_command(msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection', default='TCP', help='Connection type: TCP or UDP')
    parser.add_argument('-ro', '--robot', default='KMR1', help='Robot to connect to: KMR1 or KMR2')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    
    try:
        kmp_commands_node = KmpCommandsNode(args.connection, args.robot)
        rclpy.spin(kmp_commands_node)
    except KeyboardInterrupt:
        print(cl_yellow('Keyboard interrupt received, shutting down...'))
    except Exception as e:
        print(cl_red(f'Error: {e}'))
    finally:
        try:
            if 'kmp_commands_node' in locals():
                kmp_commands_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            print(cl_red('Error: rclpy shutdown failed'))


if __name__ == '__main__':
    main()
