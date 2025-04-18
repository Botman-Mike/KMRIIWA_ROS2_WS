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
from std_msgs.msg import String
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
        elif connection_type == 'UDP':
            self.soc = UDPSocket(ip,port,self.name)
        else:
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
        connection_timeout = 600.0  # INCREASED TO 10 MINUTES FOR TROUBLESHOOTING
        
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
        print(data)
        msg = "shutdown"
        self.soc.send(msg)
        #self.udp_soc.isconnected = False

    def twist_callback(self, data):
        msg = 'setTwist ' + str(data.linear.x) + " " + str(data.linear.y) + " " + str(data.angular.z)
        self.soc.send(msg)

    def pose_callback(self, data):
        msg = 'setPose ' + str(data.position.x) + " " + str(data.position.y) + " " + str(data.orientation.z)
        self.soc.send(msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    kmp_commands_node = KmpCommandsNode(args.connection,args.robot)

    rclpy.spin(kmp_commands_node)

    try:
        kmp_commands_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
