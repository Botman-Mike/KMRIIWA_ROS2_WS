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

import _thread as thread
import sys
import math
import time
import rclpy
from rclpy.node import Node
from kmr_msgs.msg import LbrStatusdata
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from tcpSocket import TCPSocket
from udpSocket import UDPSocket
from std_msgs.msg import Bool

from rclpy.utilities import remove_ros_args
import argparse

def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class LbrStatusNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('lbr_statusdata_node')
        self.name='lbr_statusdata_node'
        self.last_status_timestamp = 0
        self.path_finished = False
        self.declare_parameter('port', 30006)
        port = self.get_parameter('port').value
        if robot == 'KMR1':
            self.declare_parameter('KMR1/ip', '172.31.1.206')
            ip = self.get_parameter('KMR1/ip').value
        elif robot == 'KMR2':
            self.declare_parameter('KMR2/ip', '172.31.1.206')
            ip = self.get_parameter('KMR2/ip').value
        else:
            ip = None


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
            self.soc=None


        # Make Publisher for statusdata
        self.pub_lbr_statusdata = self.create_publisher(LbrStatusdata, 'lbr_statusdata', 10)
        
        # Connection status publisher
        self.connection_status_pub = self.create_publisher(Bool, 'lbr_connected', 10)

        self.create_timer(0.05, self.poll_statusdata)

    def poll_statusdata(self):
        if self.soc and self.soc.isconnected and self.soc.lbr_statusdata:
            try:
                self.status_callback(self.pub_lbr_statusdata, self.soc.lbr_statusdata)
            except Exception as e:
                self.get_logger().error(f"Error processing status data: {e}")

    def status_callback(self, status_publisher, data):
        if data is None or len(data) < 2:
            return
            
        try:
            msg = LbrStatusdata()
            msg.header.stamp = self.get_clock().now().to_msg()
            
            # Validate data has expected format
            status_elements = data[1].split(",")
            if len(status_elements) < 2:
                return
                
            if (status_elements[1] != self.last_status_timestamp):
                self.last_status_timestamp = status_elements[1]
                
                for i in range(2, len(status_elements)):
                    split = status_elements[i].split(":")
                    
                    # Make sure we have both key and value
                    if len(split) < 2:
                        continue
                        
                    if (split[0] == "ReadyToMove"):
                        if (split[1] == "true"):
                            msg.ready_to_move = True
                        else:
                            msg.ready_to_move = False
                    elif (split[0] == "isLBRmoving"):
                        if (split[1] == "true"):
                            msg.is_lbr_moving = True
                            self.soc.is_lbr_moving = True
                        else:
                            msg.is_lbr_moving = False
                            self.soc.is_lbr_moving = False
                    elif (split[0] == "PathFinished"):
                        if (split[1] == "true"):
                            msg.path_finished = True
                        else:
                            msg.path_finished = False
                    elif (split[0] == "LBRsafetyStop"):
                        if (split[1] == "true"):
                            msg.lbr_safetystop = True
                        else:
                            msg.lbr_safetystop = False
                            
                status_publisher.publish(msg)
                
                # Publish connection status
                status_msg = Bool()
                status_msg.data = True
                self.connection_status_pub.publish(status_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error in status_callback: {e}")


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    lbr_statusdata_node = LbrStatusNode(args.connection,args.robot)

    rclpy.spin(lbr_statusdata_node)
    # Ensure socket closed before shutdown
    try:
        lbr_statusdata_node.soc.close()
    except Exception:
        pass

    try:
        lbr_statusdata_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
