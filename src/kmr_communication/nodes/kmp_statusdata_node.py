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
import time
import sys
import math
import rclpy
from rclpy.node import Node
from kmr_msgs.msg import KmpStatusdata
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from tcpSocket import TCPSocket
from udpSocket import UDPSocket
from std_msgs.msg import Bool

from rclpy.utilities import remove_ros_args
import argparse

def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class KmpStatusNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('kmp_statusdata_node')
        self.name='kmp_statusdata_node'
        self.last_status_timestamp = None
        self.declare_parameter('port', 30001)
        port = int(self.get_parameter('port').value)
        self.get_logger().info(f"{self.get_name()} listening on port {port}")
        if robot == 'KMR1':
            self.declare_parameter('KMR1/ip', '172.31.1.206')
            ip = str(self.get_parameter('KMR1/ip').value)
        elif robot == 'KMR2':
            self.declare_parameter('KMR2/ip', '172.31.1.206')
            ip = str(self.get_parameter('KMR2/ip').value)
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
        self.pub_kmp_statusdata = self.create_publisher(KmpStatusdata, 'kmp_statusdata', 10)
        # Connection status publisher (for diagnostics, like LBR node)
        self.connection_status_pub = self.create_publisher(Bool, 'kmp_connected', 10)
        self.create_timer(1.0, self.publish_connection_status)
        # Start a timer to poll for data if connected
        self.create_timer(0.05, self.poll_statusdata)

    def poll_statusdata(self):
        if self.soc and self.soc.isconnected and self.soc.kmp_statusdata:
            self.status_callback(self.pub_kmp_statusdata, self.soc.kmp_statusdata)

    def status_callback(self, publisher, data):
        if data is None:
            return
        # split into elements and validate
        status_elements = data[1].split(",")
        if len(status_elements) < 2:
            self.get_logger().warn(f"{self.get_name()}: malformed statusdata: {status_elements}")
            return
        timestamp = status_elements[1]
        # process only on new timestamp
        if self.last_status_timestamp is None or timestamp != self.last_status_timestamp:
            self.last_status_timestamp = timestamp
            msg = KmpStatusdata()
            msg.header.stamp = self.get_clock().now().to_msg()
            for i in range(2, len(status_elements)):
                split = status_elements[i].split(":")
                if(split[0]=="OperationMode"):
                    msg.operation_mode = split[1]
                elif (split[0] == "ReadyToMove"):
                    if (split[1] == "true"):
                        msg.ready_to_move = True
                    else:
                        msg.ready_to_move = False
                elif (split[0] == "WarningField"):
                    if (split[1] == "true"):
                        msg.warning_field_clear = True
                    else:
                        msg.warning_field_clear = False
                elif (split[0] == "ProtectionField"):
                    if (split[1] == "true"):
                        msg.protection_field_clear = True
                    else:
                        msg.protection_field_clear = False
                elif (split[0] == "isKMPmoving"):
                    if (split[1] == "true"):
                        msg.is_kmp_moving = True
                    else:
                        msg.is_kmp_moving = False
                elif (split[0] == "KMPsafetyStop"):
                    if (split[1] == "true"):
                        msg.kmp_safetystop = True
                    else:
                        msg.kmp_safetystop = False
            publisher.publish(msg)

    def publish_connection_status(self):
        msg = Bool()
        msg.data = self.soc.isconnected if self.soc else False
        self.connection_status_pub.publish(msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    kmp_statusdata_node = KmpStatusNode(args.connection,args.robot)

    rclpy.spin(kmp_statusdata_node)
    # Ensure socket closed before shutdown
    try:
        kmp_statusdata_node.soc.close()
    except Exception:
        pass

    try:
        kmp_statusdata_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
