#!/usr/bin/env python3

# Copyright 2020 Morten Melby Dahl.
# Copyright 2020 Norwegian University of Science and Technology.
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
import argparse
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.utilities import remove_ros_args
import time

# Import scripts
from cloud_transform import *
from laser_to_pointcloud import *

import rclpy
import numpy as np
import math
from rclpy.node import Node
from rclpy.duration import Duration  # Add this import for the Duration class

# Messages
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import std_msgs

# Use tf2_ros instead of tf
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import launch_ros.actions

# Message synchronization
from message_filters import ApproximateTimeSynchronizer
import message_filters


class LaserConcatenator(Node):

    def __init__(self, simulated):
        super().__init__('laser_concatenator')
        self.name = 'laser_concatenator'

        self.simulated = simulated.lower() == 'true'

        # Create QoS profile for sensor data
        qos_profile_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create publisher to publish final pointcloud
        self.publisher_ = self.create_publisher(
            PointCloud2, 
            'pc_concatenated', 
            qos_profile=qos_profile_sensor
        )

        # Setup for listening to transformations using tf2
        self.get_logger().info('Initializing TF buffer and listener.')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Listen to TF for 3 seconds
        start_time = time.time()
        self.get_logger().info('Listening to /tf and /tf_static for 3 seconds.')
        while (time.time() - start_time < 3.0):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Gets the transform message from original to goal frame.
        try:
            self.transform_B1 = self.tf_buffer.lookup_transform("base_footprint", "laser_B1_link", rclpy.time.Time(), timeout=Duration(seconds=1.0))
            self.get_logger().info(f'Got B1 transform: {self.transform_B1.child_frame_id} -> {self.transform_B1.header.frame_id}')
            self.transform_B4 = self.tf_buffer.lookup_transform("base_footprint", "laser_B4_link", rclpy.time.Time(), timeout=Duration(seconds=1.0))
            self.get_logger().info(f'Got B4 transform: {self.transform_B4.child_frame_id} -> {self.transform_B4.header.frame_id}')
        except tf2_ros.LookupException as e:
            self.get_logger().error(f'Error looking up transform: {e}\nDid you remember to launch the publisher from kmr_bringup?')
            self.get_logger().error('Shutting down.')
            rclpy.shutdown()

        if (self.transform_B1.header.frame_id != self.transform_B4.header.frame_id):
            self.get_logger().warn("Warning: B1 and B4 transformations do not share the same goal frame. Concatenation not possible.")
            rclpy.shutdown()

        # Generate transformation matrices in the form of NumPy arrays from TransformStamped
        self.T1 = CloudTransform().generate_transform(self.transform_B1)
        self.T4 = CloudTransform().generate_transform(self.transform_B4)

        # Subscribe to the two laser scan topics with the sensor QoS profile
        self.subscriber_1 = message_filters.Subscriber(self, LaserScan, 'scan', qos_profile=qos_profile_sensor)
        self.subscriber_2 = message_filters.Subscriber(self, LaserScan, 'scan_2', qos_profile=qos_profile_sensor)

        # Synchronize messages from the two laser topics
        self.synchronizer = ApproximateTimeSynchronizer([self.subscriber_1, self.subscriber_2], 10, 0.01, allow_headerless=False)
        self.get_logger().info('Initialized laser scan synchronizer.')
        
        # Register the callback for synchronized messages
        self.synchronizer.registerCallback(self.callback)

    def callback(self, scan, scan2):
        # Project laser scans into PointCloud2 messages
        pc2_msg1 = LaserToPointcloud().projectLaser(scan)
        pc2_msg2 = LaserToPointcloud().projectLaser(scan2)

        # Transform the clouds to the same frame
        pc2_msg1_transformed = CloudTransform().do_transform_cloud(pc2_msg1, self.T1, scan, self.simulated)
        pc2_msg2_transformed = CloudTransform().do_transform_cloud(pc2_msg2, self.T4, scan, self.simulated)

        # Concatenate the clouds
        pc2_concatenated = LaserToPointcloud().concatenate_clouds(pc2_msg1_transformed, pc2_msg2_transformed)
        pc2_concatenated.header.frame_id = self.transform_B1.header.frame_id

        # Publish the combined cloud
        self.publisher_.publish(pc2_concatenated)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-sim', '--simulated')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    concatenator_node = LaserConcatenator(args.simulated)
    rclpy.spin(concatenator_node)

    try:
        concatenator_node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print("Error: rclpy shutdown failed", e)


if __name__ == '__main__':
    main()