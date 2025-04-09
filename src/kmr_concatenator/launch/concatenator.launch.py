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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import sys
from rclpy.utilities import remove_ros_args
import argparse


def generate_launch_description(argv=sys.argv[1:]):

    # Declare the simulation argument with default value set to true
    simulated_arg = DeclareLaunchArgument(
        'simulated',
        default_value='true',  # Change to 'true' for simulation
        description='Set to "true" for simulation, "false" for real robot'
    )

    # Launch the concatenator node with the simulation flag
    concatenator_node = launch_ros.actions.Node(
        package='kmr_concatenator',
        executable='concatenator_node.py',
        name='concatenator_node',
        output='screen',
        parameters=[],
        arguments=['-sim', 'true']  # Changed from 'false' to 'true' for simulation mode
    )

    return LaunchDescription([
        simulated_arg,
        concatenator_node,
        launch_ros.actions.Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            namespace="",
            name="pointcloud_to_laserscan",
            output="screen",
            parameters=[{
                'use_sim_time': True,
                'range_max': 10.0,
                'range_min': 0.3
            }],
            remappings=[('cloud_in', 'pc_concatenated'),
                        ('scan', 'scan_concatenated')]
            )
    ])
