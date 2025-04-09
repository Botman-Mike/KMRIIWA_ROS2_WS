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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():

    connection_type_TCP = 'TCP'
    connection_type_UDP = 'UDP'

    robot = "KMR2"
    
    # # Default parameter values to prevent warnings
    # default_params = {
    #     'port': 30000,  # Default port that will be overridden by specific node parameters
    #     'KMR2/ip': '172.31.1.206'  # Default IP address
    # }
    
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('kmr_communication'),
            'param',
            'bringup.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path to parameter file to load'),

#        launch_ros.actions.Node(
#            package="tf2_ros",
#            executable="static_transform_publisher",
#            output="screen",
#            arguments=['0','0','0','0','0','0','laser_B4_link','scan_2'],
#           ),

#        launch_ros.actions.Node(
#            package="tf2_ros",
#            executable="static_transform_publisher",
#            output="screen",
#            arguments=['0','0','0','0','0','0','laser_B1_link','scan'],
#           ),

        launch_ros.actions.Node(
            package="kmr_communication",
            executable="kmp_commands_node.py",
            name="kmp_commands_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP,'-ro', robot],
            parameters=[param_dir, {'port': 30002, 'KMR2/ip': '172.31.1.206'}]),

        launch_ros.actions.Node(
           package="kmr_communication",
           executable="kmp_laserscan_node.py",
           name="kmp_laserscan_node",
           output="screen",
           emulate_tty=True,
           arguments=['-c', connection_type_TCP, '-ro', robot],
           parameters=[param_dir, {'port': 30003, 'KMR2/ip': '172.31.1.206'}]),

        launch_ros.actions.Node(
           package="kmr_communication",
           executable="kmp_odometry_node.py",
           name="kmp_odometry_node",
           output="screen",
           emulate_tty=True,
           arguments=['-c', connection_type_TCP,'-ro',robot],
           parameters=[param_dir, {'port': 30004, 'KMR2/ip': '172.31.1.206'}]),

        launch_ros.actions.Node(
           package="kmr_communication",
           executable="kmp_statusdata_node.py",
           name="kmp_statusdata_node",
           output="screen",
           emulate_tty=True,
           arguments=['-c', connection_type_TCP, '-ro', robot],
           parameters=[param_dir, {'port': 30001, 'KMR2/ip': '172.31.1.206'}]),

        launch_ros.actions.Node(
            package="kmr_communication",
            executable="lbr_commands_node.py",
            name="lbr_commands_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot],
            parameters=[param_dir, {'port': 30005, 'KMR2/ip': '172.31.1.206'}]),

        launch_ros.actions.Node(
            package="kmr_communication",
            executable="lbr_statusdata_node.py",
            name="lbr_statusdata_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot],
            parameters=[param_dir, {'port': 30006, 'KMR2/ip': '172.31.1.206'}]),

        launch_ros.actions.Node(
            package="kmr_communication",
            executable="lbr_sensordata_node.py",
            name="lbr_sensordata_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot],
            parameters=[param_dir, {'port': 30007, 'KMR2/ip': '172.31.1.206'}]),
    ])
