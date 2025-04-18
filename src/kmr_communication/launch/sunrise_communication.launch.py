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
import sys

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

    # Make robot name configurable via launch argument
    robot_name = LaunchConfiguration('robot_name', default='KMR1')
    
    # Bind to all interfaces to accept connections from any machine
    bind_ip = LaunchConfiguration('bind_ip', default='0.0.0.0')
    
    # The actual robot IP - used for outgoing connections
    robot_ip = LaunchConfiguration('robot_ip', default='172.31.1.10')
    
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('kmr_communication'),
            'param',
            'bringup.yaml'))

    # Create the launch description
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path to parameter file to load'),
            
        DeclareLaunchArgument(
            'robot_name',
            default_value='KMR1',
            description='Robot name (KMR1/KMR2)'),
            
        DeclareLaunchArgument(
            'bind_ip',
            default_value='0.0.0.0',
            description='IP address to bind server sockets to (0.0.0.0 = all interfaces)'),
            
        DeclareLaunchArgument(
            'robot_ip',
            default_value='172.31.1.10',
            description='IP address of the actual robot (for outgoing connections)'),

        # TF transformations for laser scanners - uncommented to ensure proper coordinate frames
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=['0','0','0','0','0','0','laser_B4_link','scan_2'],
           ),

        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=['0','0','0','0','0','0','laser_B1_link','scan'],
           ),

        # Command nodes - keep as TCP for reliability
        launch_ros.actions.Node(
            package="kmr_communication",
            executable="kmp_commands_node.py",
            name="kmp_commands_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP,'-ro', robot_name],
            parameters=[param_dir, {'port': 30002, 'ip': bind_ip, 'robot_ip': robot_ip, 'respect_safety': True}]),

        # Sensor nodes - changed to UDP for better performance
        launch_ros.actions.Node(
           package="kmr_communication",
           executable="kmp_laserscan_node.py",
           name="kmp_laserscan_node",
           output="screen",
           emulate_tty=True,
           arguments=['-c', connection_type_UDP, '-ro', robot_name],
           parameters=[param_dir, {'port': 30003, 'ip': bind_ip, 'robot_ip': robot_ip}]),

        launch_ros.actions.Node(
           package="kmr_communication",
           executable="kmp_odometry_node.py",
           name="kmp_odometry_node",
           output="screen",
           emulate_tty=True,
           arguments=['-c', connection_type_UDP,'-ro', robot_name],
           parameters=[param_dir, {'port': 30004, 'ip': bind_ip, 'robot_ip': robot_ip}]),

        # Status nodes - keep as TCP for reliability
        launch_ros.actions.Node(
           package="kmr_communication",
           executable="kmp_statusdata_node.py",
           name="kmp_statusdata_node",
           output="screen",
           emulate_tty=True,
           arguments=['-c', connection_type_TCP, '-ro', robot_name],
           parameters=[param_dir, {'port': 30001, 'ip': bind_ip, 'robot_ip': robot_ip}]),

        # LBR command node - keep as TCP
        launch_ros.actions.Node(
            package="kmr_communication",
            executable="lbr_commands_node.py",
            name="lbr_commands_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot_name],
            parameters=[param_dir, {'port': 30005, 'ip': bind_ip, 'robot_ip': robot_ip, 'respect_safety': True}]),

        # LBR status node - keep as TCP
        launch_ros.actions.Node(
            package="kmr_communication",
            executable="lbr_statusdata_node.py",
            name="lbr_statusdata_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot_name],
            parameters=[param_dir, {'port': 30006, 'ip': bind_ip, 'robot_ip': robot_ip}]),

        # LBR sensor node - changed to UDP for better performance
        launch_ros.actions.Node(
            package="kmr_communication",
            executable="lbr_sensordata_node.py",
            name="lbr_sensordata_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_UDP, '-ro', robot_name],
            parameters=[param_dir, {'port': 30007, 'ip': bind_ip, 'robot_ip': robot_ip}]),
    ])

    # Try to add monitoring nodes only if they exist
    pkg_prefix = get_package_share_directory('kmr_communication')
    lib_dir = os.path.join(pkg_prefix, 'lib', 'kmr_communication')
    
    # Add nodes if they exist, otherwise skip them
    if os.path.exists(os.path.join(lib_dir, 'emergency_stop_bridge.py')):
        ld.add_action(
            launch_ros.actions.Node(
                package="kmr_communication",
                executable="emergency_stop_bridge.py",
                name="emergency_stop_bridge",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    'check_interval': 0.1
                }])
        )
    else:
        print(f"Warning: emergency_stop_bridge.py not found in {lib_dir}, skipping.")
        
    if os.path.exists(os.path.join(lib_dir, 'safety_monitor.py')):
        ld.add_action(
            launch_ros.actions.Node(
                package="kmr_communication",
                executable="safety_monitor.py",
                name="safety_monitor",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    'check_interval': 0.1,
                    'auto_reset': False,
                    'auto_reset_delay': 5.0
                }])
        )
    else:
        print(f"Warning: safety_monitor.py not found in {lib_dir}, skipping.")
        
    if os.path.exists(os.path.join(lib_dir, 'communication_health_monitor.py')):
        ld.add_action(
            launch_ros.actions.Node(
                package="kmr_communication",
                executable="communication_health_monitor.py",
                name="communication_health_monitor",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    'check_interval': 1.0,
                    'timeout_threshold': 600.0  # INCREASED TO 10 MINUTES FOR TROUBLESHOOTING
                }])
        )
    else:
        print(f"Warning: communication_health_monitor.py not found in {lib_dir}, skipping.")
        
    return ld
