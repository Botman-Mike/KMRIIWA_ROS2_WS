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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    """Launch the monitoring nodes for the KMR communication system."""
    
    return LaunchDescription([
        # Health and safety monitoring nodes
        launch_ros.actions.Node(
            package="kmr_communication",
            executable="communication_health_monitor.py",
            name="communication_health_monitor",
            output="screen",
            emulate_tty=True,
            parameters=[{
                'check_interval': 1.0,
                'timeout_threshold': 3.0
            }]),
            
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
            }]),
            
        launch_ros.actions.Node(
            package="kmr_communication",
            executable="emergency_stop_bridge.py",
            name="emergency_stop_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[{
                'check_interval': 0.1
            }]),
    ])
