import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package
    pkg_dir = get_package_share_directory('kmr_communication')
    
    # Declare launch arguments
    connection_type = LaunchConfiguration('connection_type')
    robot_name = LaunchConfiguration('robot_name')
    
    connection_type_arg = DeclareLaunchArgument(
        'connection_type',
        default_value='TCP',
        description='Connection type (TCP/UDP)'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='KMR1',
        description='Robot name (KMR1/KMR2)'
    )
    
    # Create node actions
    kmp_commands_node = Node(
        package='kmr_communication',
        executable='kmp_commands_node.py',
        name='kmp_commands_node',
        arguments=['-c', connection_type, '-ro', robot_name],
        parameters=[{'respect_safety': True}]
    )
    
    lbr_commands_node = Node(
        package='kmr_communication',
        executable='lbr_commands_node.py',
        name='lbr_commands_node',
        arguments=['-c', connection_type, '-ro', robot_name],
        parameters=[{'respect_safety': True}]
    )
    
    kmp_statusdata_node = Node(
        package='kmr_communication',
        executable='kmp_statusdata_node.py',
        name='kmp_statusdata_node',
        arguments=['-c', connection_type, '-ro', robot_name]
    )
    
    lbr_statusdata_node = Node(
        package='kmr_communication',
        executable='lbr_statusdata_node.py',
        name='lbr_statusdata_node',
        arguments=['-c', connection_type, '-ro', robot_name]
    )
    
    kmp_odometry_node = Node(
        package='kmr_communication',
        executable='kmp_odometry_node.py',
        name='kmp_odometry_node',
        arguments=['-c', connection_type, '-ro', robot_name]
    )
    
    kmp_laserscan_node = Node(
        package='kmr_communication',
        executable='kmp_laserscan_node.py',
        name='kmp_laserscan_node',
        arguments=['-c', connection_type, '-ro', robot_name]
    )
    
    lbr_sensordata_node = Node(
        package='kmr_communication',
        executable='lbr_sensordata_node.py',
        name='lbr_sensordata_node',
        arguments=['-c', connection_type, '-ro', robot_name]
    )
    
    # Health and safety monitoring nodes
    communication_health_monitor = Node(
        package='kmr_communication',
        executable='communication_health_monitor.py',
        name='communication_health_monitor',
        parameters=[{
            'check_interval': 1.0,
            'timeout_threshold': 3.0
        }]
    )
    
    safety_monitor = Node(
        package='kmr_communication',
        executable='safety_monitor.py',
        name='safety_monitor',
        parameters=[{
            'check_interval': 0.1,
            'auto_reset': False,
            'auto_reset_delay': 5.0
        }]
    )
    
    emergency_stop_bridge = Node(
        package='kmr_communication',
        executable='emergency_stop_bridge.py',
        name='emergency_stop_bridge',
        parameters=[{
            'check_interval': 0.1
        }]
    )
    
    # Return the launch description
    return LaunchDescription([
        connection_type_arg,
        robot_name_arg,
        kmp_commands_node,
        lbr_commands_node,
        kmp_statusdata_node,
        lbr_statusdata_node,
        kmp_odometry_node,
        kmp_laserscan_node, 
        lbr_sensordata_node,
        communication_health_monitor,
        safety_monitor,
        emergency_stop_bridge
    ])
