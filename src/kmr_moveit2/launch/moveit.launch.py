import os
import yaml
import shutil  # Import shutil to check for executables
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Check if taskset is available and set prefix accordingly
    taskset_path = shutil.which('taskset')
    p_core_prefix = ''
    e_core_prefix = ''
    if taskset_path:
        print(f"Taskset found at {taskset_path}, applying core pinning.")
        # Note the trailing space in the prefixes
        p_core_prefix = 'taskset -c 0-7 ' 
        e_core_prefix = 'taskset -c 8-19 '
    else:
        print("Taskset command not found, skipping core pinning.")

    # Load configuration files from kmr_moveit2 and kmr_bringup
    moveit_cpp_yaml_file = os.path.join(get_package_share_directory('kmr_moveit2'),
                                          "config", "moveit_cpp.yaml")
    robot_description_config = load_file('kmr_bringup', 'urdf/kmriiwa.urdf')
    robot_description = {'robot_description': robot_description_config}

    robot_description_semantic_config = load_file('kmr_moveit2', 'config/iiwa14.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('kmr_moveit2', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    controllers_yaml = load_yaml('kmr_moveit2', 'config/controllers.yaml')
    moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml}

    ompl_planning_pipeline_config = {'ompl': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization 
default_planner_request_adapters/FixWorkspaceBounds 
default_planner_request_adapters/FixStartStateBounds 
default_planner_request_adapters/FixStartStateCollision 
default_planner_request_adapters/FixStartStatePathConstraints""",
        'start_state_max_bounds_error': 0.1
    }}
    ompl_planning_yaml = load_yaml('kmr_moveit2', 'config/ompl_planning.yaml')
    if ompl_planning_yaml:
        ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    state_publisher_launch_file_dir = os.path.join(get_package_share_directory('kmr_bringup'), 'launch')

    # Launch MoveIt components
    run_moveit_node = Node(
        name='run_moveit',
        package='kmr_moveit2',
        executable='run_moveit',
        prefix=p_core_prefix,  # Add P-core prefix here
        output='screen',
        emulate_tty=True,
        parameters=[moveit_cpp_yaml_file,
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    ompl_planning_pipeline_config,
                    moveit_controllers]
    )

    # RViz node (loads robot description)
    rviz_config_file = os.path.join(get_package_share_directory('kmr_moveit2'),
                                    "rviz", "moveit.rviz")
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        prefix=e_core_prefix,  # Add E-core prefix here
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description]
    )

    # Publish static transform from world to base_footprint for consistency
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_footprint']
    )

    # Launch fake joint driver node using configurations from kmr_moveit2
    fake_joint_driver_node = Node(
        package='fake_joint_driver',
        executable='fake_joint_driver_node',
        prefix=p_core_prefix,  # Add P-core prefix here
        parameters=[os.path.join(get_package_share_directory("kmr_moveit2"), "config", "iiwa_controllers.yaml"),
                    os.path.join(get_package_share_directory("kmr_moveit2"), "config", "start_positions.yaml"),
                    robot_description],
        output='screen'
    )

    return LaunchDescription([
        static_tf,
        rviz_node,
        run_moveit_node,
        fake_joint_driver_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [state_publisher_launch_file_dir, '/state_publisher.launch.py']
            )
        )
    ])
