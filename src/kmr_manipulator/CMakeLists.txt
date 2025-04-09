cmake_minimum_required(VERSION 3.8)
project(kmr_manipulator)

# Default to C++17 for ROS 2 Humble
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(action_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# Add your custom packages
find_package(kmr_msgs REQUIRED)
find_package(pipeline_srv_msgs REQUIRED)
find_package(object_analytics_msgs REQUIRED) 
find_package(object_msgs REQUIRED)

# Install Python modules
ament_python_install_package("script")

# Install Python executables
install(PROGRAMS
  nodes/realsense/object_detection_node.py
  # Add other Python scripts here
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if you have any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Comment out or remove the installation of config directory
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}/
# )

# Install message definition examples for reference
install(DIRECTORY
  msg_definitions
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
