#!/bin/bash

# Define workspace path
WORKSPACE_PATH="/home/ros2-control/kmriiwa_ws_devel/src"

# Create object_msgs package
cd $WORKSPACE_PATH
ros2 pkg create --build-type ament_cmake object_msgs
mkdir -p object_msgs/msg

# Create message files for object_msgs
cat > object_msgs/msg/ObjectInBox.msg << EOF
# Object in 2D box
string object.data           # Object name
float32 probability          # Confidence score
sensor_msgs/RegionOfInterest roi    # Region of interest
EOF

cat > object_msgs/msg/ObjectsInBoxes.msg << EOF
# Objects detected within their 2D bounding boxes
std_msgs/Header header       # Header stamp should be acquisition time of image
                             # Header frame_id should be camera frame
object_msgs/ObjectInBox[] objects_vector # List of objects detected
EOF

# Setup CMakeLists.txt for object_msgs
cat > object_msgs/CMakeLists.txt << EOF
cmake_minimum_required(VERSION 3.8)
project(object_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(\${PROJECT_NAME}
  "msg/ObjectInBox.msg"
  "msg/ObjectsInBoxes.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

ament_package()
EOF

# Setup package.xml for object_msgs
cat > object_msgs/package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>object_msgs</name>
  <version>0.1.0</version>
  <description>Object detection messages compatible with ROS 2 Humble</description>
  <maintainer email="user@example.com">maintainer</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  
  <member_of_group>rosidl_interface_packages</member_of_group>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Create object_analytics_msgs package
cd $WORKSPACE_PATH
ros2 pkg create --build-type ament_cmake object_analytics_msgs
mkdir -p object_analytics_msgs/msg

# Create message files for object_analytics_msgs
cat > object_analytics_msgs/msg/ObjectInBox3D.msg << EOF
# Object in 3D box
string object.data           # Object name
float32 probability          # Confidence score
sensor_msgs/RegionOfInterest roi    # Region of interest
geometry_msgs/Point32 min    # Min point of 3D box
geometry_msgs/Point32 max    # Max point of 3D box
EOF

cat > object_analytics_msgs/msg/ObjectsInBoxes3D.msg << EOF
# Objects detected with 3D info
std_msgs/Header header       # Header stamp should be acquisition time of image
                             # Header frame_id should be camera frame
object_analytics_msgs/ObjectInBox3D[] objects_in_boxes  # List of objects
EOF

# Setup CMakeLists.txt for object_analytics_msgs
cat > object_analytics_msgs/CMakeLists.txt << EOF
cmake_minimum_required(VERSION 3.8)
project(object_analytics_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(\${PROJECT_NAME}
  "msg/ObjectInBox3D.msg"
  "msg/ObjectsInBoxes3D.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_package()
EOF

# Setup package.xml for object_analytics_msgs
cat > object_analytics_msgs/package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>object_analytics_msgs</name>
  <version>0.1.0</version>
  <description>Object analytics messages compatible with ROS 2 Humble</description>
  <maintainer email="user@example.com">maintainer</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  
  <member_of_group>rosidl_interface_packages</member_of_group>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Create pipeline_srv_msgs package
cd $WORKSPACE_PATH
ros2 pkg create --build-type ament_cmake pipeline_srv_msgs
mkdir -p pipeline_srv_msgs/msg
mkdir -p pipeline_srv_msgs/srv

# Create message and service files for pipeline_srv_msgs
cat > pipeline_srv_msgs/msg/PipelineRequest.msg << EOF
string pipeline_name         # Name of requested processing pipeline
string processing_type       # Type of processing requested
string[] parameters          # Additional parameters for the pipeline
EOF

cat > pipeline_srv_msgs/srv/PipelineSrv.srv << EOF
# Request
pipeline_srv_msgs/PipelineRequest pipeline_request
sensor_msgs/Image input_image
---
# Response
bool success
string error_msg
sensor_msgs/Image output_image
EOF

# Setup CMakeLists.txt for pipeline_srv_msgs
cat > pipeline_srv_msgs/CMakeLists.txt << EOF
cmake_minimum_required(VERSION 3.8)
project(pipeline_srv_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(\${PROJECT_NAME}
  "msg/PipelineRequest.msg"
  "srv/PipelineSrv.srv"
  DEPENDENCIES sensor_msgs std_msgs
)

ament_package()
EOF

# Setup package.xml for pipeline_srv_msgs
cat > pipeline_srv_msgs/package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>pipeline_srv_msgs</name>
  <version>0.1.0</version>
  <description>Pipeline service messages compatible with ROS 2 Humble</description>
  <maintainer email="user@example.com">maintainer</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  
  <member_of_group>rosidl_interface_packages</member_of_group>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Create kmr_msgs package 
cd $WORKSPACE_PATH
ros2 pkg create --build-type ament_cmake kmr_msgs
mkdir -p kmr_msgs/action

# Create action file for kmr_msgs
cat > kmr_msgs/action/ObjectSearch.action << EOF
# Goal definition
string object_name
float32 min_confidence
---
# Result definition
bool success
string message
object_analytics_msgs/ObjectInBox3D[] detected_objects
---
# Feedback definition
string current_state
int32 objects_found
float32 search_progress
EOF

# Setup CMakeLists.txt for kmr_msgs
cat > kmr_msgs/CMakeLists.txt << EOF
cmake_minimum_required(VERSION 3.8)
project(kmr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(object_analytics_msgs REQUIRED)

rosidl_generate_interfaces(\${PROJECT_NAME}
  "action/ObjectSearch.action"
  DEPENDENCIES std_msgs action_msgs object_analytics_msgs
)

ament_package()
EOF

# Setup package.xml for kmr_msgs
cat > kmr_msgs/package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>kmr_msgs</name>
  <version>0.1.0</version>
  <description>KMR robot messages compatible with ROS 2 Humble</description>
  <maintainer email="user@example.com">maintainer</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <depend>std_msgs</depend>
  <depend>action_msgs</depend>
  <depend>object_analytics_msgs</depend>
  
  <member_of_group>rosidl_interface_packages</member_of_group>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

echo "Message packages created with appropriate definitions for ROS 2 Humble."
echo "Now build your workspace with: colcon build --symlink-install"
