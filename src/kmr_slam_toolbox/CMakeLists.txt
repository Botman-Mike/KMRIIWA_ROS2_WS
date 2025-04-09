cmake_minimum_required(VERSION 3.10)
project(kmr_slam_toolbox)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

############################### rviz plugins ###############################
# Qt6 configuration - order matters
set(CMAKE_AUTOMOC ON)
# This must be set BEFORE finding Qt6
set(QT_NO_CREATE_VERSIONLESS_TARGETS ON)
set(QT_NO_CREATE_VERSIONLESS_FUNCTIONS ON)

# Find packages in the correct order
find_package(Qt6 COMPONENTS Core Widgets REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

install(FILES rviz_plugins.xml
  DESTINATION share
)

#find_package(nav2_map_server REQUIRED)
#find_package(tf2_sensor_msgs REQUIRED)
#find_package(std_srvs REQUIRED)
#find_package(slam_toolbox REQUIRED)


#set(QT_LIBRARIES Qt5::Widgets Qt5::Core)
#macro(qt_wrap_ui)
#  qt5_wrap_ui(${ARGN})
#endmacro()
#macro(qt_wrap_cpp)
#  qt5_wrap_cpp(${ARGN})
#endmacro()
#
#set(dependencies
#  rclcpp
#  message_filters
#  nav_msgs
#  sensor_msgs
#  tf2
#  tf2_ros
#  visualization_msgs
#  pluginlib
#  nav2_map_server
#  tf2_geometry_msgs
#  tf2_sensor_msgs
#  std_msgs
#  std_srvs
#  builtin_interfaces
#  rviz_common
#  rviz_rendering
#  rviz_default_plugins
#  #interactive_markers
#)

#qt_wrap_cpp(MOC_FILES rviz_plugin/slam_toolbox_rviz_plugin.h)
#set(SOURCE_FILES rviz_plugin/slam_toolbox_rviz_plugin.cpp )
#set(HEADER_FILES rviz_plugin/slam_toolbox_rviz_plugin.h )
#add_library(SlamToolboxPlugin ${SOURCE_FILES} ${HEADER_FILES})
#ament_target_dependencies(SlamToolboxPlugin
#  ${dependencies}
#)
#target_link_libraries(SlamToolboxPlugin kartoSlamToolbox ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES})
#rosidl_target_interfaces(SlamToolboxPlugin ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_compile_definitions(SlamToolboxPlugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
#pluginlib_export_plugin_description_file(kmr_slam_toolbox rviz_plugins.xml)


############################### ---------- ###############################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch config rviz created_maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    DESTINATION lib/${PROJECT_NAME}
)


################################################################################

ament_package()

