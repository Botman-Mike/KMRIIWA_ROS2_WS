# kmriiwa_ws_devel
<img align="right" src="https://img.directindustry.com/images_di/photo-g/17587-12407502.webp" height="250"/>
Forked from a repository for master thesis and specialization project in Robotics & Automation at NTNU (Fall 2019) by Charlotte Heggem and Nina Marie Wahl. 

Now further developed by Mike Wescott of Spirit Aerosystems for ROS 2 Humble.  

This is still a work in progress.

Use this repo on the command PC and use KMRIIWA_ROS2_JAVA as the Sunrise Workbench project on the KMR IIWA robot.

**Intention:**
This project aims to further develop the communication API between a KUKA KMR iiwa and ROS2. 
Multiple ROS packages are used for including functionality. 
Navigation2 is used for navigating the mobile vehicle. 
SLAM_Toolbox is used for SLAM.
MoveIt2 is used for path planning for the manipulator. 

A Robotiq gripper is used for manipulating objects. 
Four Intel Realsense D435 cameras are mounted to the robot.
If cameras and gripper are to be used, they are launched on a separate onboard computer (Intel NUC). 

**System requirements:** 
- Ubuntu 20.04
- Python 3.8.5
- ROS2 'Humble Hawksbill'


**Required ROS Packages:**
- Gazebo packages
- Navigation2
- (MoveIt2)
- laserscan_to_pointcloud
- (Ros2 Intel Realsense (ROS2 Wrapper for Intel® RealSense™ Devices))
- (ROS2 Openvino Toolkit (dependent on OpenVino Toolkit))
- (ROS2 Object Analytics)

## Guide
All the packages inside the src folder are made to enable the use of a KMR iiwa robot with ROS2. A simple description of all the packages and a quick-start guide follows.
### kmr_behaviortree
The package is made to enable the use of behaviortrees with the KMR. [Behaviortree.CPP](https://www.behaviortree.dev/) interfaces with Navigation2 and MoveIt2, which makes it simple to make pre-installed programs.

### kmr_bringup
This package publises the transformation between different parts of the robot. This is required when odometry and laser scan data needs to be expressed in different frames of the robot.

### kmr_communication
To establish communication between the KMR and ROS2, this package sets up a TCP or UDP connection which receives and sends information between the two systems.

### kmr_concatenator
As the preferred SLAM software SLAM_Toolbox only takes in data from a single LaserScan topic, data from the two laser scanners on the KMR has to be transformed and concatenated. This package concatenates the data by converting between LaserScan and PointCloud2 and transforming the individual points before adding them back together. It is also supported by the 'laserscan_to_pointcloud' package with the pointcloud_to_laserscan_node.

### kmr_manipulator
This package was developed to enable the use of a Robotiq gripper outside of the KUKA system. It takes commands from a topic and translates them into movement of the gripper.

### kmr_moveit2
This package enables trajectory generation for the iiwa arm by integrating MoveIt2.

### kmr_msgs
Custom messages used for actions and moving the gripper.

### kmr_navigation2
Enables the use of Navigation2 with the KMR.

### kmr_simulation
Simulate the KMR iiwa in Gazebo.

### kmr_slam
Enables both RTAB-Map and Cartographer to be used with the robot. This is not the preferred SLAM method, and better results have been achieved by using SLAM_Toolbox. Currently not tested on the Foxy build.

### kmr_slam_toolbox
Integrates the use of [SLAM_Toolbox](https://github.com/SteveMacenski/slam_toolbox/) with the robot. A requirement to run this package is to have kmr_concatenator running in order to get input from both lasers. Further instructions inside the package.

### kmr_sunrise
Code which is installed on the KMR through KUKA Workbench. It enables communication with ROS over a TCP or UDP connection.

### kmr_sunrise_original
The original code from which the updated kmr_sunrise code is based upon.

### Quick-start guide
Currently, only the packages that enable mapping of an environment using [SLAM_Toolbox](https://github.com/SteveMacenski/slam_toolbox/) and Navigation2 has been enabled for use in ROS2 'Foxy Fitzroy'. Other packages, such as MoveIt2, are yet to be fully tested with the current version.

After building, remember to source the setup file (install/setup.bash).

To launch the robot in a simulated environment, simply run
```
$ ros2 launch kmr_simulation gazebo.launch.py
```
You should now see the robot visualized in Gazebo. 

If you wish to use a real KMR, run
```
$ ros2 launch kmr_communication sunrise_communication.launch.py
```
Now run the KMRiiwaSunriseApplication.java application on the KMR. 
It's a requirement that you have both the computer running ROS2 and the KMR connected to the same access point and have the correct IP address of your computer configured in both the kmr_communications package and java application on the KMR.

To move the robot using the keyboard, run
```
$ ros2 run kmr_navigation2 twist_keyboard.py
```
Regardless of whether you simulate or run the real KMR, you should now see topics such as /scan and /scan_2 being active if you type `ros2 topic list` in the terminal.

To get the transformations between different joints of the robot, run
```
$ ros2 launch kmr_bringup state_publisher.launch.py
```
This is required in order to get kmr_concatentator to work. kmr_concatenator takes input from the two laser scanners on the robot and concatenates them into a single LaserScan message expressed in a single frame. The transformation from laser scanner frames to a central frame is therefore required.

It is important that you set simulated to either 'true' or 'false' in the concatenator.launch.py file. By default, it is 'false'.

To concatenate the laser scan messages, run
```
$ ros2 launch kmr_concatenator concatenator.launch.py
```
By again running `ros2 topic list`, you should now see /pc_concatenated and /scan_concatenated. /pc_concatenated is created by the concatenator_node, and /scan_concatenated is created by the pointcloud_to_laserscan_node using the concatenated point cloud in /pc_concatenated. 

To create maps using SLAM and the simulated KMR, run
```
$ ros2 launch kmr_slam_toolbox gazebo_online_async_launch.py
```
If you are using the real KMR, run
```
$ ros2 launch kmr_slam_toolbox KMR_online_async_launch.launch.py
```

To visualize the map you are creating, we need to launch RViz. This can be done by running
```
$ ros2 launch kmr_bringup rviz.launch.py
```
You should now be able to see the map appear as you move the robot.

To navigate the robot around the map you just made, simply add the saved map to the launch file of kmr_navigation2. 
Connect the robot using 
```
$ ros2 launch kmr_communications sunrise_communication.launch.py
```
Start the laser concatenator by running
```
$ ros2 launch kmr_bringup state_publisher.launch.py
$ ros2 launch kmr_concatenator concatenator.launch.py
```
Then start Navigation2 by running
```
$ ros2 launch kmr_navigation2 navigation2.launch.py
```
Now estimate the initial position of the robot by using the "2D pose estimate" button. Once that is done, either set a goal by using the "2D Goal point" button, or make a waypoint route by using the RViz menu for Navigation2. The robot should now be moving on its own! An example of path planning can be seen in the Example section.

## Example
By running communications and SLAM_Toolbox, I was able to map out parts of the MANULAB at NTNU in Trondheim, Norway.
<p align="center">
  <img src="https://raw.githubusercontent.com/MortenMDahl/kmriiwa_ws_devel/foxy/images/MANULAB.PNG"/>
</p>

This map can be used to navigate the robot using Navigation2.
<p align="center">
  <img src="https://raw.githubusercontent.com/MortenMDahl/kmriiwa_ws_devel/foxy/images/plannedpath.png"/>
</p>
