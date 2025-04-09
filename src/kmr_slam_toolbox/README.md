## 1 - Description:
SLAM_Toolbox is the built-in SLAM software that comes with ROS2. It works by taking in laser scan readings and converting them to a map of the surrounding area. From this map, the position and odometry of the robot can be taken. 

Settings can be adjusted by changing the files in the config folder. These files are referenced in the launch file, so if you create additional config files, remember to reference them.

SLAM_Toolbox only takes in a single laser scan topic. Due to this, the two laser scanners on the KMR has to be concatenated in order to utilize them both. This is done by kmr_concatenator, so remember to launch it before expecting maps to be created. If you only want to use a single laser, change the input laser in the configuration to be on the /scan topic.

## 2 - Requirements:
- Navigation2
- SLAM_Toolbox (should be included with Navigation2)

- Be connected to the KMR using kmr_communications or launch gazebo.
	- Having the nodes on the KMR launched and communicating with the external computer

## 3 - Run:

** To use SLAM_Toolbox with the KMR, you should already have launched kmr_concatenator. **

To launch SLAM_Toolbox for mapping, run:

```
$ ros2 launch kmr_slam_toolbox KMR_online_async_launch.launch.py
```

To save the map you have created, run:
```
$ ros2 run nav2_map_server map_saver_cli -f 'map_name'
```
The map is saved to the current terminal directory.

To launch for localization after a map is created, run:

```
$ ros2 launch kmr_slam_toolbox KMR_localization_online_async_launch.launch.py
```
Remember to change the map name in the config file.

To view the map while its being traversed or created, launch rviz:

```
$ ros2 launch kmr_bringup rviz2.launch.py
```
If the map disappears from Rviz, simply do a restart. The map should still be there, as it comes from kmr_slam_toolbox.


## 4 - Notes:
The Rviz plugin for SLAM_Toolbox has currently not been ported from ROS1:
https://github.com/SteveMacenski/slam_toolbox/issues/177
