## 1 - Description:
This is a package made to concatenate the two laser scanner sensors on the KUKA KMR iiwa. This is done due to SLAM_Toolbox only taking a single laser scan topic as input. 

Two LaserScan messages are subscribed to and filtered by the time between messages. If the time is below the set requirement, a callback function converts them to PointCloud2. The two PointCloud2 messages are unpacked and transformed into a common frame. They are then added together into a third PointCloud2 message and published. This message is then subscribed to by a 'pointcloud_to_laserscan_node' from the 'pointcloud_to_laserscan' package, and returned as a LaserScan. This LaserScan now consist of data from both laser scanners taken at the approximate same time.

Parts of the code which involves unpacking the points is based on the deprecated 'laser_geometry' package.

## 2 - Requirements:
- ROS 2 Humble
- pointcloud_to_laserscan package.
- Communication with the KMR or launching gazebo.

## 3 - Run:
To launch the concatenator, run:
```
$ ros2 launch kmr_concatenator concatenator.launch.py
```

You should now see the topics /pc_concatenated and /scan_concatenated being active.

## 4 - Notes:
The frequency and quality of concatenated laser scans highly depend on the allowed time between messages. This is changed in the ApproximatedTimeSynchronizer() function inside scripts/concatenator_node.py file. 
Lowering the time requirement between messages increases the accuracy of scans, but decreases the frequency of publishing. Vice versa if it's increased.

To change the topic names, see the launch file for /scan_concatenated and the concatenator_node.py file in scripts for /pc_concatenated.
