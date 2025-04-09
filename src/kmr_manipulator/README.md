## 1. Description

This package handles tasks associated with manipulation of the LBR iiwa, updated for ROS 2 Humble compatibility.

- Vision using a Intel® RealSense™ D435 camera
- Object detection and localization
- Grasping using a Robotiq 2F-85 gripping

## 2. Requirements
The following packages needs to be installed:
- ROS 2 Humble
- ROS 2 Intel Realsense (humble branch)
- ROS 2 Openvino Toolkit (humble-compatible version)
- ROS 2 Object Analytics (see UPGRADE_INSTRUCTIONS.md for setup)

## 3. Run
The camera and gripper must be connected to a computer by USB. An onboard computer with ROS 2 Humble installed is useful for this purpose. 
The nodes are launched by running the command:

```
$ ros2 launch kmr_maniupulator nuc.launch.py 
```

