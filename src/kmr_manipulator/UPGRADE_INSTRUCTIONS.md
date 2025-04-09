# ROS 2 Humble Upgrade Instructions

## Required Packages

The following packages need to be ported or installed for ROS 2 Humble compatibility:

1. **kmr_msgs** - Contains the `ObjectSearch` action
2. **pipeline_srv_msgs** - Contains the `PipelineSrv` service and `PipelineRequest` message
3. **object_analytics_msgs** - Contains `ObjectsInBoxes3D` and `ObjectInBox3D` messages
4. **object_msgs** - Contains `ObjectInBox` and `ObjectsInBoxes` messages

## Installation Steps

### Option 1: Build from Source

For each package that needs to be ported:

```bash
cd /home/ros2-control/kmriiwa_ws_devel/src
git clone [REPOSITORY_URL] [PACKAGE_NAME]
cd [PACKAGE_NAME]
# If needed, switch to a Humble-compatible branch
git checkout humble
```

### Option 2: Create Compatible Interfaces

If the original packages are not available for Humble, create compatible interfaces:

```bash
cd /home/ros2-control/kmriiwa_ws_devel/src
ros2 pkg create --build-type ament_cmake [PACKAGE_NAME]_msgs
```

Then define your message/service/action files in the `msg`, `srv`, and `action` folders.

### Build the Workspace

After setting up all required packages:

```bash
cd /home/ros2-control/kmriiwa_ws_devel
colcon build --symlink-install
source install/setup.bash
```

## Code Changes

The main code changes for ROS 2 Humble compatibility have been applied:

1. Updated action server initialization to include all required callbacks
2. Changed callbacks to be properly asynchronous
3. Improved error handling and executor usage
4. Updated service call pattern to use async/await correctly

These changes ensure compatibility with the ROS 2 Humble action server and executor model.

## Additional Implementation Details for Custom Message Packages

### Create the Missing Message Packages

For each custom message package:

```bash
cd /home/ros2-control/kmriiwa_ws_devel/src
ros2 pkg create --build-type ament_cmake [PACKAGE_NAME]_msgs
```

### Define Message/Service/Action Files

Create the necessary message, service, and action files in the `msg`, `srv`, and `action` folders of each package.

### Update CMakeLists.txt and package.xml

Ensure that the `CMakeLists.txt` and `package.xml` files are correctly set up to build the custom messages.

### Build the Workspace

After setting up all required packages:

```bash
cd /home/ros2-control/kmriiwa_ws_devel
colcon build --symlink-install
source install/setup.bash
```