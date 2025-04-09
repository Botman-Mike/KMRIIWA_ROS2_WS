
# Message Definitions for Humble Compatibility

This folder contains example message definitions for the custom message packages needed for ROS 2 Humble compatibility.

## object_msgs

### ObjectInBox.msg
```
# ObjectInBox.msg

# Header for timestamp and frame information
std_msgs/Header header

# Bounding box of the detected object
sensor_msgs/RegionOfInterest roi

# Class label of the detected object
string class_label

# Confidence score of the detection
float32 confidence
```

## object_analytics_msgs

### ObjectsInBoxes3D.msg
```
# ObjectsInBoxes3D.msg

# Header for timestamp and frame information
std_msgs/Header header

# List of detected objects in 3D bounding boxes
ObjectInBox3D[] objects
```

### ObjectInBox3D.msg
```
# ObjectInBox3D.msg

# Header for timestamp and frame information
std_msgs/Header header

# 3D bounding box of the detected object
geometry_msgs/Point[] bounding_box

# Class label of the detected object
string class_label

# Confidence score of the detection
float32 confidence
```

## pipeline_srv_msgs

### PipelineRequest.srv
```
# PipelineRequest.srv

# Request part of the service
string request_data

---
# Response part of the service
string response_data