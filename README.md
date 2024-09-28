# Object Chasing Robot

This project is a part of the larger Autonomous Vehicle initiative.

## Overview

This ROS2 package enables a robot to chase a desired object observed in its local coordinate frame. The robot always faces the object and maintains a desired distance from it. The system combines camera and LIDAR data for accurate object tracking and uses PID control for smooth movement.

## Dependencies
- ROS2 Humble
- sensor_msgs
- std_msgs
- geometry_msgs
- OpenCV (for image processing)
- NumPy (for numerical computations)

## Features
- Object detection using Raspberry Pi camera
- Distance measurement using LIDAR
- PID control for smooth object tracking
- Fusion of camera and LIDAR data for accurate object localization

## Nodes
**1. detect_object**

This node subscribes to the Raspberry Pi Camera topic and processes the images to identify the location of the tracked object. It publishes the object's location in degrees.

**2. get_object_range**

This node subscribes to the object location from detect_object and the LIDAR data from the `/scan` topic. It combines this information to publish the angular position and distance of the object.

**3. chase_object**

This node implements two PID controllers (angular and linear) to make the robot face the object and maintain a desired distance. It subscribes to the data from get_object_range and publishes velocity commands for the robot.

## Usage
1. Ensure all dependencies are installed.
2. Clone this package into your ROS2 workspace.
3. Build the package using `colcon build`.
4. Source your workspace.
5. Run the nodes using:
```
ros2 run bb8_chase_object detect_object
ros2 run bb8_chase_object get_object_range
ros2 run bb8_chase_object chase_object
```

## Sensor Setup

Ensure that the Raspberry Pi camera and LIDAR are properly connected and publishing data. You may need to run additional nodes or adjust parameters for these sensors.

## Tips
- Coordinate your camera and LIDAR data in the same reference frame and units.
- Consider the sampling time of your system when implementing PID control.
- Adjust PID gains for optimal performance.
- Use visualization tools like RViz for debugging and monitoring.

## Troubleshooting
- Verify that all sensors are publishing data correctly.
- Check that the object detection algorithm is working as expected.
- Ensure that the robot's motor controllers are responding to velocity commands.
- Use ROS2 tools like `ros2 topic echo` and `rqt_graph` for debugging.

## Reference

[LIDAR](http://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/)
