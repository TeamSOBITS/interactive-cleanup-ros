# ROS2 Package for Interactive Cleanup

This project is ROS2 package for the Interactive Cleanup task of the RoboCup@Home Simulation.

See also [wiki page](https://github.com/RoboCupatHomeSim/interactive-cleanup-ros/wiki).


## Prerequisites

Same as below for OS and ROS2 version.  
https://github.com/RoboCupatHomeSim/documents/blob/master/SoftwareManual/Environment.md#ubuntu-pc

## How to Install

### Install Rosbridge Server

Please see below.  
http://wiki.ros.org/rosbridge_suite

### Install SIGVerse Rosbridge Server

Please see below.  
https://github.com/SIGVerse/ros_package/tree/humble-devel/sigverse_ros_bridge

### Install ROS Package of Interactive Cleanup

```bash:
$ cd ~/colcon_ws/src
$ git clone -b humble-devel https://github.com/RoboCupatHomeSim/interactive-cleanup-ros.git
$ cd ..
$ colcon build
```

## How to Execute

### How to Execute Sample ROS2 Node

It is a simple ROS2 node that communicates with the Interactive Cleanup application.

```bash:
$ ros2 launch interactive_cleanup_ros sample.launch.py
```

### How to Execute Teleoperation Tool

You can operate HSR with keyboard operation.  
It is for debugging.

```bash:
$ ros2 launch interactive_cleanup_ros teleop_key.launch.py
```

## License

This project is licensed under the SIGVerse License - see the LICENSE.txt file for details.
