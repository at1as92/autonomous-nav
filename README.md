# Autonomous Navigation of Robot Base

As part of NP Specialist Diploma in Robotics Engineering Autonomous Navigation module, I had to write a ROS programme to allow a robot to autonomously navigate in a designated indoor area. The tasks for this assignment are:

1. Creating a map of the indoor area using gmapping
2. The robot will have read the waypoints from file, navigate sequentially to the waypoints without stopping.
3. Additional marks awarded for automatic map changing at one of the waypoints, auto-start after power-on and having a programme to manage the waypoint saving process.

## Demonstration Video
[Youtube](https://youtu.be/pkNq5EGGhBY)

## Hardware and Software

* eaibot [SMART](https://edu.eaibot.com/products/view/16.html)
* ROS Noetic 

## Tasks for this Assignment

### Map Creation

Map of NP Blk 7 Level 1 indoor open space and corridor created using ROS gmapping package to generate a 2D map of approx. 53.4m by 11.4m. Cleaned up map PGM file generated using GIMP.

### Waypoint Saving

Performed by teleoperating or manually pushing robot to various waypoints and running waypoint_saver.py script in command line. Programme extracts robot's current pose information for saving as JSON.

### Navigation and Automatic Map Changing

Two classes are created within the navigation script: (1) CurrentPose and (2) MoveThruWaypoints
* CurrentPose: Subscribes to the robot pose information from the /amcl_pose topic
* MoveThruWaypoint:
  * Reads and decodes waypoints from JSON file
  * Publishes new waypoint to the /move_base_simple/goal topic when robot is less than a preset distance away
  * Serves new map through launch files if required

### Launch Files

nav.launch launches the required packages, sensors, maps and files for navigation.
