# Agent Controller Overview

## Overview
The goal of this package is to give you an Agent class that handles are the tedious back end information for navigating the turtlebots. For example, the robots can only move foward and backwards and turn on the Z axis. However, most agent controllers will give a desired position to go to. 

## Structure
When you create the agent, you will pass in the robot number. This number will be used to get all the topics listed below:
- /robot1/camera_info
    information about the camera
- /robot1/cmd_vel
    used to move the robot
- /robot1/oakd/rgb/preview/image_raw
    image topic information
    frame is base_link
- /robot1/odometry
    simulated odometry 
    This doesn't show up in RViz as a fixed frame though??
- /robot1/scan
    lidar scan 
- /robot1/scan/points
    lidar scan point
- /vrpn_mocap/turtlebot1/pose
    x,y,z and rotation cordinates from an "external vision system"

The second argument passed in is an array to give the numbers to the neighbors. This will be used to get the global position of the neighbors
- /vrpn_mocap/turtlebot2/pose

## Agent Sequance
### neighbor_pose_callback
This is called every time we get an updated position from our neighbors. It will update the agent attribute self._neighbor_position[2]. This has an array (x,y) for that neighbors position

### pose_callback
This is called every time my own position is updated. This will set the attributes self.direction_facing (Gives the angle the robot is facing) and self.position (Gives an array of (x, y) position). Then the self.controller method is called

### lidar_callback
Lidar is used to avoid obsitcals by default. If you pass in laser_avoid = False, this will be disabled.
This minimum distance can be set by passing in laser_distance

## Agent Property
- self.position
    Returns the self._position (The (x, y) of the robot)
    Set with self.position([x,y])
- self.rate
    Returns the position refresh rate
    (Not used)
- self.direction_facing
    Returns the angle the robot is facing
    Set with self.direction_facing(q)
- self.motion_complete
    Returns when agent has reached its goal
    Set with self.motion_complete(True)
- self.path_obstructed
    This has no setter. This is if any of the obstruction vars are set. Inside the setter of the obsturuction vars, you should call "set_path_obstructed_"
    ** Note: if adding obsturction vars, you need to modify this method
- self.path_obstructed_laser
    Returns if the path is obsturcted (according to the laser)
    Set this var if you find that the laser says the path is not available

## Agent Methods
- get_angle_quad
    Finds the heading angle of the robot from its quaternion orientation
    :param q: quaternion rotation from ROS msg
    :return: radians 
- get_angle
    Finds the angle between x and y
    :param x: an array of [x, y] position for x
    :param y: an array of [x, y] position for y
    :return: radians
- move_to_position
    Start moving the robot to the desired direction. This handles when to drive vs when to turn. Also handles setting speeds for each
    :param desired_location: a position vector [x, y] you want to go to
    :return: None
- move_robot_
    Internal Method used to send ROS topic
    :param x: The desired value to put into linear X
    :param z: The desired value to put into angluar Z
    :return: None
- controller
    This is the main logic for controlling the agent. 

    This should be overridden by the derived classes, and the default
    raises NotImplementedError.

    You should find your desired position you want this agent to go to and then send that to the self.move_to_position method

    :raises: NotImplementedError