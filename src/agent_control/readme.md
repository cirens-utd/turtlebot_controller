# Agent Controller Overview

** Need to scrub through to make sure document is updated with current changes **

## Overview
The goal of this package is to give you an Agent class that handles are the tedious back end information for navigating the turtlebots. For example, the robots can only move foward and backwards and turn on the Z axis. However, most agent controllers will give a desired direction to go in. 

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

## Agent initialization
When starting your agent, you are required to pass in:
- my_number (ex: 1)
- my_neighbors (ex: [2,3,4]. Default: [])

Additional options:
- sim (default: False)                  # Used when running in Simulator to setup message type
- sync_move (default: False)            # Not implemented yet. Will make all the nodes move in synce with each other
- desination_tolerance (default: 0.01)  # How far away from the target before we are considered to be at goal
- angle_tolerance (default: 0.1)        # How far away from the angle we can be and still considered at target
- laser_avoid (default: True)           # Use lidar to avoid obsticles 
- laser_distance (default: 0.5)         # How close the lidar can see an item before it stops and avoids it
- laser_delay (default: 5)              # When avoiding, the robot will delay this amount of seconds (not, when avoiding a neighbor, the higher number robot will wait 2x this number)
- laswer_walk_around (default: 2)       # Choose which way to walk around (0: Left, 1: Right, 2: which ever way is shorter)
- laser_avoid_loop_max (default: 1)     # How many "loops" the robot will do before it errors. This number can be increased to go around more complicated items
- neighbor_avoid (default: True)        # Will not allow the robots to run into its neighbor. (Laser avoidance normally will kick in first, but sometimes the laser will not pick up the neighbor)
- neighbor_delay (default: 5)           # Amount of time the robot will wait before it avoids. The higher number robot will wait 2x this

## Agent Sequance
### neighbor_pose_callback
This is called every time we get an updated position from our neighbors. It will update the agent attribute self._neighbor_position[2]. This has an array (x,y) for that neighbors position

### pose_callback
This is called every time my own position is updated. This will set the attributes self.direction_facing (Gives the angle the robot is facing) and self.position (Gives an array of (x, y) position). 

### lidar_callback
Lidar is used to avoid obsitcals by default. If you pass in laser_avoid = False, this will be disabled.
This minimum distance can be set by passing in laser_distance

### controller_loop
This is a shell function that is called every 0.1 seconds. It will first determin if the robot is ready to move by making sure all the valid topics are being received. After this, it will then start calling the controller function

### controller
This is the function that should be over written. This should hold the code for controlling the agent. 
 Needed info from agent.
        self.position                   This agents position
        self.neighbor_position          Dictionary of neighbors position
        self.move_direction([x,y])      Function to move in a direction
        self.move_to_position([x,y])    Function to move to a position

## Agent Property
- self.robot_ready
    Returns the status of the robot being ready
- self.position
    Returns the self._position (The (x, y) of the robot)
    Set with self.position([x,y])
- self.rate
    Returns the position refresh rate
    (Not used)
- self.direction_facing
    Returns the angle the robot is facing
    Set with self.direction_facing(q)
- self.desired_location
    The current destination that the robot is attempting to go to
    This is not updated when robot is attempting to avoid an obsticle
- self.motion_complete
    Returns when agent has reached its goal
    Set with self.motion_complete(True)
- self.path_obstructed
    This has no setter. This is if any of the obstruction vars are set. Inside the setter of the obsturuction vars, you should call "set_path_obstructed_"
    ** Note: if adding obsturction vars, you need to modify this method
- self.path_obstructed_laser
    Returns if the path is obsturcted (according to the laser)
    Set this var if you find that the laser says the path is not available
- self.path_obstructed_neighbor
    Returns when a neighbor is blocking the robots path
- self.laser_avoid_error
    This will return true if the robot reaches the max loops when avoiding an obsticle.

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
- diff_angles
    Finds the smalles angle between two angles.
    :param x: a radian between 0 - 2pi
    :param y: a radian between 0 - 2pi
    :return: radians
- move_direction
    pass in magnitude for both x and y. This function will add the direction to our current position and pass that to move to position
- move_to_position
    Start moving the robot to the desired direction. This handles when to drive vs when to turn. Also handles setting speeds for each
    :param desired_location: a position vector [x, y] you want to go to
    :return: None
- move_robot_
    Internal Method used to send ROS topic. Not intended to for outside use. But is the main controller for sending direct commands to the robot
    :param x: The desired value to put into linear X
    :param z: The desired value to put into angluar Z
    :return: None
- new_controller
    This function is used to reset the robot back to default and ready to run a new controller
- controller
    This is the main logic for controlling the agent. 

    This should be overridden by the derived classes, and the default
    raises NotImplementedError.

    You should find your desired position you want this agent to go to and then send that to the self.move_to_position method

    :raises: NotImplementedError
## Agent Arguments
- -i --index (Default: 1) # Index of this robot
- -n --neighbor (Default: []) # Array of neighbors
- -s --sim (Default: False) # Running in simmulation mode
- -l --laser_avoid (Default: True) # Avoid obsticles using laser
- -m --loop_max (Default: 1) # number of loops the laser will make before error
- -b --neighbor_avoid (Default: True) # Avoid neighbors using position

# Exmples:
Current Examples include:
consensus and LF_Formation

## Consensus
### Args
- -i --index (Default: 1) # Index of this robot
- -n --neighbor (Default: []) # Array of neighbors
- -s --sim (Default: False) # Running in simmulation mode
- -l --laser_avoid (Default: True) # Avoid obsticles using laser
- -m --loop_max (Default: 1) # number of loops the laser will make before error
- -b --neighbor_avoid (Default: True) # Avoid neighbors using position
### Launch file
ros2 launch agent_control consensus_batch.launch.py
- default is to load from config/consensus_config.yaml (only way to specify robot numbers)
Other args
- number_robots (Default: 10) # Number of robots you are controlling (sequential numbering)
- yaml_load (Default: True) # Used to load yaml
## LF_formation
### Args
- -i --index (Default: 1) # Index of this robot
- -s --sim (Default: False) # Running in simmulation mode
- -l --laser_avoid (Default: True) # Avoid obsticles using laser
- -m --loop_max (Default: 1) # number of loops the laser will make before error
- -b --neighbor_avoid (Default: True) # Avoid neighbors using position
- -f --formation (Deafult: Not valid path) # path to formation yaml. Example in src/agent_control/config/agent_setup/agent_setup.yaml

# Special Notes
This is a cpp package that also includes python. This has an added step of adding the python packages in the CMakeLists.txt file. 