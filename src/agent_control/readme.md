# Agent Controller Overview

## Overview
The goal of this package is to give you an Agent class that handles are the tedious back end information for navigating the turtlebots. For example, the robots can only move foward and backwards and turn on the Z axis. However, most agent controllers will give a desired direction to go in. 

## Release Updates:
V0.0.1 - 5/23/25
- Added monitoring to see if neighbors are completed
- Added boolean to decide if new controller starts with start position or just start with the current position
- Updated variable names start_heading, end_heading to not have the internal reference
- Updated direction_facing to be direction_heading
- Added a variable driving_heading_tolerance
- Added Logging
- Added shutdown method to be called before shutting down rclp
- FIXED: Angle control more consistant to all angles
 - Found error in setters for desired angle and direction heading
- Added Replay veiwer

## Starting Controler
The robot will wait for the required topics to be present before it can start moving. For example, if you are  using lidar detection, the lidar topic needs to be posting. Additionally, all the neighbors you have in your list need to have their positions positing before it will start.

After the robot is ready to move, the self.robot_ready will go true. The robot will turn to have a heading of 0. (coded in the init method. self.start_heading) After the robot sees all its neighbors have their heading of 0 as well, the self.robot_moving will turn true and the controller will start working

After the controller is completed, the robot will do the end_controller method. This can be chained to allow mutliple controller sequances or visual keys to let users know it is done. By default, this will turn to a heading of pi. (coded in the init method. self.end_heading)

## Ending Script
If you are using the logging feature, be sure to have your code end cleanly so you can call the shutdown function from the agent Node. This will package up the replay file and zip it.

## Multi Controller System
After the controller is complete, the destination_reached will be True. This will then call the end_controller function to do any post-controller processing. The default is to turn to the end_heading position. The end_controller function should then set motion_complete to True. This will then trigger the checking of the neighbors position through the function check_neighbors_finished. The Default for this is to check if all neighbors are in the end_heading direction. This function will then set neighbors_complete to True. At this point, you are able to issue the new_controller function to reset the state of the agent and launch a new controller. (Note: you can set restart_start_position to False if you don't want to run the robot ready verifications)

Additional Comments: you can modify any of these functions as you do with controller. Just keep in mind, that the end_controller and check_neighbors_finished are working off the same assumption to determine if the agents are completed. These both should be modified if one is modified.

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
- logging (default: True)
    Saves logging information to a file. 
- desination_tolerance (default: 0.01)  # How far away from the target before we are considered to be at goal
- angle_tolerance (default: 0.1)        # How far away from the angle we can be and still considered at target
- at_goal_historisis                    # After reaching goal, how far the new goal needs to go before starting back up 
- restricted_area                       # Boolean to restrict where the robot can move into on a planed move
- restricted_x_min                      # The lowest x value that the robot will go to
- restricted_x_max                      # The highest x value that the robot will go to
- restricted_y_min                      # The lowest y value the robot will go to
- restricted_y_max                      # The highest y value the robot will go to
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
This is called every time my own position is updated. This will set the attributes self.direction_heading (Gives the angle the robot is facing) and self.position (Gives an array of (x, y) position). 

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
        self.neighbor_orientation       Dictionary of neighbors heading
        self.move_direction([x,y])      Function to move in a direction
        self.move_to_position([x,y])    Function to move to a position

### end_controller
This can be overriden to determin what the robot will do when the controller is completed.
By default, the robot will face to have a heading of Pi.

### check_neighbors_finished
This will go through and see if all the neighbors are finished. When they are, it will set self.neighbors_completed to True

### new_controller
This can be called to reset all the values back to their default and start a new controller. You can change restart_start_position to False if you don't want all robots to go through the ready sequance again

## Agent Property
- self.robot_ready
    Returns the status of the robot being ready
- self.robot_moving
    Returns the status of the robot is moving (aka, all neighbors and self are ready and contoller started)
- self.destination_reached
    Returns the status of the robot reaching its goal destination
- self.desired_heading
    Returns the status of heading control of the robot. When demanding the robot to turn to an angle, this will go true when it reaches that angle.
- self.position
    Returns the self._position (The (x, y) of the robot)
- self.rate
    Returns the position refresh rate
    (Not used)
- self.direction_heading
    Returns the angle the robot is facing
- self.desired_location
    The current destination that the robot is attempting to go to
    This is not updated when robot is attempting to avoid an obsticle
- self.desired_angle
    The current goal angle for the robot
- self.motion_complete
    Returns when agent has reached its goal
- self.neighbors_complete
    Returns when agents sees all their neighbors are in the end position
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
- restart_start_position
    This will flag if you want to return to start position before starting a new controller
- driving_heading_tolerance
    This controlls how close to a striaght line you need to be before you start moving forward
- logging_enable
    Boolean to know if the file needs to be zipped. By setting this, you will create and distroy the logging timer and zip up the file.
- logging_paused
    Boolean to pause the logging recording feature as long as it is set to true

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
- move_to_angle
    Change the robot to have a desired heading
    :param rad: a radian you want to go to [0, 2pi)
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
- end_controller
    This is a "Controller Complete Sequance"

    This can be overridden depending on what you want your default state to be at the end of your controller. The default is for the bot to turn to the pretermined angle and just wait

    When modifing this, after the robot reaches the desired position, it will call this mehod until motion_complete is set to true. If a new destination is called, this method will stop
    being called and the robot will move to the new positon

    example to change this method:
    if you want to flash LED's or Play a song on completetion
    if you have more advanced logic to prepare for the next controller to be called
- shutdown
    Adding function to clean up the enviorment. Currently only finishing the log file stuff

# Logging Information
** Add variables to be logged
** Can edit the _log_dict_length **
** Enable Variables? Neighbor avoid, lasar avoid, area_restricted, ect

## Agent Arguments
- -i --index (Default: 1) # Index of this robot
- -n --neighbor (Default: []) # Array of neighbors
- -s --sim (Default: False) # Running in simmulation mode
- -l --laser_avoid (Default: True) # Avoid obsticles using laser
- -m --loop_max (Default: 1) # number of loops the laser will make before error
- -b --neighbor_avoid (Default: True) # Avoid neighbors using position
- -f --formation # Path to yaml for formation control

# Exmples:
Current Examples include:
consensus, LF_Formation, followMe

## Consensus
### Args
- -i --index (Default: 1) # Index of this robot
- -n --neighbor (Default: []) # Array of neighbors
- -s --sim (Default: False) # Running in simmulation mode
- -l --laser_avoid (Default: True) # Avoid obsticles using laser
- -m --loop_max (Default: 1) # number of loops the laser will make before error
- -b --neighbor_avoid (Default: True) # Avoid neighbors using position
### Launch file
ros2 launch agent_control consensus_batch.launch.py sim_mode:=True
- default is to load from config/consensus_config.yaml (only way to specify robot numbers)
Other args
- number_robots (Default: 10) # Number of robots you are controlling (sequential numbering)
- yaml_load (Default: True) # Used to load yaml
- sim_mode (Default: False) # Used to run in simulation mode
## LF_formation
### Args
- -i --index (Default: 1) # Index of this robot
- -s --sim (Default: False) # Running in simmulation mode
- -l --laser_avoid (Default: True) # Avoid obsticles using laser
- -m --loop_max (Default: 1) # number of loops the laser will make before error
- -b --neighbor_avoid (Default: True) # Avoid neighbors using position
- -f --formation (Deafult: Not valid path) # path to formation yaml. Example in src/agent_control/config/agent_setup/agent_setup.yaml
### Launch File
ros2 launch agent_control lf_formation.launch.py sim_mode:=True
Args:
- sim_mode (Default: False) # Used to run in simulation mode
## FollowMe
### Args
- -i --index (Default: 1) # Index of this robot
- -s --sim (Default: False) # Running in simmulation mode
- -l --laser_avoid (Default: True) # Avoid obsticles using laser
- -m --loop_max (Default: 1) # number of loops the laser will make before error
- -b --neighbor_avoid (Default: True) # Avoid neighbors using position
- -f --formation (Deafult: Not valid path) # path to formation yaml. Example in src/agent_control/config/agent_setup/agent_setup.yaml
### Launch File,
No launch file yet. 
### TODO
This uses a 3 node formation control and will be used to see how a leader node effects the formation


# Special Notes
This is a cpp package that also includes python. This has an added step of adding the python packages in the CMakeLists.txt file. Also, the nodes will be called by the python script name and not the name seutp in the setup.py file
