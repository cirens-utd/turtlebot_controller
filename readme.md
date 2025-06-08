# Turtlebot Controller Nodes

# Overview
These packages allow for the simulation and control of Turtlebots. This requires ros_gz to be installed on the machine to use the ros_pose_scrapper function for getting position. Each node has its own readme to describe each on specifically.

# Nodes
## ros_pose_scraper
Takes the encoded PoseArray from cirens_ros_bridge and creates each robots mocap topic for simulation.

## turtlebot_base
Backbone for the simulation of the turtlebots. Need to use the two launch scripts (launch_sim.launch.py and launch_robots.launch.py) to get start up simulation.

## agent_control
This gives an easy way to control the turtle bot. It is able to do lidar obsticle avoidance, neighbor avoidance, moving to a desired position (x,y) and move in a desired direction. The goal is to make implementation of control systems easier by using this as a backbone. The readme for this node explains on the features that it is capable of. 

# Replayer
Have added a replay viewer that takes the turtleReplay files and will allow them to be played back for analyzing how the node control behaved. 

## Arguments
- -s: Will save the replay as an mp4 file. 
- -p: Will prevent the actual playback window from opening
- -f: Set the filename of the mp4 to be saved
- -b: Will save the reply as a beautified json for analytical analysis

## SSH Setup
### Generate Key
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"

### Copy Key
ssh-copy-id user@remote_host

# Scripts for launching

## Remote Launch Script
This script allows for operations to be preformed on each robot. These operations are copy, sync, build, deepBuild, run, stop, pullLogs, clearLogs, calibrate, testCalibrate, test

These operations are saved in the scripts directory and will launch in their own terminal for simultaneous launching. These are all ran from the host computer. But it does require the scripts in the main directory (start_mocap, start_node, and test_node) to be on the robot and executable
### Operations
- copy: will copy entire source directoroy and needs sh files from root using scp
- sync: Will do the sync function, not this will effect permisions
- build: Will do a symlink build on the turtle bot and set all needed files as executable
- deepBuild: Will do a normal colcon build and set all needed files as executable
- run: Will start mocap and the start.sh script
- stop: Will kill mocap and the node started in start.sh
- pullLogs: Will copy files from Replays in the code directory
- clearLogs: Will remove Replay files from robot
- calibrate: Will run calibration on the robot and save the file
- testCalibrate: Will test calibration on the robot and will not save any file
- test: Will turn the robots to make sure they all are responding

### Starting a node
This will require you to update teh start_node.sh file and move it to the robot. What ever is in that robots start_node.sh file is what will run

### Example
`./remote_launch <operation> <robot_numbers_separated_by_commas>`

`<operation>` includes `copy`, `sync`, `build`, `run` and `stop`. To run consensus or formation control, uncomment the corresponding command in `start_node.sh`; make sure to comment the other command. Following that, execute the `copy`, `build` and `run` operations for the desired robots. For formation control, make sure that the `.yaml` file used in `start_node.sh` has the correct robot numbers and the correct formation matrix. If the code does not work on some robot, `ssh` into the robot, `cd ~/Turtlebot_Controller` and check the log.txt file to see why it failed