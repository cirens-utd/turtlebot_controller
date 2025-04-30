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

## SSH Setup
### Generate Key
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"

### Copy Key
ssh-copy-id user@remote_host



# Commands to run

The general command is:

`./remote_launch <operation> <robot_numbers_separated_by_commas>`

`<operation>` includes `copy`, `build`, `run` and `stop`. To run consensus or formation control, uncomment the corresponding command in `start_node.sh`; make sure to comment the other command. Following that, execute the `copy`, `build` and `run` operations for the desired robots. For formation control, make sure that the `.yaml` file used in `start_node.sh` has the correct robot numbers and the correct formation matrix. If the code does not work on some robot, `ssh` into the robot, `cd ~/Turtlebot_Controller`, `source install/local_setup.bash` and `ros2 run agent_control <code_file>` to observe the error.
