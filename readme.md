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