#!/bin/bash

source /opt/ros/humble/setup.bash
source /etc/turtlebot4/setup.bash

# This script starts the agent node with the specified robot number

# Check if a robot number was provided
if [ -z "$1" ]; then
  echo "Error: Please provide a robot number."
  exit 1
fi

ROBOT_NUM=$1

tmux new-session -d -s ros_session2 "bash -c 'ros2 topic pub -r 2 /robot$ROBOT_NUM/cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0, y: 0.0, z: 1.0}}\"'"