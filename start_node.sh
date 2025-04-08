#!/bin/bash

# This script starts the agent node with the specified robot number

# Check if a robot number was provided
if [ -z "$1" ]; then
  echo "Error: Please provide a robot number."
  exit 1
fi

ROBOT_NUM=$1
shift
other_robots="$@"

# Run the ROS 2 agent with the given robot number
ros2 run agent_control consensus.py -i $ROBOT_NUM -n $other_robots
