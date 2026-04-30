#!/bin/bash

source /opt/ros/humble/setup.bash
source /etc/turtlebot4/setup.bash
source install/setup.bash

# This script starts the agent node with the specified robot number

# Check if a robot number was provided
if [ -z "$1" ]; then
  echo "Error: Please provide a robot number."
  exit 1
fi

ROBOT_NUM=$1
shift
# other_robots="$@"
other_robots="[$(echo "$@" | sed 's/ /,/g')]"

# ##New Robot
# # Calibration 
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control calibration.py --ros-args -p robot.id:=$ROBOT_NUM &> ./log.txt'"

# # Adversary
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control doNothing.py --ros-args --params-file src/agent_control/config/CPIH/Network1.yaml -p robot.id:=$ROBOT_NUM -p robot.neighbors:=$other_robots -p logging.enabled:=true&> ./log.txt'"

#CPIH Experiements
# Exp 1: 9 robot run 
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control CPIH.py --ros-args --params-file src/agent_control/config/CPIH/Network1.yaml -p robot.id:=$ROBOT_NUM -p robot.neighbors:=$other_robots -p logging.enabled:=true&> ./log.txt'"


# Vision Follow
tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control aprilTagDetection.py --ros-args -p robot.id:=$ROBOT_NUM -p tag.follow:=true -p led.override:=true -p sensors.use_mocap:=False &> ./log.txt'"

# # Concesus
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control consensus.py --ros-args -p robot.id:=$ROBOT_NUM -p robot.neighbors:=$other_robots > ./log.txt'"
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control consensus.py --ros-args -p robot.id:=$ROBOT_NUM -p robot.neighbors:=$other_robots -p logging.enabled:=true&> ./log.txt'"

## FollowMe
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control followMe_triangle.py --ros-args -p robot.id:=$ROBOT_NUM -p robot.neighbors:=$other_robots &> ./log.txt'"

## LF_Furmation
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control LF_formation.py -f ~/Turtlebot_Controller/src/agent_control/config/5_agent_formation/pentagon_setup.yaml --ros-args -p robot.id:=$ROBOT_NUM -p robot.neighbors:=$other_robots &> ./log.txt'"
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control LF_formation.py -f ~/Turtlebot_Controller/src/agent_control/config/5_agent_formation/pentagon_setup.yaml --ros-args -p robot.id:=$ROBOT_NUM -p robot.neighbors:=$other_robots -p logging.enabled:=true&> ./log.txt'"

## formation control command
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control LF_multi_formation.py --ros-args -p robot.id:=$ROBOT_NUM -p robot.neighbors:=$other_robots &> ./log.txt'"

## Coverage
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control coverage.py -b -3.8 0.19 -0.72 3.36 2.69 2.0 2.74 -2.6 -0.18 -5.0 -2.48 -5.21 --ros-args -p robot.id:=$ROBOT_NUM -p robot.neighbors:=$other_robots&> ./log.txt'"
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control coverage.py -b -3.8 0.19 -0.72 3.36 2.69 2.0 2.74 -2.6 -0.18 -5.0 -2.48 -5.21 --ros-args -p robot.id:=$ROBOT_NUM -p robot.neighbors:=$other_robots -p logging.enabled:=true&> ./log.txt'"


# # Vision Concesus
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control aprilTagDetection.py --ros-args -p robot.id:=$ROBOT_NUM -p logging.enabled:=true &> ./log.txt'"

# testme 
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control testme.py --ros-args -p robot.id:=$ROBOT_NUM &> ./log.txt'"
# Calibration 
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control calibration.py --ros-args -p robot.id:=$ROBOT_NUM &> ./log.txt'"


## Old Node not used
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control LF_formation.py -f ~/Turtlebot_Controller/src/agent_control/config/agent_setup/agent_setup.yaml --ros-args -p robot.id:=$ROBOT_NUM &> ./log.txt'"
