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
other_robots="$@"

# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control consensus.py -i $ROBOT_NUM -n $other_robots -l &> ./log.txt'"
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control followMe_triangle.py -l -i $ROBOT_NUM -n $other_robots &> ./log.txt'"
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control LF_formation.py -l -i $ROBOT_NUM -f ~/Turtlebot_Controller/src/agent_control/config/agent_setup/nine_node_formation.yaml -n $other_robots &> ./log.txt'"

# formation control command
tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control LF_multi_formation.py -l -n $other_robots -i $ROBOT_NUM  &> ./log.txt'"



## Old Node not used
# tmux new-session -d -s ros_session2 "cd $(pwd) && source install/setup.bash && bash -c 'ros2 run agent_control LF_formation.py -l -i $ROBOT_NUM -f ~/Turtlebot_Controller/src/agent_control/config/agent_setup/agent_setup.yaml &> ./log.txt'"
