#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3

echo "Killing ROS nodes remotely on $pi_ip..."
ssh ${USER}@${pi_ip} << EOF
    # Get the list of active ROS nodes

    # Kill screen sessions
    #screen -X -S ros_session1 quit
    #screen -X -S ros_session2 quit
    #screen -X -S .ubuntu quit
    tmux kill-session -t ros_session1
    tmux kill-session -t ros_session2


    echo "Nodes have been killed on $pi_ip."
EOF
