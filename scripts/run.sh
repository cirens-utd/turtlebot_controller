#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3
robot_num=$4
echo "$@"
shift
shift
shift
shift

echo "Running ROS2 nodes at $pi_ip..."
ssh ${USER}@${pi_ip} << EOF

    source /opt/ros/humble/setup.bash
    source /etc/turtlebot4/setup.bash

    cd $pi_wrk_space

    # Check to see how many nodes are running on the turtle bot.
    # If less then 10 nodes running, we need to restart the turtlebot service

    if [ \$(ros2 node list | wc -l) -lt 12 ]; then
        echo "****Restarting Services!!!!******"
        sudo systemctl restart turtlebot4.service
        sleep 3
    fi

    # Launching Mocab
    ./start_mocap.sh

    # Start Node
    ./start_node.sh $robot_num $@
    

    #Note: Reattach screen: screen -r ros_session1
    #Note: Reattach tmux: tmux attach-session -t ros_session1
EOF
