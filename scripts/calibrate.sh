
USER=$1
pi_ip=$2
pi_wrk_space=$3
robot_num=$4

ssh ${USER}@${pi_ip} << EOF

    cd $pi_wrk_space
    source /opt/ros/humble/setup.bash
    source /etc/turtlebot4/setup.bash

    # Check to see how many nodes are running on the turtle bot.
    # If less then 10 nodes running, we need to restart the turtlebot service

    if [ \$(ros2 node list | wc -l) -lt 12 ]; then
        echo "****Restarting Services!!!!******"
        sudo systemctl restart turtlebot4.service
        sleep 3
    fi

    # Launching Mocab
    ./start_mocap.sh

    source install/setup.bash 
    
    ros2 run agent_control calibration.py -i $robot_num
EOF