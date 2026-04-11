
USER=$1
pi_ip=$2
pi_wrk_space=$3
robot_num=$4
shift
shift
shift
shift
other_robots="[$(echo "$@" | sed 's/ /,/g')]"

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
    
    ros2 run agent_control start_point.py --ros-args --params-file src/agent_control/config/CPIH/Network1.yaml -p robot.id:=$ROBOT_NUM -p robot.neighbors:=$other_robots
    # QE Start Points -s -2.7 -2.9 2.1 -3.7 1.7 -2.9 1.4 -3.7 -3.3 -3.4 -2.2 -3.4 -1 3.2 -.17 5 .91 2.9
    
EOF