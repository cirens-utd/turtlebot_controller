#!/bin/bash

USER="ubuntu"
pi_wrk_space="~/Turtlebot_Controller"

# Check if we have arguments (robot numbers)
if [ $# -lt 2 ]; then
  echo "Usage: $0 <operation> <robot_number1> <robot_number2> ..."
  echo "Operations: build, copy, run, stop"
  echo "Example: $0 copy 1 3 5"
  exit 1
fi

operation="$1"

# remove operation from arguments
shift

# loop through the robot numbers
for robot_num in "$@"; do
    # IP Address
    pi_ip="192.168.0.$((220 + robot_num))"

    echo "Performing '$operation' on robot$robot_num at IP $pi_ip..."

    case "$operation" in
        "copy")
        echo "Coping directory to $pi_ip..."
        scp -r ./src ./start_node.sh ${USER}@${pi_ip}:${pi_wrk_space}
        ;;

    "build")
        echo "Building workspace at $pi_ip..."
        ssh ${USER}@${pi_ip} << EOF
            cd $pi_wrk_space
            if [ ! -d build ]; then
                echo "Building needed..."
                colcon build
                #colcon build --symlink-install
            else
                echo "Workspace already built."
            fi
            chmod +x start_node.sh
EOF
        ;;

    "run")
        echo "Running ROS2 nodes at $pi_ip..."
        ssh ${USER}@${pi_ip} << EOF
            # Start tmux session for the first terminal
            #tmux new-session -d -s ros_sessions1 "bash -c 'ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.131 port:=3883 update_freq:=180 >> $pi_wrk_space/mocab_log.txt 2>&1'"
            screen -dmS ros_sessions1 bash -c "ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.131 port:=3883 update_freq:=180 >> $pi_wrk_space/mocab_log.txt 2>&1"


            # Split window and run commands for second terminal
            #tmux new-session -d -s ros_sessions2 "bash -c 'cd $pi_wrk_space && if [ ! -d build ]; then colcon build; fi && source install/setup.bash && ./start_node.sh $robot_num \"@\" >> $pi_wrk_space/log.txt 2>&1'"
            screen -dmS ros_sessions2 bash -c "cd $pi_wrk_space && if [ ! -d build ]; then colcon build; fi && source install/setup.bash && ./start_node.sh $robot_num \"@\" >> $pi_wrk_space/log.txt 2>&1"

            #Note: Reattach screen: screen -r ros_sessions1
EOF
        ;;

    "stop")
        echo "Killing ROS nodes remotely on $pi_ip..."
        ssh ${USER}@${pi_ip} << EOF
            # Get the list of active ROS nodes

            ## I may not need this. Killing the session may do it for me
            #./end_node.sh

            # Kill tmux sessions
            #tmux kill-session -t ros_sessions1
            #tmux kill-session -t ros_sessions2
            screen -X -S ros_sessions1 quit
            screen -X -S ros_sessions2 quit


            echo "Nodes have been killed on $pi_ip."
EOF
      ;;
      
    *)
        # Handle invalid operation input
        echo "Invalid operation: $operation"
        echo "Valid operations are: build, copy, run, stop"
        exit 1
        ;;
    esac

    echo "Operation '$operation' completed for robot$robot_num at $pi_ip."
done

echo "All operations complete"