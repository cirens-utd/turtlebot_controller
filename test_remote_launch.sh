#!/bin/bash

USER="stufmuff"
pi_wrk_space="~/Documents/TestRos"

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

# IP Address
pi_ip="10.0.0.204"
robot_num="$1"

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

        # Loop through all Python files
        for file in ./src/agent_control/agent_control/*.py; do
            # Check if the file exists (in case there are no .py files)
            if [ "\$file" != "./src/agent_control/agent_control/*.py" ]; then
                # Mark the file as executable
                chmod +x "\$file"
                echo "Marked \$file as Executable"
            else
                echo "No Python Files Found!"
            fi
        done
        
        if [ ! -d build ]; then
            echo "Building needed..."
            colcon build --symlink-install
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
        tmux new-session -d -s ros_session1 "bash -c 'source /opt/ros/humble/setup.bash && ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.131 port:=3883 update_freq:=180 >> $pi_wrk_space/mocab_log.txt 2>&1'"
        
        # Check if screen session is running
        if [ $? -ne 0 ]; then
            echo "Error starting session ros_session1"
        else
            echo "ros_session1 started successfully."
        fi

        # Split window and run commands for second terminal
        tmux new-session -d -s ros_session2 "bash -c 'source /opt/ros/humble/setup.bash && cd $pi_wrk_space && colcon build --symlink-install && source install/setup.bash && ./start_node.sh $robot_num $@ >> $pi_wrk_space/log.txt 2>&1'"
        
        # Check if screen session is running
        if [ $? -ne 0 ]; then
            echo "Error starting session ros_session2"
        else
            echo "ros_session2 started successfully."
        fi

        #Note: Reattach screen: screen -r ros_session1
        #Note: Reattach tmux: tmux attach-session -t ros_session1
EOF
    ;;

"stop")
    echo "Killing ROS nodes remotely on $pi_ip..."
    ssh ${USER}@${pi_ip} << EOF
        # Get the list of active ROS nodes

        ## I may not need this. Killing the session may do it for me
        #./end_node.sh

        # Kill tmux sessions
        tmux kill-session -t ros_session1
        tmux kill-session -t ros_session2


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


echo "All operations complete"