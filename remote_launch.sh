#!/bin/bash

USER="ubuntu"
pi_wrk_space="~/Turtlebot_Controller"

# Check if we have arguments (robot numbers)
if [ $# -lt 2 ]; then
  echo "Usage: $0 <operation> <robot_number1> <robot_number2> ..."
  echo "Operations: build, copy, run, stop, test"
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
        scp -r ./src ./start_node.sh ./start_mocap.sh ./test_node.sh ${USER}@${pi_ip}:${pi_wrk_space}
        
        ;;

    "sync")
        echo "Syncing up files to $pi_ip..."
        rsync -avz --delete ./src ./start_node.sh ./start_mocap.sh ./test_node.sh ${USER}@${pi_ip}:${pi_wrk_space}
        ;;

    "build")
        echo "Building workspace at $pi_ip..."
        ssh ${USER}@${pi_ip} << EOF

            source /opt/ros/humble/setup.bash
            source /etc/turtlebot4/setup.bash

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
            
            echo "Building..."
            colcon build --symlink-install
            chmod +x start_node.sh start_mocap.sh test_node.sh
EOF
        ;;
    "deepBuild")
        echo "Building workspace at $pi_ip..."
        ssh ${USER}@${pi_ip} << EOF

            source /opt/ros/humble/setup.bash
            source /etc/turtlebot4/setup.bash

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
            
            echo "Building..."
            colcon build
            chmod +x start_node.sh start_mocap.sh test_node.sh
EOF
        ;;

    "run")
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
        ;;

    "stop")
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
        ;;
        
    "test")
        echo "Testing robots will move"
        ssh ${USER}@${pi_ip} << EOF

            cd $pi_wrk_space
            ./test_node.sh $robot_num
EOF
        ;;
    *)
        # Handle invalid operation input
        echo "Invalid operation: $operation"
        echo "Valid operations are: build, copy, run, stop, test"
        exit 1
        ;;
    esac

    echo "Operation '$operation' completed for robot$robot_num at $pi_ip."
done

echo "All operations complete"
