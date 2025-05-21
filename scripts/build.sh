#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3

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
