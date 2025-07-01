#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3

echo "Copying python files to $pi_ip..."
scp -r ./_python_modules ${USER}@${pi_ip}:${pi_wrk_space}


echo "Building Python packages at $pi_ip..."
ssh ${USER}@${pi_ip} << EOF

    source /opt/ros/humble/setup.bash
    source /etc/turtlebot4/setup.bash

    cd $pi_wrk_space/_python_modules

    python3 get-pip.py --no-index --find-links=.
EOF