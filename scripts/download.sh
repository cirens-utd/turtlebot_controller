#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3

echo "Copying python files to $pi_ip..."
rsync -avz --delete ./_python_modules ${USER}@${pi_ip}:${pi_wrk_space}
# scp -r ./_python_modules ${USER}@${pi_ip}:${pi_wrk_space}


echo "Building Python packages at $pi_ip..."
ssh ${USER}@${pi_ip} << EOF

    source /opt/ros/humble/setup.bash
    source /etc/turtlebot4/setup.bash

    cd $pi_wrk_space/_python_modules

    # This will go find all the .whl files and install them. It will also search inside other directores in this folder
    find . -name "*.whl" -type f -exec pip3 install --no-index --find-links=. {} \;

    # This will go find all the .tar.gz files and install them. It will also search inside other directores in this folder
    find . -name "*.tar.gz" -type f -exec pip3 install --no-index --find-links=. --no-build-isolation {} \;

EOF