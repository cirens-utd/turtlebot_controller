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

    sudo shutdown -r now
EOF
