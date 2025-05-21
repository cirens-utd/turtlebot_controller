#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3

echo "Copying directory to $pi_ip..."
scp -r ./src ./start_node.sh ./start_mocap.sh ./test_node.sh ${USER}@${pi_ip}:${pi_wrk_space}
