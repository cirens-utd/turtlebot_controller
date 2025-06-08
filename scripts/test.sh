#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3
robot_num=$4

echo "Testing robots will move"
ssh ${USER}@${pi_ip} << EOF

    cd $pi_wrk_space
    ./test_node.sh $robot_num
EOF
