#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3
robot_num=$4
robot_name="Robot${robot_num}_"

echo "Coping Replays From $pi_ip..."
scp -r ${USER}@${pi_ip}:${pi_wrk_space}/Replays ./

echo "Coping Log File from $pi_ip..."
scp ${USER}@${pi_ip}:${pi_wrk_space}/log.txt ./${robot_name}log.txt