#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3

echo "Coping Replays From $pi_ip..."
scp -r ${USER}@${pi_ip}:${pi_wrk_space}/Replays ./