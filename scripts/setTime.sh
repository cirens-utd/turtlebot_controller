#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3

echo "Setting time for $pi_ip..."

current_time=$(date)
echo "$current_time"
ssh ${USER}@${pi_ip} << EOF

    sudo date -s '$current_time'
    sudo timedatectl set-timezone America/Chicago

EOF