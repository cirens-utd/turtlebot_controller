#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3

echo "Syncing up Script files to $pi_ip..."
rsync -avz --delete ./scripts ${USER}@${pi_ip}:${pi_wrk_space}
