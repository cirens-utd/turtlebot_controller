#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3

echo "Syncing up files to $pi_ip..."
rsync -avz --delete ./src ${USER}@${pi_ip}:${pi_wrk_space}
