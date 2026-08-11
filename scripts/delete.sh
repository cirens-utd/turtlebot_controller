#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3

echo "Clearing Log Files to $pi_ip..."
ssh ${USER}@${pi_ip} "rm -rf ${pi_wrk_space}/build ${pi_wrk_space}/install ${pi_wrk_space}/log"