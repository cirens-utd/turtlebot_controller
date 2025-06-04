#!/bin/bash

USER=$1
pi_ip=$2
pi_wrk_space=$3
robot_num=$4
shift
shift
shift
shift

./scripts/stop.sh $USER $pi_ip $pi_wrk_space $robot_num $@ && \
./scripts/copy.sh $USER $pi_ip $pi_wrk_space $robot_num $@ && \
./scripts/run.sh $USER $pi_ip $pi_wrk_space $robot_num $@

