#!/bin/bash

source /opt/ros/humble/setup.bash
source /etc/turtlebot4/setup.bash

if [ $(ps aux | grep vrpn | wc -l) -le 1 ]; then
	#screen -dmS ros_session1 bash -c "ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.131 port:=3883 update_freq:=180"
	tmux new-session -d -s ros_session1 "bash -c 'ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.3 port:=3883 update_freq:=180'"
fi
