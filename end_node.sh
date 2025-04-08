#!/bin/bash

# This script ends the agent node and any other dependencies listed in this file

# Define the list of nodes you want to kill (you can add more nodes to this list)
TARGET_NODES=(
    "/vrpn_mocap/vrpn_mocap_client_node"
)

TARGET_SCRIPTS=(
    "consensus.py"
)

# Loop through the target nodes and kill them if they are running
for node in "${TARGET_NODES[@]}"; do
    # Check if the node is running
    if ros2 node list | grep -q "$node"; then
        echo "Node $node found. Killing it..."
        
        # Kill the node
        ros2 node kill "$node"
        echo "Node $node has been killed."
    else
        echo "Node $node is not running."
    fi
done


for script in "${TARGET_SCRIPTS[@]}"; do
    pkill -f "$script"
done
