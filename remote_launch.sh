#!/bin/bash

USER="ubuntu"
pi_wrk_space="/home/$USER/Turtlebot_Controller"

# Check if we have arguments (robot numbers)
if [ $# -lt 2 ]; then
  echo "Usage: $0 <operation> <robot_number1> <robot_number2> ..."
  echo "Operations: build, copy, run, stop, test"
  echo "Example: $0 copy 1 3 5"
  exit 1
fi

operation="$1"

# remove operation from arguments
shift

# loop through the robot numbers
for robot_num in "$@"; do
    # IP Address
    pi_ip="192.168.0.$((220 + robot_num))"

    echo "Performing '$operation' on robot$robot_num at IP $pi_ip..."

    gnome-terminal -- bash -c "./scripts/${operation}.sh $USER $pi_ip $pi_wrk_space || exec bash"

    # # Handle invalid operation input
    # echo "Invalid operation: $operation"
    # echo "Valid operations are: build, copy, run, stop, test"
    # exit 1

    echo "Operation '$operation' completed for robot$robot_num at $pi_ip."
done

echo "All operations complete"
