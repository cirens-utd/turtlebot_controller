#!/bin/bash

USER="ubuntu"
pi_wrk_space="/home/$USER/Turtlebot_Controller"

# Check if we have arguments (robot numbers)
if [ $# -lt 2 ]; then
  echo "Usage: $0 <operation> <robot_number1> <robot_number2> ..."
  echo "Operations: copy, sync, build, deepBuid, run, stop, pullLogs, calibrate, testCalibrate, test"
  echo "Example: $0 copy 1 3 5"
  exit 1
fi

operation="$1"

# remove operation from arguments
shift

# Detect OS
OS_TYPE="$(uname)"
open_terminal() {
  local cmd="cd $(pwd) && ./scripts/${operation}.sh $USER $pi_ip $pi_wrk_space $robot_num $@ || exec bash"

  if [[ "$OS_TYPE" == "Darwin" ]]; then
    # macOS
    osascript <<EOF
tell application "Terminal"
    do script "$cmd"
end tell
EOF
  elif [[ "$OS_TYPE" == "Linux" ]]; then
    # Linux (assuming gnome-terminal is installed)
    gnome-terminal -- bash -c "$cmd"
  else
    echo "Unsupported OS: $OS_TYPE"
    exit 1
  fi
}

# loop through the robot numbers
for robot_num in "$@"; do
    # IP Address
    pi_ip="192.168.0.$((220 + robot_num))"

    echo "Performing '$operation' on robot$robot_num at IP $pi_ip..."
    open_terminal
    echo "Operation '$operation' triggered for robot$robot_num at $pi_ip."
done

echo "All operations complete"
