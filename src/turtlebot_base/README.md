# Turtlebot Base Simulator

## Overview
This will launch a gazebo simulaion with the very bare bones to contorl multiple turtle bots. Each turtle bot has a lidar sensor and a camera.

## Launching
To launch, just run ros2 launch turtlebot_base launch_sim.launch.py
- By default, this will load the configuration that is in the config/simulation_config.yaml file

## Configuration Declarations
- yaml_load := when set to a value other than True, this will allow the other arguments to be used
- robot_number := dynamically set the number of robots to be spawn
- world_name := the srf file to be loaded. This must be in the worlds directory

## Ros Topics
- Each robot will have its own topics. This is setup to match the layout in the CIReNS lab in dallas
- Each robot will start with /robot#/ where number is the number you are controlling
- The global position will use the name /vrpn_mocap/turtlebot#/pose

### Topics Examples
- /robot1/camera_info
    information about the camera
- /robot1/cmd_vel
    used to move the robot
- /robot1/oakd/rgb/preview/image_raw
    image topic information
    frame is base_link
- /robot1/odometry
    simulated odometry 
    This doesn't show up in RViz as a fixed frame though??
- /robot1/scan
    lidar scan 
- /robot1/scan/points
    lidar scan point
- /vrpn_mocap/turtlebot1/pose
    x,y,z and rotation cordinates from an "external vision system"