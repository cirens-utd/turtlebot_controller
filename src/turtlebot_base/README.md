# Turtlebot Base Simulator

# Requirements
Need to have ros_gz installed (sudo apt install ros-humble-ros-gz)
Need to build custom package cirens_ros_bridge and have it sourced to use c_ros_gz_bridge
    - This can be found in the CIReNS Github
    https://github.com/cirens-utd/cirens_ros_bridge

# Overview
This will launch a gazebo simulaion with the very bare bones to contorl multiple turtle bots. Each turtle bot has a lidar sensor and a camera. Due to errors with timing. there are two main scripts to launch. One for the enviorment and one for the bots. More documentation to come to creating custom worlds.

## Launching
### World
Command: ros2 launch turtlebot_base launch_sim.launch.py
- By default, this will launch the empty.sdf file that is located in the worlds folder
- This can be changed by passing in 
    - world_sdf (Default: empty.sdf)

### Robots
Command: ros2 launch turtlebot_base launch_robots.launch.py
- Arguments:
    - yaml_load     (Default: True. Set to False if wanting to use other arguments)
    - robot_number  (Default: 1)
    - wolrd_name    (Defulat: empty_world)

The yaml file to be loaded will be in teh config folder and named simulation_config.yaml

## Ros Topics
- Each robot will have its own topics. This is setup to match the layout in the CIReNS lab in Dallas
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

# Trouble Shooting
## Install Notes:
Installed ros using the deb package
    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#install-ros-2-packages
Get colcon stuff:
    sudo apt install python3-colcon-common-extensions
Build c_ros_gz_bridge
    This has given some issues on new machines

## Issues found on the lab PC
- Missing packages below, 
    - Not sure if we need to do a package install of ros-humble-ros-gz-interfaces first? Need to check into this
- when trying to build, it cannot find module "catkin_pkg"
    - pip install catkin_pkg
- Modules that cannot be found 
    - "em"
        - pip install empy==3.3.4
    - "lark"
        - pip install lark

### ProtoBuff Error - No resolution yet
- We haven't found a way around this yet..
- Now we are getting that the protoc was built by an older version
    - my vm runs 3.12.4
    - Can check protobuf version by 
        python -c "import google.protobuf; print(google.protobuf.__version__)"

## Ros Discovery Mode
- Our lab uses the turtle bots in descovery mode (aka, not simple mode). To switch between the two.

Change ros to SimpleMode:
- change ~/.bashrc file
    Should have the descovery mode ros sourced. You can comment this out and then use typical ros
    - Discovery: source /etc/turtlebot4_discovery/setup.bash
    - Simple:   source /opt/ros/humble/setup.bash
                export RMW_IMPLEMENTATION=rmw_fastrtp_cpp
                export ROS_DOMAIN_ID=0
- ros2 dameon stop
- ros2 dameon start
