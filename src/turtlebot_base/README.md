# Turtlebot Base Simulator

# Requirements
Need to have ros_gz installed (sudo apt install ros-humble-ros-gz)
Need to build custom package cirens_ros_bridge and have it sourced to use c_ros_gz_bridge

# issues when installing on new machine
Installed ros using the debpackage
Get colcon stuff:
    sudo apt install python3-colcon-common-extensions


- when trying to build, it cannot find module "catkin_pkg"
    - pip install catkin_pkg
- Modules that cannot be found 
    - "em"
        - pip install empy==3.3.4
    - "lark"
        - pip install lark
- Now we are getting that the protoc was built by an older version
    - my vm runs 3.12.4

---FAILED
Lets attempt to build ros_gz from source
 - google ros_gz and go to humble branch. 
 - follow read me to get clone and do this fun stuff
    - still required catkin_pkg, em, lark, numpy
- back to the protoc was built by an older version 

-- still failing
    - ros-humble-ros-gz-interfaces


Remove ros and gazebo. Went into enviorment with protoc 3.12.4
Then sudo apt-get install ros-humble-ros-gz

Version of protobuf was still showing to be 3.21.12
Can check protobuf version by 
    python -c "import google.protobuf; print(google.protobuf.__version__)"

Finally created an enviorment with the correct version of protoc. Did a fresh install of ros. and then ros-humbel-ros-gz (to get gazebo)
- Then needed to get catkin_pkg, empy, lark, numpy
--Still too old of a version for protobuf

Change ros to discovery:
- change ~/.bashrc file
- ros2 dameon stop
- ros2 dameon start

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