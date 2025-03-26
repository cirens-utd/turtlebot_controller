# ros_pose_scrapper

## name_array_node
This is created with 3 parameters:
- robot_name
- source_topic
- new_name

This will subscirbe to the topic {source_topic} (Default to /encoded_poses) that is created by the cirens_ros_bridge.  This will break out the comma delimited list in the Frame_id and then republish the pose as a PoseStamped with the Frame_ID being the name for each item. The robot_name must match that used in Gazebo. The new_name will allow you to change the name for casting to the mocap topic.

## Naming Scheme.
This assumes the robot are named the same, just the number at the end changing. 
- Robot Name
    - robot1
    - ...
    - robot22
- Mocap name
    - turtlebot1
    - ...
    - turtlebot22

## Final Topic
/vrpn_mocap/{new_name+number}/pose