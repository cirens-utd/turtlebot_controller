import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.launch_context import LaunchContext

import numpy as np
import yaml
import pdb




def generate_launch_description():
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='1',
        description="Number of robots to spawn"
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty_world',
        description="Name of world"
    )
    yaml_load_arg = DeclareLaunchArgument(
        'yaml_load',
        default_value='True',
        description="Load from config/simulation_config.yaml or load with Launch arguments"
    )

    package_name='turtlebot_base'
    # pkg_path = os.path.join(get_package_share_directory(package_name))
    # sim_path_config = os.path.join(pkg_path,'config','simulation_config.yaml')
    # my_yaml = get_yaml(sim_path_config)
    # pdb.set_trace()

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments=[
                    ('gz_args', os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.sdf') + " -r")
                ]
             )
        
    # Bring up joint states
    joint_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # odom_tf
    odom_pub = Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        '/gui/camera/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
                    ],
                    remappings=[
                        ('/gui/camera/pose', '/odom'),
                    ]
                )

    # robot_nodes = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','launch_robots.launch.py'
    #             )])
    # )

    # Launch them all!
    return LaunchDescription([
        robot_number_arg,
        world_name_arg,
        yaml_load_arg,
        rsp,
        gazebo,
        joint_publisher, 
        odom_pub
    ])