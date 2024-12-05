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


def get_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def update_locations(loc, mode=0):
    new_loc = []
    
    if mode == 0:       # move up 2m
        x = 2
        y = 0
    elif mode == 1:     # move down 2m
        x = -2
        y = 0
    elif mode == 2:     # move left 2m
        x = 0
        y = 2
    elif mode == 3:     # move right 2m
        x = 0
        y = -2
    elif mode == 4:     # move up to the right
        x = 2
        y = 2
    elif mode == 5:     # move down to the left
        x = -2
        y = -2
    elif mode == 6:     # move up to the left
        x = 2
        y = -2
    elif mode == 7:     # move down to the right
        x = -2
        y = 2

    for i in range(len(loc)):
        new_loc.append((loc[i][0] + x, loc[i][1] + y))
    return new_loc

def launch_dynamic_nodes(context: LaunchContext, *args, **kwargs):
    yaml_load = LaunchConfiguration('yaml_load').perform(context)

    if yaml_load == 'True':
        package_name='turtlebot_base'
        pkg_path = os.path.join(get_package_share_directory(package_name))
        sim_path_config = os.path.join(pkg_path,'config','simulation_config.yaml')
        yaml_info = get_yaml(sim_path_config)
        locations = np.array(yaml_info['locations'])
        if yaml_info['randamize']:
            locations += np.random.normal(0, yaml_info['noise'], locations.shape)
        number_robots = yaml_info['number_robots']
        world_name = yaml_info['world_name']
    else:
        number_robots = int(LaunchConfiguration('robot_number').perform(context))
        world_name = str(LaunchConfiguration('world_name').perform(context))
        locations = [[0,0], [-1,0], [1,0], [0, 1], [0,-1], [1,1], [-1,-1], [1,-1], [-1,1]]

    mode = 0

    nodes = []

    for i in range(number_robots):
        if len(locations) * (mode+1) <= i:
            locations = update_locations(locations, mode)
            mode += 1
        index = i % len(locations)

        robot_name = f"robot{i+1}"
        lidar_topic = f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/laser/scan'
        camera_base = f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/camera/'
        camera_topic = camera_base + 'image'
        camera_info = camera_base + 'camera_info'

        #  Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
        spawn_entity = Node(package='ros_gz_sim', executable='create',
                            arguments=['-world', world_name, '-topic', f'robot_description',
                                    '-name', robot_name, '-z', '0.05', '-x', str(locations[index][0]), '-y', str(locations[index][1])],
                            output='screen')
        
        # Run Bridges for ros
        cmd_vel =  Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=[f'/model/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                                f'/model/{robot_name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                                f'/model/{robot_name}/tf@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'],
                        remappings=[
                                    (f'/model/{robot_name}/cmd_vel', f'{robot_name}/cmd_vel'),
                                    (f'/model/{robot_name}/odometry', f'{robot_name}/odometry'),
                                    (f'/model/{robot_name}/tf', f'{robot_name}/tf')
                                    # (f'/model/{robot_name}/tf', f'/vrpn_mocap/turtlebot{i+1}/pose')
                        ],
                        output='screen'
                    )

        lidar_msg = Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=[f'{lidar_topic}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                                f'{lidar_topic}/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
                        remappings=[
                            (f'{lidar_topic}', f'{robot_name}/scan'),
                            (f'{lidar_topic}/points', f'{robot_name}/scan/points')
                        ],
                        output='screen'
                    )

        camera_pub = Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=[f'{camera_topic}@sensor_msgs/msg/Image@gz.msgs.Image',
                                f'{camera_info}@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
                        remappings=[
                            (f'{camera_topic}', f'{robot_name}/oakd/rgb/preview/image_raw'),
                            (f'{camera_info}', f'{robot_name}/camera_info')
                        ],
                        output='screen'
                    )

        dynamic_pose_pub = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/world/{world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            remappings=[
                (f'/world/{world_name}/dynamic_pose/info', '/testpose'),
            ]
        )

        nodes.append(dynamic_pose_pub)
        nodes.append(spawn_entity)
        nodes.append(cmd_vel)
        nodes.append(lidar_msg)
        nodes.append(camera_pub)
    return nodes




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

    dynamic_nodes = OpaqueFunction(function=launch_dynamic_nodes)

    # Launch them all!
    return LaunchDescription([
        robot_number_arg,
        world_name_arg,
        yaml_load_arg,
        rsp,
        gazebo,
        joint_publisher, 
        odom_pub,
        dynamic_nodes
    ])