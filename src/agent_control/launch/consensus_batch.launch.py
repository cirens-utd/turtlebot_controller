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

def launch_dynamic_nodes(context: LaunchContext, *args, **kwargs):
    package_name = 'agent_control'
    yaml_load = LaunchConfiguration('yaml_load').perform(context)
    number_robots = LaunchConfiguration('number_robots').perform(context)
    numbers = np.arange(1, int(number_robots) + 1)
    if yaml_load == 'True':
        pkg_path = os.path.join(get_package_share_directory(package_name))
        path_config = os.path.join(pkg_path,'config','consensus_config.yaml')
        yaml_info = get_yaml(path_config)
        numbers = yaml_info['robot_numbers']
        world_name = yaml_info['world_name']
    
    nodes = []
    for i in numbers:
        neighbors = [str(j) for j in numbers if j != i]
        nodes.append(
            Node(
                package=package_name,
                executable='consensus_node',
                output='screen',
                arguments=[
                    '-i', str(i), 
                    '-n'
                ] + [str(item) for sublist in [[neighbor] for neighbor in neighbors] for item in sublist]
            )
        )
    return nodes

def generate_launch_description():
    number_robots_arg = DeclareLaunchArgument(
        'number_robots',
        default_value='10',
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
        description="Load from config/consensus_config.yaml or load with Launch arguments"
    )

    dynamic_nodes = OpaqueFunction(function=launch_dynamic_nodes)

    # Launch them all!
    return LaunchDescription([
        number_robots_arg,
        world_name_arg,
        yaml_load_arg,
        dynamic_nodes
    ])