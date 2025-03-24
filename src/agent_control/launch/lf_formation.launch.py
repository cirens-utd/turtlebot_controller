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
    sim_mode = LaunchConfiguration('sim_mode').perform(context)

    package_name = 'agent_control'
    
    pkg_path = os.path.join(get_package_share_directory(package_name))
    path_config = os.path.join(pkg_path,'config','agent_setup','agent_setup.yaml')
    yaml_info = get_yaml(path_config)
    numbers = yaml_info['robot_numbers']
    world_name = yaml_info['world_name']

    sim_argument = []
    if sim_mode == "True":
        sim_argument = ['-s']
    
    nodes = []
    for i in numbers:
        nodes.append(
            Node(
                package=package_name,
                executable='LF_formation.py',
                output='screen',
                arguments=[
                    '-i', str(i), 
                    '-f', str(path_config)
                ] + sim_argument
            )
        )
    return nodes

def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='False',
        description="Run in simulation formate"
    )
    dynamic_nodes = OpaqueFunction(function=launch_dynamic_nodes)
    
    return LaunchDescription([
        sim_arg,
        dynamic_nodes
    ])