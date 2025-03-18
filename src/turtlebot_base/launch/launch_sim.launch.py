import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.launch_context import LaunchContext


package_name='turtlebot_base'

def world_function_create(context: LaunchContext, *args, **kwargs):
    global package_name

    world_name = LaunchConfiguration('world_sdf').perform(context)

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments=[
            ('gz_args', os.path.join(get_package_share_directory(package_name), 'worlds', world_name) + " -r")
        ]
        )
    return [gazebo]


def generate_launch_description():
    global package_name

    world_sdf_arg = DeclareLaunchArgument(
        'world_sdf',
        default_value='empty.sdf',
        description="SDF File that is in the worlds folder"
    )


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    world_node = OpaqueFunction(function=world_function_create)
        
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

    # Launch them all!
    return LaunchDescription([
        world_sdf_arg,
        rsp,
        world_node,
        joint_publisher, 
        odom_pub
    ])