import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import yaml
import argparse
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    param_file = os.path.join(get_package_share_directory('farmbot_controller'), 'config', 'params.yaml')

    nodes_array = []
    
    zeroturn = Node(
        package='farmbot_controller',
        namespace=namespace,
        executable='zeroturn',
        name='zeroturn',
        parameters=[
            {"frame_prefix": namespace+"/"},
            {"namespace": namespace},
            yaml.safe_load(open(param_file))['zeroturn']['ros__parameters'], 
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    nodes_array.append(zeroturn)

    return nodes_array


def generate_launch_description(): 
    namespace_arg = DeclareLaunchArgument('namespace', default_value='fb')
    antena_arg = DeclareLaunchArgument('double_antenna', default_value='True')
    
    return LaunchDescription([
        namespace_arg,
        antena_arg, 
        OpaqueFunction(function = launch_setup)
    ])