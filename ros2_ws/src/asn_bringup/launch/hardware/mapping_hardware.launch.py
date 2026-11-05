import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node, SetRemap

def include(package, filename, arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share(package), 'launch', filename)), launch_arguments=arguments.items())

def check_map(context):
    path = LC('map').perform(context)
    if not path or not os.path.isfile(os.path.expanduser(path)):
        raise RuntimeError('Supply map:=/absolute/path/to/your_saved_map.yaml; map file does not exist: ' + path)
    return []

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        include('slam_toolbox', 'online_async_launch.py', {
            'use_sim_time': 'false', 'autostart': 'true', 'use_lifecycle_manager': 'false',
            'slam_params_file': os.path.join(share('asn_navigation'), 'config', 'hardware', 'slam_toolbox_mapping.yaml')}),
        Node(package='rviz2', executable='rviz2', name='mapping_rviz',
             condition=IfCondition(LC('use_rviz')),
             arguments=['-d', os.path.join(share('asn_navigation'), 'rviz', 'hardware', 'mapping.rviz')],
             parameters=[{'use_sim_time': False}], output='screen'),
    ])
