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
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('container_name', default_value='asn_nav2_container_' + str(os.getpid())),
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('create_container', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=check_map),
        Node(package='rclcpp_components', executable='component_container_isolated',
             name=LC('container_name'), condition=IfCondition(LC('create_container')), output='screen'),
        include('nav2_bringup', 'localization_launch.py', {
            'map': LC('map'), 'use_sim_time': 'false', 'autostart': LC('autostart'),
            'use_composition': LC('use_composition'), 'container_name': LC('container_name'),
            'params_file': os.path.join(share('asn_navigation'), 'config', 'hardware', 'amcl_localization_hardware.yaml')}),
    ])
