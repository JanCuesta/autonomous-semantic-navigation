"""Navigation over the live SLAM Toolbox map, without AMCL or map_server."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slam_share = get_package_share_directory('slam_toolbox')
    navigation_share = get_package_share_directory('asn_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(
                navigation_share, 'config', 'slam_toolbox_mapping.yaml'),
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
        }.items())
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(
                navigation_share, 'config', 'nav2_navigation.yaml'),
            'use_composition': 'False',
        }.items())

    return LaunchDescription([slam, navigation])
