from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def _simulation_launch_description():

    bringup_share = get_package_share_directory("asn_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")
    navigation_share = get_package_share_directory("asn_navigation")

    localization_launch_path = os.path.join(
        bringup_share,
        "launch",
        "localization.launch.py"
    )

    nav_launch_path = os.path.join(
        nav2_share,
        "launch",
        "navigation_launch.py"
    )

    nav2_params_path = os.path.join(
        navigation_share,
        "config",
        "nav2_navigation.yaml"
    )

    localization_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path)
    )

    navigation_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_path),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "params_file": nav2_params_path,
            "use_composition": "False",
        }.items()
    )

    return LaunchDescription([
        localization_launcher,
        navigation_launcher,
    ])

def _select_profile(context):
    from launch.substitutions import LaunchConfiguration
    if LaunchConfiguration('hardware').perform(context).lower() in ('true', '1', 'yes'):
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('asn_bringup'), 'launch', 'hardware', 'navigation_hardware.launch.py')))]
    return list(_simulation_launch_description().entities)


def generate_launch_description():
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='false'),
        OpaqueFunction(function=_select_profile),
    ])
