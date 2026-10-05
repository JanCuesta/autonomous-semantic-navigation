from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    #Package locations
    nav2_share = get_package_share_directory("nav2_bringup")
    asn_navigation_share = get_package_share_directory("asn_navigation")

    nav2_launch_path = os.path.join(
        nav2_share,
        "launch",
        "localization_launch.py"
    )

    params_path = os.path.join(
        asn_navigation_share,
        "config",
        "amcl_localization.yaml"
    )

    map_path = os.path.join(
        asn_navigation_share,
        "maps",
        "v1_sim_map.yaml"
    )

    localization_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            "map": map_path,
            "params_file": params_path,
            "use_sim_time": "true",
            "autostart":"true",
            "use_composition":"False"
        }.items()
    )

    return LaunchDescription([
        localization_launcher,
    ])