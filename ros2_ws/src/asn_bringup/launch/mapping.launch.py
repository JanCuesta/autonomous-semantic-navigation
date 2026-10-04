from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # -------------------------
    # Package locations
    # -------------------------

    slam_share = get_package_share_directory("slam_toolbox")
    navigation_share = get_package_share_directory("asn_navigation")

    # -------------------------
    # File paths
    # -------------------------

    slam_launch_path = os.path.join(
        slam_share,
        "launch",
        "online_async_launch.py"
    )

    slam_params = os.path.join(
        navigation_share,
        "config",
        "slam_toolbox_mapping.yaml"
    )

    rviz_path = os.path.join(
        navigation_share,
        "rviz",
        "mapping.rviz"
    )

    # -------------------------
    # SLAM Toolbox
    # -------------------------

    slam_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": slam_params,
            "autostart": "true",
            "use_lifecycle_manager": "false",
        }.items()
    )

    # -------------------------
    # RViz
    # -------------------------

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            rviz_path
        ],
        parameters=[{
            "use_sim_time": True
        }],
        output="screen"
    )

    return LaunchDescription([
        slam_launcher,
        rviz,
    ])