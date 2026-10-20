import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


# Start autonomous navigation:
# ros2 service call /semantic/nbv/auto_enable std_srvs/srv/SetBool '{data: true}'

# Stop autonomous navigation:
# ros2 service call /semantic/nbv/auto_enable std_srvs/srv/SetBool '{data: false}'


def generate_launch_description():

    # Package directories
    bringup_share = get_package_share_directory("asn_bringup")
    semantic_share = get_package_share_directory(
        "asn_semantical_navigation"
    )
    navigation_share = get_package_share_directory("asn_navigation")



    simulation_path = os.path.join(
        bringup_share, "launch", "simulation.launch.py"
    )

    navigation_path = os.path.join(
        bringup_share, "launch", "navigation.launch.py"
    )

    rviz_path = os.path.join(
        semantic_share, "rviz", "Semantical_nav_map.rviz"
    )

    # Simulation ROS interfaces
    simulation_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_path)
    )

    # Nav2 + AMCL
    navigation_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_path)
    )

    # RGB-D coverage confidence map
    coverage_node = Node(
        package="asn_semantical_navigation",
        executable="coverage_node",
        name="coverage_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "processing_hz": 2.0,
        }]
    )

    # Autonomous Next-Best-View navigation
    autonomous_node = Node(
        package="asn_semantical_navigation",
        executable="nbv_auto_node",
        name="nbv_auto_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "require_target": False,
            "require_live_detector": False,
            "auto_max_goals": ParameterValue(
                LaunchConfiguration("auto_max_goals"),
                value_type=int
            ),
            "auto_goal_timeout_s": 90.0,
            "frontier_enabled": True,
            "frontier_threshold": 30,
        }]
    )

    # RViz visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        parameters=[{
            "use_sim_time": True,
        }]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "auto_max_goals",
            default_value="5",
            description="Maximum autonomous goals per activation"
        ),

        simulation_launcher,
        navigation_launcher,
        coverage_node,
        autonomous_node,
        rviz_node,

    ])
