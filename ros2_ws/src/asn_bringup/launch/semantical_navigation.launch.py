import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

from launch.actions import ExecuteProcess

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

    simulation_path = os.path.join(
        bringup_share, "launch", "simulation.launch.py"
    )

    navigation_path = os.path.join(
        bringup_share, "launch", "navigation.launch.py"
    )

    rviz_path = os.path.expanduser(
        "~/projects/autonomous-semantic-navigation/"
        "ros2_ws/src/asn_semantical_navigation/"
        "rviz/Semantical_nav_map.rviz"
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

    approach_goal_node = Node(
        package="asn_semantical_navigation",
        executable="approach_goal_node",
        name="approach_goal_node",
        output="screen",
        parameters=[{
            'use_sim_time': True,
            'stand_off_m': 0.60,
            'clearance_m': 0.22,
            'target_path_clearance_m': 0.30,
            'target_timeout_s': 2.5,
            'robot_frame': 'base_link',
        }]
    )

    object_localization_node = Node(
        package="asn_semantical_navigation",
        executable="object_localizer_node",
        name="object_localizer_node",
        output="screen",
        parameters=[{
            'use_sim_time': True,
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera_info',
            'detections_topic': '/semantic/search/detections_json',
            'map_frame': 'map',
            'max_sync_difference_s': 0.10,
            'min_depth_m': 0.15,
            'max_depth_m': 5.0,
            'roi_fraction': 0.40,
            'min_valid_pixels': 20,
            'min_valid_fraction': 0.30,
            'max_depth_spread_m': 1.0,
            'confirmation_positions': 3,
            'max_position_jitter_m': 0.45,
            'target_timeout_s': 2.0,
        }]
    )

    yolo_detector_node = ExecuteProcess(
        cmd=[
            os.path.expanduser(
                '~/projects/autonomous-semantic-navigation/'
                '.venv-perception/bin/python'
            ),
            '-m',
            'asn_semantical_navigation.yolo_detector_node',
            '--ros-args',
            '-p', 'image_topic:=/camera/color/image_raw',
            '-p', 'model:=yolo11n.pt',
            '-p', 'device:=cpu',
            '-p', 'confidence_threshold:=0.55',
            '-p', 'confirmation_frames:=3',
            '-p', 'processing_hz:=2.0',
            '-p', 'use_sim_time:=true',
        ],
        output='screen'
    )

    search_command_node = Node(
        package='asn_semantical_navigation',
        executable='search_command_node',
        name='search_command_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
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
            'auto_max_goals',
            default_value='5'
        ),

        simulation_launcher,
        navigation_launcher,
        coverage_node,
        autonomous_node,
        rviz_node,

        search_command_node,
        yolo_detector_node,
        object_localization_node,
        approach_goal_node,
    ])