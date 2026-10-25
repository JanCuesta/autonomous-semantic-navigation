import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

# Integrated semantic navigation is explicitly operator-started:
#
#   ros2 topic pub --once /semantic/search/command std_msgs/msg/String \
#       "{data: 'find a backpack'}"
#   ros2 service call /semantic/mission/start std_srvs/srv/Trigger '{}'
#
# Cancel a running semantic mission:
#   ros2 service call /semantic/mission/cancel std_srvs/srv/Trigger '{}'
#
# Do NOT manually arm /semantic/nbv/auto_enable or call
# /semantic/approach/send_goal during a semantic mission. The supervisor
# coordinates the NBV -> approach ownership handoff.

def generate_launch_description():
    bringup_share = get_package_share_directory("asn_bringup")

    simulation_path = os.path.join(
        bringup_share, "launch", "simulation.launch.py"
    )
    navigation_path = os.path.join(
        bringup_share, "launch", "navigation.launch.py"
    )

    # Preserve the existing project-local RViz configuration.
    rviz_path = os.path.expanduser(
        "~/projects/autonomous-semantic-navigation/"
        "ros2_ws/src/asn_semantical_navigation/"
        "rviz/Semantical_nav_map.rviz"
    )

    simulation_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_path)
    )
    navigation_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_path)
    )

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

    # Keep the approach node running to publish preview candidates.
    # Its manual /semantic/approach/send_goal service MUST NOT be invoked
    # during a semantic mission; the manager owns the final Nav2 goal.
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

    # YOLO runs in its separate perception environment, as before.
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

    # The manager is the only component that arms NBV exploration during a
    # semantic mission. Require an explicit target and live YOLO frames.
    autonomous_node = Node(
        package="asn_semantical_navigation",
        executable="nbv_auto_node",
        name="nbv_auto_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "require_target": True,
            "require_live_detector": True,
            # For integrated missions, only a validated RGB-D map pose
            # should stop NBV; 2D YOLO confirms visibility, not position.
            "stop_on_2d_confirmation": False,
            "auto_max_goals": ParameterValue(
                LaunchConfiguration("auto_max_goals"),
                value_type=int
            ),
            "auto_goal_timeout_s": 90.0,
            "frontier_enabled": True,
            "frontier_threshold": 30,
        }]
    )

    # Does not move on startup; /semantic/mission/start is required.
    semantic_mission_manager_node = Node(
        package="asn_semantical_navigation",
        executable="semantical_mission_manager_node",
        name="semantical_mission_manager_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration('start_mission_manager')),
        parameters=[{
            "use_sim_time": True,
            "prepare_timeout_s": 30.0,
            "detector_stale_s": 5.0,
            "stop_timeout_s": 25.0,
            "localization_timeout_s": 20.0,
            "approach_timeout_s": 120.0,
        }]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('auto_max_goals', default_value='5'),
        DeclareLaunchArgument(
                'start_mission_manager',
                default_value='true'
         ),
        simulation_launcher,
        navigation_launcher,
        coverage_node,
        autonomous_node,
        semantic_mission_manager_node,
        rviz_node,
        search_command_node,
        yolo_detector_node,
        object_localization_node,
        approach_goal_node,
    ])
