"""Semantic pipeline for an already running, namespaced TurtleBot 4/Nav2.

Default mode starts only target selection and RGB detection. Full mode requires
verified aligned RGB-D, CameraInfo, PointCloud2, map, TF, and Nav2 interfaces.
Even in full mode, movement requires an explicit semantic mission start call.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _check_full_stack(context):
    if LaunchConfiguration('enable_full_stack').perform(context).lower() not in (
            'true', '1', 'yes'):
        return []
    required = ('depth_topic', 'camera_info_topic', 'pointcloud_topic')
    missing = [name for name in required
               if not LaunchConfiguration(name).perform(context).strip()]
    if missing:
        raise RuntimeError('Full semantic stack needs verified robot topics: '
                           + ', '.join(missing))
    if (LaunchConfiguration('map_frame').perform(context) != 'map' or
            LaunchConfiguration('robot_frame').perform(context) != 'base_link'):
        raise RuntimeError('The current mission supervisor requires TF frame '
                           'IDs map and base_link. Adapt its frame checks '
                           'before using different robot frame IDs.')
    return []


def generate_launch_description():
    namespace = LaunchConfiguration('robot_namespace')
    robot = PathJoinSubstitution(['/', namespace])
    map_topic = PathJoinSubstitution([robot, 'map'])
    nav_action = PathJoinSubstitution([robot, 'navigate_to_pose'])
    nav_status = PathJoinSubstitution(
        [robot, 'navigate_to_pose', '_action', 'status'])
    tf_topic = LaunchConfiguration('tf_topic')
    tf_static_topic = LaunchConfiguration('tf_static_topic')

    # Keep /semantic/* as project-local interfaces. Only the robot-facing
    # endpoints are remapped. Frame IDs are not changed by ROS namespaces.
    robot_remappings = [
        ('/map', map_topic), ('map', map_topic),
        ('/navigate_to_pose', nav_action),
        ('/navigate_to_pose/_action/status', nav_status),
        ('/speed_limit', PathJoinSubstitution([robot, 'speed_limit'])),
        ('/tf', tf_topic), ('/tf_static', tf_static_topic),
    ]
    full = IfCondition(LaunchConfiguration('enable_full_stack'))
    actual_time = {'use_sim_time': False}

    yolo = ExecuteProcess(
        cmd=[LaunchConfiguration('perception_python'), '-m',
             'asn_semantical_navigation.yolo_detector_node', '--ros-args',
             '-p', ['image_topic:=', LaunchConfiguration('rgb_topic')],
             '-p', 'compressed_image:=true',
             '-p', 'use_sim_time:=false'],
        output='screen')

    search = Node(
        package='asn_semantical_navigation', executable='search_command_node',
        name='search_command_node', parameters=[actual_time], output='screen')

    localizer = Node(
        package='asn_semantical_navigation', executable='object_localizer_node',
        name='object_localizer_node', condition=full, output='screen',
        remappings=robot_remappings,
        parameters=[actual_time, {
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'map_frame': LaunchConfiguration('map_frame'),
        }])

    coverage = Node(
        package='asn_semantical_navigation', executable='coverage_node',
        name='coverage_node', condition=full, output='screen',
        remappings=robot_remappings + [
            ('/depth_pcl', LaunchConfiguration('pointcloud_topic'))],
        parameters=[actual_time])

    approach = Node(
        package='asn_semantical_navigation', executable='approach_goal_node',
        name='approach_goal_node', condition=full, output='screen',
        remappings=robot_remappings,
        parameters=[actual_time, {
            'robot_frame': LaunchConfiguration('robot_frame'),
            'stand_off_m': 0.60,
            'clearance_m': 0.22,
            'target_path_clearance_m': 0.30,
        }])

    nbv = Node(
        package='asn_semantical_navigation', executable='nbv_auto_node',
        name='nbv_auto_node', condition=full, output='screen',
        remappings=robot_remappings,
        parameters=[actual_time, {
            'robot_frame': LaunchConfiguration('robot_frame'),
            'nav_action_name': nav_action,
            'require_target': True,
            'require_live_detector': True,
            'stop_on_2d_confirmation': False,
            'auto_max_goals': 5,
        }])

    mission = Node(
        package='asn_semantical_navigation',
        executable='semantical_mission_manager_node',
        name='semantical_mission_manager_node', condition=full,
        output='screen', remappings=robot_remappings,
        parameters=[actual_time])

    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='robot_09'),
        DeclareLaunchArgument('rgb_topic', default_value=PathJoinSubstitution(
            [robot, 'oakd', 'rgb', 'image_raw', 'compressed'])),
        DeclareLaunchArgument('depth_topic', default_value=''),
        DeclareLaunchArgument('camera_info_topic', default_value=''),
        DeclareLaunchArgument('pointcloud_topic', default_value=''),
        DeclareLaunchArgument('tf_topic', default_value=PathJoinSubstitution(
            [robot, 'tf'])),
        DeclareLaunchArgument('tf_static_topic', default_value=PathJoinSubstitution(
            [robot, 'tf_static'])),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('robot_frame', default_value='base_link'),
        DeclareLaunchArgument('enable_full_stack', default_value='false'),
        DeclareLaunchArgument('perception_python', default_value=os.path.expanduser(
            '~/projects/autonomous-semantic-navigation/.venv-perception/bin/python')),
        OpaqueFunction(function=_check_full_stack),
        search, yolo, localizer, coverage, approach, nbv, mission,
    ])
