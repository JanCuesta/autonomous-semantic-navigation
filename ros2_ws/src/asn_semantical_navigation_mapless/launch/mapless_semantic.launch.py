"""Mapless semantic navigation. Start the robot drivers separately.

Deliberately launches NO SLAM, AMCL, map server, Nav2 planner, or costmap.
Motion is disabled and routed to isolated /mapless/cmd_vel by default.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PKG = 'asn_semantical_navigation_mapless'
ACTION_REMAPS = [
    ('/navigate_to_pose', '/mapless/navigate_to_pose'),
    ('/navigate_to_pose/_action/status',
     '/mapless/navigate_to_pose/_action/status'),
]


def generate_launch_description():
    time_value = ParameterValue(LC('use_sim_time'), value_type=bool)
    shared = {'use_sim_time': time_value}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/mapless/cmd_vel'),
        DeclareLaunchArgument('twist_stamped', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('rgb_topic',
                              default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic',
                              default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/camera_info'),
        DeclareLaunchArgument('robot_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('auto_max_goals', default_value='25'),
        DeclareLaunchArgument('start_yolo', default_value='false'),
        DeclareLaunchArgument('perception_python', default_value=os.path.expanduser(
            '~/projects/autonomous-semantic-navigation/.venv-perception/bin/python')),
        Node(package=PKG, executable='mapless_motion_node',
             name='mapless_motion_node', output='screen', parameters=[shared, {
                 'enable_motion': ParameterValue(LC('enable_motion'),
                                                 value_type=bool),
                 'cmd_vel_topic': LC('cmd_vel_topic'),
                 'twist_stamped': ParameterValue(LC('twist_stamped'),
                                                value_type=bool),
                 'scan_topic': LC('scan_topic'),
                 'robot_frame': LC('robot_frame'),
                 'odom_frame': LC('odom_frame'),
             }]),
        Node(package=PKG, executable='search_command_node',
             parameters=[shared], output='screen'),
        # Preserved detector from the copied semantic package. Requires the
        # user's original ultralytics-enabled perception environment.
        ExecuteProcess(condition=IfCondition(LC('start_yolo')), output='screen',
                       cmd=[LC('perception_python'), '-m',
                            PKG + '.yolo_detector_node', '--ros-args',
                            '-p', ['image_topic:=', LC('rgb_topic')],
                            '-p', ['use_sim_time:=', LC('use_sim_time')]]),
        Node(package=PKG, executable='object_localizer_node',
             name='object_localizer_node', output='screen',
             remappings=ACTION_REMAPS, parameters=[shared, {
                 'map_frame': LC('odom_frame'),
                 'depth_topic': LC('depth_topic'),
                 'camera_info_topic': LC('camera_info_topic'),
             }]),
        Node(package=PKG, executable='nbv_auto_node',
             name='nbv_auto_node', output='screen', parameters=[shared, {
                 'scan_topic': LC('scan_topic'),
                 'robot_frame': LC('robot_frame'),
                 'odom_frame': LC('odom_frame'),
                 'auto_max_goals': ParameterValue(LC('auto_max_goals'),
                                                  value_type=int),
                 'stop_on_2d_confirmation': False,
             }]),
        Node(package=PKG, executable='approach_goal_node',
             name='approach_goal_node', output='screen', parameters=[shared, {
                 'scan_topic': LC('scan_topic'),
                 'robot_frame': LC('robot_frame'),
                 'odom_frame': LC('odom_frame'),
             }]),
        Node(package=PKG, executable='semantical_mission_manager_node',
             name='semantical_mission_manager_node', output='screen',
             remappings=ACTION_REMAPS, parameters=[shared]),
    ])
