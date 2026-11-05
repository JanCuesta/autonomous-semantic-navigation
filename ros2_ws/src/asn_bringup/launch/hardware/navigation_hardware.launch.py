import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC, PythonExpression
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
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel', description='TwistStamped output; override for stationary validation'),
        Node(package='rclcpp_components', executable='component_container_isolated', name=LC('container_name'), output='screen',
             parameters=[os.path.join(share('asn_navigation'), 'config', 'hardware', 'nav2_navigation.yaml')]),
        include('asn_bringup', 'hardware/localization_hardware.launch.py', {'map': LC('map'), 'autostart': LC('autostart'), 'use_composition': 'True', 'container_name': LC('container_name'), 'create_container': 'false'}),
        GroupAction([
            SetRemap(src='/cmd_vel', dst=LC('cmd_vel_topic'),
                     condition=IfCondition(PythonExpression(["'", LC('cmd_vel_topic'), "' != '/cmd_vel'"]))),
            include('nav2_bringup', 'navigation_launch.py', {
                'use_sim_time': 'false', 'autostart': LC('autostart'), 'use_composition': 'True', 'container_name': LC('container_name'),
                'params_file': os.path.join(share('asn_navigation'), 'config', 'hardware', 'nav2_navigation.yaml')}),
        ]),
        Node(package='rviz2', executable='rviz2', name='navigation_rviz',
             condition=IfCondition(LC('use_rviz')),
             arguments=['-d', os.path.join(share('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
             parameters=[{'use_sim_time': False}], output='screen'),
    ])
