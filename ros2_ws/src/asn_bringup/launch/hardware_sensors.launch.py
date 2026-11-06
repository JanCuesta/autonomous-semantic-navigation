"""Stationary laptop RGB-D transport and reduced coverage cloud; no motion nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth = LaunchConfiguration('depth_topic')
    info = LaunchConfiguration('camera_info_topic')
    return LaunchDescription([
        DeclareLaunchArgument('depth_topic', default_value='/oakd/stereo/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/oakd/stereo/camera_info'),
        Node(package='image_transport', executable='republish',
             name='hardware_depth_decode', output='screen',
             parameters=[{'in_transport': 'compressedDepth', 'out_transport': 'raw',
                          'use_sim_time': False}],
             remappings=[('in', depth),
                         ('in/compressedDepth', [depth, '/compressedDepth']),
                         ('out', '/hardware_depth/image_raw')]),
        Node(package='image_proc', executable='crop_decimate_node',
             name='coverage_depth_resize', output='screen',
             parameters=[{'image_transport': 'raw', 'decimation_x': 4,
                          'decimation_y': 4, 'interpolation': 0, 'use_sim_time': False}],
             remappings=[('in/image_raw', '/hardware_depth/image_raw'), ('in/camera_info', info),
                         ('/hardware_depth/camera_info', info),
                         ('out/image_raw', '/coverage_depth/image_raw'),
                         ('out/camera_info', '/coverage_depth/camera_info')]),
        Node(package='depth_image_proc', executable='point_cloud_xyz_node',
             name='hardware_depth_cloud', output='screen',
             parameters=[{'depth_image_transport': 'raw', 'use_sim_time': False}],
             remappings=[('image_rect', '/coverage_depth/image_raw'),
                         ('camera_info', '/coverage_depth/camera_info'),
                         ('points', '/depth_pcl')]),
    ])
