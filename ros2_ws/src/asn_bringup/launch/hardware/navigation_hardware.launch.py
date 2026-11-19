import fcntl
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml

def include(package, filename, arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share(package), 'launch', filename)), launch_arguments=arguments.items())

def check_map(context):
    path = LC('map').perform(context)
    if not path or not os.path.isfile(os.path.expanduser(path)):
        raise RuntimeError('Supply map:=/absolute/path/to/your_saved_map.yaml; map file does not exist: ' + path)
    # Unique container names do not isolate root-namespace lifecycle services.
    lock_dir = Path(os.environ.get('XDG_RUNTIME_DIR') or Path.home() / '.ros')
    lock_dir.mkdir(exist_ok=True)
    lock = (lock_dir / 'asn-hardware-navigation.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for proc in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                args = proc.read_bytes().split(b'\0')
            except OSError:
                continue
            if (args and os.path.basename(args[0]) == b'component_container_isolated'
                    and any(a.startswith(b'__node:=asn_nav2_container') for a in args)):
                raise RuntimeError('A project Nav2 container is already running')
    except BlockingIOError as exc:
        lock.close()
        raise RuntimeError('A hardware navigation launch already holds the lock') from exc
    except Exception:
        lock.close()
        raise
    context.extend_globals({'asn_hardware_navigation_lock': lock})
    return []

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('container_name', default_value='asn_nav2_container_' + str(os.getpid())),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('navigation_autostart', default_value=LC('autostart')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/nav2_validation_cmd_vel', description='Final monitored TwistStamped output; isolated by default'),
        OpaqueFunction(function=check_map),
        Node(package='rclcpp_components', executable='component_container_isolated', name=LC('container_name'), output='screen',
             parameters=[os.path.join(share('asn_navigation'), 'config', 'hardware', 'nav2_navigation.yaml')]),
        include('asn_bringup', 'hardware/localization_hardware.launch.py', {'map': LC('map'), 'autostart': LC('autostart'), 'use_composition': 'True', 'container_name': LC('container_name'), 'create_container': 'false'}),
        GroupAction([
            SetRemap(src='docking_server:cmd_vel', dst='cmd_vel_nav'),
            include('nav2_bringup', 'navigation_launch.py', {
                'use_sim_time': 'false', 'autostart': LC('navigation_autostart'), 'use_composition': 'True', 'container_name': LC('container_name'),
                'params_file': RewrittenYaml(
                    source_file=os.path.join(share('asn_navigation'), 'config', 'hardware', 'nav2_navigation.yaml'),
                    param_rewrites={'cmd_vel_out_topic': LC('cmd_vel_topic')},
                    convert_types=True)}),
        ]),
        Node(package='rviz2', executable='rviz2', name='navigation_rviz',
             condition=IfCondition(LC('use_rviz')),
             arguments=['-d', os.path.join(share('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
             parameters=[{'use_sim_time': False}], output='screen'),
    ])
