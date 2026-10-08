from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    odometry = Node(
        package="asn_simulation",
        executable="asn_wheel_odom_unwrapped",
        name="asn_wheel_odom_unwrapped",
        parameters=[{
            "use_sim_time": True,
            "wheel_radius": 0.03575,
            "wheel_separation": 0.23822,
            "publish_tf": True,
        }],
    )



    description_share = get_package_share_directory("asn_description")

    urdf_path = os.path.join(
        description_share,
        "urdf",
        "robot.urdf"
    )

    with open(urdf_path, "r") as file:
        robot_description = file.read()


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_description,
        }],
    )

    return LaunchDescription([
        joint_state_broadcaster,
        diff_drive_controller,
        odometry,
        robot_state_publisher
    ])
