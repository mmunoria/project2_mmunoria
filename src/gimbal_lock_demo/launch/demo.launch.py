# gimbal_lock_demo/launch/demo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gimbal_lock_demo")

    urdf_xacro_path = os.path.join(pkg_share, "urdf", "gimbalrobot.urdf.xacro")
    rviz_config_path = os.path.join(pkg_share, "rviz", "demo.rviz")

    # Run xacro to produce the robot_description
    robot_description = Command(["xacro ", urdf_xacro_path])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            rviz,
        ]
    )
