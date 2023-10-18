#!/usr/bin/env python3

# Author: Brighten Lee

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_model = "ps5"
    joy_config_dir = LaunchConfiguration(
        "joy_config_dir",
        default=os.path.join(
            get_package_share_directory("former_teleop"), "config", joy_model, ".yaml"
        ),
    )

    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                # parameters=[{"device_name": joy_model}],
            ),
            Node(
                package="former_teleop",
                executable="former_teleop_node",
                name="former_teleop_node",
                output="screen",
                parameters=[joy_config_dir],
                remappings={("/cmd_vel", "/teleop/cmd_vel")},
            ),
        ]
    )
