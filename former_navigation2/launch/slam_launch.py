#!/usr/bin/env python3

# Author: Brighten Lee

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    slam_config_dir = LaunchConfiguration(
        "slam_config_dir",
        default=os.path.join(
            get_package_share_directory("former_navigation2"),
            "config",
            "mapper_params_lifelong.yaml",
        ),
    )

    return LaunchDescription(
        [
            Node(
                package="slam_toolbox",
                executable="lifelong_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_config_dir, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
