#!/usr/bin/env python3

# Author: Brighten Lee

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    auto_dock_launch_file_dir = os.path.join(
        get_package_share_directory("roas_dock"), "launch"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [auto_dock_launch_file_dir, "/auto_dock_launch.py"]
                )
            ),
        ]
    )
