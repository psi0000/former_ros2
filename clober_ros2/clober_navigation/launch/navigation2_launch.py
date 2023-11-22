#!/usr/bin/env python3

# Author: Brighten Lee

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    autostart = LaunchConfiguration("autostart", default="true")
    
    clober_bringup_launch_file_dir = os.path.join(
        get_package_share_directory("clober_base"), "launch"
    )
    clober_description_launch_file_dir = os.path.join(
        get_package_share_directory("clober_description"), "launch"
    )
    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("clober_navigation"),
            "map", 
            "gz_map.yaml"
        ),
    )
    nav2_config_dir = LaunchConfiguration(
        "nav2_config_dir",
        default=os.path.join(
            get_package_share_directory("clober_navigation"),
            "param",
            "clober_gz.yaml",
        ),
    )

    nav2_launch_dir = os.path.join(
        get_package_share_directory("clober_navigation2"), "launch"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [clober_bringup_launch_file_dir, "/base_launch.py"]
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [clober_description_launch_file_dir, "/description_launch.py"]
                )
            ),

            # rviz2
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [clober_description_launch_file_dir, "/rviz2_launch.py"]
                )
            ),

            # navigation2 bringup 
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_dir, "/bringup_launch.py"]),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "map": map_dir,
                    "params_file": nav2_config_dir,
                }.items(),
            ),
            
            
        ]
    )
