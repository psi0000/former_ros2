#!/usr/bin/env python3

# Author: Brighten Lee

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    robot_config_dir = LaunchConfiguration(
        "robot_config_dir",
        default=os.path.join(
            get_package_share_directory("clober_base"), "config", "clober.yaml"
        ),
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("clober_description"),
                    "urdf",
                    "clober.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    teleop_launch_file_dir = os.path.join(
        get_package_share_directory("clober_teleop"), "launch"
    )
    sick_tim_launch_file_dir = os.path.join(
        get_package_share_directory("clober_base"), "launch/include"
    )
    d435_launch_file_dir = os.path.join(
        get_package_share_directory("clober_base"), "launch/include"
    )
    imu_launch_file_dir = os.path.join(
        get_package_share_directory("clober_base"), "launch/include"
    )

    return LaunchDescription(
        [
            # Node(
            #     package="clober_base",
            #     executable="clober_gpio_node",
            #     name="clober_gpio_node",
            #     output="screen",
            #     parameters=[robot_config_dir],
            # ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="clober_base",
                executable="clober_base_node",
                output="screen",
                parameters=[robot_description, robot_config_dir],
            ),
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["clober_base_controller"],
                output="screen",
            ),
            # Node(
            #     package="robot_localization",
            #     executable="ekf_node",
            #     name="ekf_node",
            #     output="screen",
            #     parameters=[robot_config_dir],
            # ),
            Node(
                package="twist_mux",
                executable="twist_mux",
                output="screen",
                parameters=[robot_config_dir],
                remappings={
                    ("/cmd_vel_out", "/clober_base_controller/cmd_vel_unstamped")
                },
            ),
            Node(
                package="twist_mux",
                executable="twist_marker",
                output="screen",
                parameters=[
                    {"frame_id": "base_link", "scale": 1.0, "vertical_position": 2.0}
                ],
                remappings={("/twist", "/clober_base_controller/cmd_vel_unstamped")},
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [teleop_launch_file_dir, "/teleop_launch.py"]
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [sick_tim_launch_file_dir, "/sick_tim_launch.py"]
                ),
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         [d435_launch_file_dir, "/d435_launch.py"]
            #     ),
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([imu_launch_file_dir, "/mi_launch.py"]),
            ),
        ]
    )
