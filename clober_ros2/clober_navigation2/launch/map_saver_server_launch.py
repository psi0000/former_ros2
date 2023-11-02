#!/usr/bin/env python3

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Parameters
    lifecycle_nodes = ["map_saver"]
    use_sim_time = false
    autostart = True
    save_map_timeout = 10000
    free_thresh_default = 0.25
    occupied_thresh_default = 0.65

    start_map_saver_server_cmd = launch_ros.actions.Node(
        package="nav2_map_server",
        executable="map_saver_server",
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {"save_map_timeout": save_map_timeout},
            {"free_thresh_default": free_thresh_default},
            {"occupied_thresh_default": occupied_thresh_default},
        ],
    )

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    ld = LaunchDescription()

    ld.add_action(start_map_saver_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
