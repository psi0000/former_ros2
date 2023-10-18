#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="sick_scan2",
                executable="sick_generic_caller",
                name="sick_scan2_node",
                output="screen",
                parameters=[
                    {
                        "scanner_name": "sick_tim_5xx",
                        "hostname": "192.168.10.11",
                        "port": 2112,
                        "frame_id": "laser_link",
                        "min_ang": -1.74533,
                        "max_ang": 1.74533,
                    }
                ],
            ),
        ]
    )
