#!/usr/bin/env python3
#
# Copyright 2022 CLOBOT Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from threading import local

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    nav_dir = get_package_share_directory('clober_navigation')
    launch_dir = os.path.join(nav_dir,'launch')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_topic = LaunchConfiguration('map_topic')
    scan_topic = LaunchConfiguration('scan_topic')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='clober_1',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_map_topic_cmd = DeclareLaunchArgument(
        'map_topic',
        default_value='/clober_1/map',
        description='map topic')

    declare_scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic',
        default_value='/clober_1/scan',
        description='scan topic')

    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

                            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                            'map_subscribe_transient_local': 'true',
                            'map': map_topic,
                            'scan': scan_topic}.items()),
                            
    ])


    # # Create the launch description and populate
    ld = LaunchDescription()

    # # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_topic_cmd)
    ld.add_action(declare_scan_topic_cmd)
    # # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld