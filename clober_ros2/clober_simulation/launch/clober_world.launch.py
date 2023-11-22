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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'clober.model'
    # world = os.path.join(get_package_share_directory('clober_simulation'), 'worlds', world_file_name)

    # launch_file_dir = os.path.join(get_package_share_directory('clober_description'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')



    # gazebo launch #
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gazebo.launch.py'))
    )

    world_path = os.path.join(get_package_share_directory(
        'clober_simulation'), 'worlds', world_file_name)

    world_model_path = os.path.join(
        get_package_share_directory('clober_simulation'), 'models')
    robot_description_path = os.path.join(
        get_package_share_directory('clober_description'), 'launch')
    
    os.environ["GAZEBO_MODEL_PATH"] = world_model_path


    return LaunchDescription([

        ExecuteProcess(
            cmd=['gzserver', world_path , 
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                '--verbose'
                ],
            output='screen'),

        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-file' , world_model_path + '/clober/' + 'model.sdf',
                '-entity', 'clober', '-spawn_service_timeout', '300', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
            output='screen'),    
        


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                robot_description_path, '/gazebo_description_launch.py'
            ]), launch_arguments={'use_sim_time': use_sim_time}.items(),
        )
    
    ])