import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time',default='true')
    open_rviz = LaunchConfiguration('open_rviz',default='true')
    rviz_config_dir = os.path.join(get_package_share_directory('clober_navigation'),'rviz','navigation.rviz')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/initialpose','initialpose'),
                  ('/map', 'map'),
                  ('/scan', 'scan'),
                  ('/global_costmap/costmap', 'global_costmap/costmap'),
                  ('/local_costmap/costmap', 'local_costmap/costmap'),
                  ('/plan', 'plan'),
                  ('/local_plan', 'local_plan'),
                  ('/global_costmap/published_footprint', 'global_costmap/published_footprint')]

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='use simulation time'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_rb0',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time':use_sim_time}],
            remappings=remappings,
            condition= IfCondition(open_rviz)
        )

    ])