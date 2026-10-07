import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory('autonomy_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(navigation_share, 'config', 'nav2_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',
                'map': os.path.join(navigation_share, 'maps', 'test_map.yaml'),
                'params_file': params_file,
                'use_sim_time': 'True',
                'autostart': 'True',
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items(),
        ),
        Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', output='screen',
            parameters=[params_file, {'use_sim_time': True}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor', output='screen',
            parameters=[{'use_sim_time': True, 'autostart': True,
                         'node_names': ['collision_monitor']}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(navigation_share, 'rviz', 'navigation.rviz')],
            parameters=[{'use_sim_time': True}],
        ),
    ])
