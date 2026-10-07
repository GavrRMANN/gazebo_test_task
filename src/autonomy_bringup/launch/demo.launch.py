import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory('autonomy_navigation')
    gazebo_share = get_package_share_directory('autonomy_gazebo')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'simulation.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
    )
    ready = Node(
        package='autonomy_navigation', executable='simulation_ready',
        name='simulation_ready', output='screen', parameters=[{'use_sim_time': True}],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
    )
    mission = Node(
        package='autonomy_navigation', executable='waypoint_mission',
        name='waypoint_mission', output='screen',
        condition=IfCondition(LaunchConfiguration('mission')),
        parameters=[{'use_sim_time': True, 'waypoints_file': LaunchConfiguration('waypoints_file')}],
    )

    def start_navigation(event, context):
        if event.returncode == 0:
            return [navigation, mission]
        return [EmitEvent(event=Shutdown(reason='Simulation did not become ready'))]

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('mission', default_value='true'),
        DeclareLaunchArgument('waypoints_file', default_value=os.path.join(
            navigation_share, 'config', 'waypoints.yaml')),
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=start_navigation)),
        simulation,
        ready,
    ])
