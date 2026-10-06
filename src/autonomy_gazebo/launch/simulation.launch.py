import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import Command

from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    gazebo_share = get_package_share_directory(
        "gazebo_ros"
    )

    gazebo_project_share = get_package_share_directory(
        "autonomy_gazebo"
    )

    description_share = get_package_share_directory(
        "autonomy_description"
    )

    world_path = os.path.join(
        gazebo_project_share,
        "worlds",
        "test_world.world",
    )

    robot_xacro = os.path.join(
        description_share,
        "urdf",
        "robot.urdf.xacro",
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            robot_xacro,
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share,
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world": world_path,
            "verbose": "true",
        }.items(),
    )

    robot_state_publisher = Node(

        package="robot_state_publisher",

        executable="robot_state_publisher",

        name="robot_state_publisher",

        output="screen",

        parameters=[
            {
                "robot_description":
                    robot_description,

                "use_sim_time":
                    True,
            }
        ],
    )

    spawn_robot = Node(

        package="gazebo_ros",

        executable="spawn_entity.py",

        name="spawn_autonomy_robot",

        output="screen",

        arguments=[
            "-entity",
            "autonomy_robot",

            "-topic",
            "robot_description",

            "-x",
            "-4.0",

            "-y",
            "-3.0",

            "-z",
            "0.001",

            "-Y",
            "0.0",
        ],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])