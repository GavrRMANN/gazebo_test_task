import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)


def generate_launch_description():

    navigation_share = get_package_share_directory(
        "autonomy_navigation"
    )

    slam_toolbox_share = get_package_share_directory(
        "slam_toolbox"
    )

    slam_params_file = os.path.join(
        navigation_share,
        "config",
        "slam.yaml",
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_share,
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": slam_params_file,
        }.items(),
    )

    return LaunchDescription([
        slam,
    ])