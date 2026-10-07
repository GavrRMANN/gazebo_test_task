from glob import glob
import os

from setuptools import find_packages, setup


package_name = "autonomy_navigation"


setup(
    name=package_name,
    version="0.1.0",

    packages=find_packages(),
    py_modules=["waypoint_mission", "gps_coordinates", "simulation_ready"],

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [
                "resource/" + package_name,
            ],
        ),

        (
            "share/" + package_name,
            [
                "package.xml",
            ],
        ),

        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.launch.py"),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob("config/*.yaml"),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "rviz",
            ),
            glob("rviz/*.rviz"),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "maps",
            ),
            glob("maps/*"),
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="Roman Gavrilov",

    description=(
        "Navigation and waypoint mission package."
    ),

    license="Proprietary",

    entry_points={
        "console_scripts": [
            "waypoint_mission = waypoint_mission:main",
            "simulation_ready = simulation_ready:main",
        ],
    },
)
