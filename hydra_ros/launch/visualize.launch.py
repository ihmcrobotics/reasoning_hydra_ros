#!/usr/bin/env python3
"""Start RViz2 with the Alex Hydra visualization configuration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rviz_config = (
        Path(get_package_share_directory("hydra_ros")) / "rviz" / "alex.rviz"
    )
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="hydra_visualizer",
                output="screen",
                arguments=["-d", str(rviz_config)],
            )
        ]
    )
