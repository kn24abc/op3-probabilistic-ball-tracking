"""Launches the goalpost filter node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="op3_perception",
                executable="goalpost_filter_node",
                name="goalpost_filter",
                output="screen",
                parameters=[{}],
            )
        ]
    )
