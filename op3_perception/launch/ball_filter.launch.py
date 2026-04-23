"""Launches the ball filter node and ball particle filter node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package='op3_perception',
                executable='ball_filter_node',
                name='ball_filter',
                output='screen',
                parameters=[{
                    'bearing_transform.target_frame': 'body_link',
                    'camera_frame': 'cam_link',
                }],
            ),
            Node(
                package='op3_perception',
                executable='ball_particle_filter_node',
                name='ball_particle_filter_node',
                output='screen',
            ),
        ]
    )
