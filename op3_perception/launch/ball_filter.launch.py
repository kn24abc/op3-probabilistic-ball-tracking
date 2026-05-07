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
                parameters=[{
                    # Only apply range likelihood when head is >20 deg down
                    # (ball estimated at <=1.5 m) — avoids collapsing on far balls.
                    'min_tilt_for_range_rad': 0.35,
                    # Looser range sigma so far-ball geometry errors don't kill weights.
                    'sigma_range_m': 1.0,
                }],
            ),
        ]
    )
