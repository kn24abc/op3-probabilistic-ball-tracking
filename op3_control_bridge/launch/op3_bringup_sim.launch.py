"""
op3_bringup_sim.launch.py

Step-by-step bring-up of the OP3 Webots simulation:
  1) Launch Webots + op3_manager (provides /camera/image_raw, /robotis_op3/imu)
  2) Start the darknet detector (consumes the camera stream)
  3) Run the perception filters (ball + goalpost)
  4) Start the control bridge
  5) Optionally start the simulated Game Controller

Detection visualizer/RQt utilities were removed to keep the launch lean and to
match the manual bring-up flow documented in operations/bringup/op3_bringup_sim.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import (FrontendLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    # Robot description (robot_state_publisher)
    op3_description_share = get_package_share_directory("op3_description")
    op3_urdf_path = PathJoinSubstitution([op3_description_share, "urdf", "robotis_op3.urdf.xacro"])
    robot_description_content = ParameterValue(Command(['xacro ', op3_urdf_path]), value_type=str)

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{'source_list': ['/robotis/present_joint_states']}],
        remappings=[('/joint_states', '/robotis/present_joint_states')],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{'robot_description': robot_description_content}],
        remappings=[('/joint_states', '/robotis/present_joint_states')],
        output="screen",
    )

    # Webots + OP3 manager (simulation mode)
    op3_webots_share = get_package_share_directory("op3_webots")
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(op3_webots_share, "launch", "robot_manager.launch.py")
        )
    )

    # Detector configuration
    darknet_share = get_package_share_directory("op3_vision_darknet")
    detector_config_name = LaunchConfiguration("detector_config_name")
    ld.add_action(
        DeclareLaunchArgument(
    "detector_config_name",
    default_value="simulation-v0.2",
            description="Subfolder in op3_vision_darknet/config containing params.yaml",
        )
    )

    detector_parameters = LaunchConfiguration("detector_parameters")
    ld.add_action(
        DeclareLaunchArgument(
            "detector_parameters",
            default_value=PathJoinSubstitution(
                [darknet_share, "config", detector_config_name, "params.yaml"]
            ),
            description="Path to the detector params.yaml",
        )
    )

    detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([darknet_share, "launch", "detector.launch.py"])
        ),
        launch_arguments={
            "detector_parameters": detector_parameters,
            "rgb_image": "/camera/image_raw",
        }.items(),
    )

    # game_controller_hl (team/bot configurable)
    team_id = LaunchConfiguration("team_id")
    bot_id = LaunchConfiguration("bot_id")
    ld.add_action(
        DeclareLaunchArgument(
            "team_id",
            default_value="33",
            description="Game Controller team ID",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "bot_id",
            default_value="3",
            description="Game Controller robot ID",
        )
    )
    game_controller_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("game_controller_hl"), "launch", "game_controller.launch"]
            )
        ),
        launch_arguments={
            "team_id": team_id,
            "bot_id": bot_id,
        }.items(),
    )

    # Perception filters (ball + goalpost)
    op3_perception_share = get_package_share_directory("op3_perception")
    ball_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(op3_perception_share, "launch", "ball_filter.launch.py")
        )
    )
    goalpost_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(op3_perception_share, "launch", "goalpost_filter.launch.py")
        )
    )

    # Control bridge parameters (ball tracker config lives in share/op3_control_bridge/config)
    control_bridge_params = PathJoinSubstitution(
        [get_package_share_directory("op3_control_bridge"), "config", "control_bridge_params.yaml"]
    )

    control_bridge_node = Node(
        package='op3_control_bridge',
        executable='control_bridge_node',
        name='control_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', 'control_bridge:=debug'],
        parameters=[
            control_bridge_params,
            {
                "imu_topic": "/robotis_op3/imu",
            },
        ],
    )

    # Build launch description in the manual bring-up order
    ld.add_action(webots_launch)          # Step 1
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(
        TimerAction(
            period=3.0,
            actions=[detector_launch],
        )
    )                                     # Step 2 (delayed)
    ld.add_action(
        TimerAction(
            period=5.0,
            actions=[ball_filter_launch, goalpost_filter_launch],
        )
    )                                     # Step 3
    ld.add_action(
        TimerAction(
            period=7.0,
            actions=[control_bridge_node],
        )
    )                                     # Step 4
    # ld.add_action(game_controller_launch) # Step 5 (disabled for sim testing)

    return ld
