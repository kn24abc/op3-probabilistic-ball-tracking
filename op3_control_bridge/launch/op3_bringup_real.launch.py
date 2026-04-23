"""
op3_bringup_real.launch.py
  bring up OP3 hardware considering safe ordering:
  - op3_manager (dynamixel, open_cr, IMU)
  - usb_cam camera publisher: /usb_cam_node/image_raw
  - op3_control_bridge
  - op3_vision_darknet detector: /usb_cam_node/image_raw
  - optional - detection_visualizer
  - TODO behavior tree
  - TODO game controller hl client

  ros2 launch op3_control_bridge op3_bringup_real.launch.py \
  enable_detection_visualizer:=True \
  detector_config_name:=yolo-v7-tiny

  


"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import (
  PythonLaunchDescriptionSource,
  FrontendLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  ld = LaunchDescription()

  # Robot description (used by robot_state_publisher / TF)
  op3_description_share = get_package_share_directory('op3_description')
  op3_urdf_path = PathJoinSubstitution([op3_description_share, 'urdf', 'robotis_op3.urdf.xacro'])
  robot_description_content = ParameterValue(Command(['xacro ', op3_urdf_path]), value_type=str)

  joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    parameters=[{'source_list': ['/robotis/present_joint_states']}],
    remappings=[('/joint_states', '/robotis/present_joint_states')],
    output='screen'
  )

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    parameters=[{'robot_description': robot_description_content}],
    remappings=[('/joint_states', '/robotis/present_joint_states')],
    output='screen'
  )

  # op3_manager (real hardware mode)
  op3_manager_share = get_package_share_directory('op3_manager')
  op3_manager_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(op3_manager_share, 'launch', 'op3_manager.launch.py')
    )
  )

  # USB camera node (keeps topic as /usb_cam_node/image_raw) ---
  usb_cam_node = Node(
    package='usb_cam',
    executable='usb_cam_node_exe',
    name='usb_cam_node_exe',
    output='log',
    parameters=[{
    'video_device': '/dev/video0',
    'image_width': 640,
    'image_height': 360,
    'framerate': 10.0,
    'camera_frame_id': 'cam_link',
    'camera_name': 'camera',
    'io_method': 'mmap',
    'pixel_format': 'mjpeg2rgb',
    'av_device_format': 'YUV422P',
    }],
    remappings=[('image_raw', '/usb_cam_node/image_raw')]
  )

  # control bridge (defaults to OpenCR IMU) ---
  control_bridge_params = PathJoinSubstitution(
    [get_package_share_directory('op3_control_bridge'), 'config', 'control_bridge_params.yaml']
  )

  control_bridge_node = Node(
      package='op3_control_bridge',
      executable='control_bridge_node',
      name='control_bridge',
      output='screen',
      # arguments=['--ros-args', '--log-level','control_bridge:=debug'],
      parameters=[
      control_bridge_params,
      {
        "imu_topic": "/robotis/open_cr/imu"
      },
    ],
  )

  # op3 vision darknet detector include ---
  darknet_share = get_package_share_directory('op3_vision_darknet')
  detector_config_name = LaunchConfiguration("detector_config_name")
  declare_detector_config_name = DeclareLaunchArgument(
    "detector_config_name",
    default_value="reality-v0.2",
    description="Subfolder in op3_vision_darknet/config containing params.yaml",
  )
  detector_parameters = LaunchConfiguration("detector_parameters")
  declare_detector_parameters = DeclareLaunchArgument(
      "detector_parameters",
      default_value=PathJoinSubstitution(
          [darknet_share, "config", detector_config_name, "params.yaml"]
      ),
      description="Path to the detector params.yaml",
  )

  detector_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          PathJoinSubstitution([darknet_share, "launch", "detector.launch.py"])
      ),
      launch_arguments={
          "detector_parameters": detector_parameters,
          "rgb_image": "/usb_cam_node/image_raw",
      }.items(),
  )

  # perception filters (ball + goalpost)
  op3_perception_share = get_package_share_directory('op3_perception')
  ball_filter_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(op3_perception_share, 'launch', 'ball_filter.launch.py')
    )
  )
#  goalpost_filter_launch = IncludeLaunchDescription(
#    PythonLaunchDescriptionSource(
#      os.path.join(op3_perception_share, 'launch', 'goalpost_filter.launch.py')
#    )
#  )

  # optional nodes
  # detection visualizer
  enable_visualizer = LaunchConfiguration("enable_detection_visualizer")

  ld.add_action(
    DeclareLaunchArgument(
      "enable_detection_visualizer",
      default_value="False",
      description="Start detection_visualizer"
    )
  )

  control_bridge_start_delay = LaunchConfiguration("control_bridge_start_delay")
  ld.add_action(
    DeclareLaunchArgument(
      "control_bridge_start_delay",
      default_value="5.0",
      description="Seconds to wait after op3_manager before starting control_bridge"
    )
  )

  visualizer_node = Node(
    package="detection_visualizer",
    executable="detection_visualizer",
    name="detection_visualizer",
    output="screen",
    remappings=[
      ("/camera/image_raw", "/usb_cam_node/image_raw"),
    ],
    parameters=[
      {},
    ],
    condition=IfCondition(enable_visualizer)
  )

  rqt_predictions_view = Node(
    package="rqt_image_view",
    executable="rqt_image_view",
    name="rqt_dbg_images",
    arguments=["/dbg_images"],
    condition=IfCondition(enable_visualizer),
)

  # game_controller_hl (team/bot configurable)
  team_id = LaunchConfiguration("team_id")
  bot_id = LaunchConfiguration("bot_id")
  ld.add_action(
    DeclareLaunchArgument(
      "team_id",
      default_value="33",
      description="Game Controller team ID"
    )
  )
  ld.add_action(
    DeclareLaunchArgument(
      "bot_id",
      default_value="3",
      description="Game Controller robot ID"
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
  
  ld.add_action(op3_manager_launch)
  ld.add_action(joint_state_publisher)
  ld.add_action(robot_state_publisher)
  ld.add_action(usb_cam_node)
  ld.add_action(
    TimerAction(
      period=control_bridge_start_delay,
      actions=[control_bridge_node],
    )
  )
  ld.add_action(declare_detector_config_name)
  ld.add_action(declare_detector_parameters)
  ld.add_action(detector_launch)
  ld.add_action(ball_filter_launch)
  #ld.add_action(goalpost_filter_launch)
  ld.add_action(game_controller_launch)

  # optional elements
  ld.add_action(visualizer_node)  
  ld.add_action(rqt_predictions_view)

  return ld

  
