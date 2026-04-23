from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    image_topic = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/image_raw",
        description="Raw camera topic to draw overlays on (simulation)",
    )
    detection_topic = DeclareLaunchArgument(
        "detection_topic",
        default_value="/perception/ball",
        description="Detection topic to visualize",
    )
    overlay_topic = DeclareLaunchArgument(
        "overlay_topic",
        default_value="/camera/overlay",
        description="Topic where the annotated image is published",
    )
    detection_timeout = DeclareLaunchArgument(
        "detection_timeout",
        default_value="0.6",
        description="Seconds to retain a detection before dropping it from the overlay",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="ROS logger verbosity for op3_perception nodes",
    )

    log_level = LaunchConfiguration("log_level")

    op3_description_path = FindPackageShare("op3_description")
    op3_urdf_path = PathJoinSubstitution([op3_description_path, "urdf", "robotis_op3.urdf.xacro"])
    robot_description = ParameterValue(Command(["xacro ", op3_urdf_path]), value_type=str)

    op3_ros_args = [
        "--ros-args",
        "--log-level",
        log_level,
        "--log-level",
        "rcl:=INFO",
        "--log-level",
        "rclcpp:=INFO",
        "--log-level",
        "rmw_fastrtps_cpp:=INFO",
    ]

    ball_filter = Node(
        package="op3_perception",
        executable="ball_filter_node",
        name="ball_filter",
        output="screen",
        arguments=list(op3_ros_args),
        parameters=[{
            "bearing_transform.target_frame": "body_link",
            "camera_frame": "cam_link",
        }],
    )

    goalpost_filter = Node(
        package="op3_perception",
        executable="goalpost_filter_node",
        name="goalpost_filter",
        output="screen",
        arguments=list(op3_ros_args),
        parameters=[{"bearing_transform.target_frame": "body_link"}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        remappings=[("/joint_states", "/robotis/present_joint_states")],
    )

    overlay_node = Node(
        package="op3_perception",
        executable="vision_overlay_node",
        name="vision_overlay_node",
        output="screen",
        arguments=list(op3_ros_args),
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "detection_topic": LaunchConfiguration("detection_topic"),
                "overlay_topic": LaunchConfiguration("overlay_topic"),
                "detection_timeout": LaunchConfiguration("detection_timeout"),
            }
        ],
    )

    return LaunchDescription(
        [
            image_topic,
            detection_topic,
            overlay_topic,
            detection_timeout,
            log_level_arg,
            ball_filter,
            goalpost_filter,
            robot_state_publisher,
            overlay_node,
        ]
    )
