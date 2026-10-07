from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf_file", default_value="", description="Robot URDF/xacro filename"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix", default_value='robot_', description="Prefix of the joint names."
        )
    )

    # Optional logging of robot description, enabled by default
    declared_arguments.append(
        DeclareLaunchArgument(
            "print_urdf", default_value="false", description="Output the robot_description."
        )
    )

    # Initialize Arguments
    urdf_file = LaunchConfiguration("urdf_file")
    prefix = LaunchConfiguration("prefix")
    log_robot_description = LaunchConfiguration("print_urdf")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file,
            " ",
            "prefix:=",
            prefix,
            " mode:=view",
        ],
        on_stderr="ignore",
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content)}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("romea_mobile_base_description"), "config", "urdf.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui"
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    robot_description_log = LogInfo(
        msg=robot_description_content, 
        condition=IfCondition(log_robot_description)
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        robot_description_log,
    ]

    return LaunchDescription(declared_arguments + nodes)
