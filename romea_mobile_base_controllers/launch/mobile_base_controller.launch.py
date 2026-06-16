from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def launch_setup(context, *args, **kwargs):

    joints_prefix = LaunchConfiguration("joints_prefix").perform(context)

    controller_name = LaunchConfiguration("controller_name").perform(context)

    controller_manager_name = LaunchConfiguration("controller_manager_name").perform(
        context
    )

    base_configuration_file_path = LaunchConfiguration(
        "base_configuration_file_path"
    ).perform(context)

    base_controller_configuration_file_path = LaunchConfiguration(
        "base_controller_configuration_file_path"
    ).perform(context)

    controller_yaml_filename = "/tmp/"+joints_prefix+"base_controller.yaml"

    with open(base_configuration_file_path, "r") as f:
        base_info = yaml.load(f, Loader=yaml.FullLoader)

    with open(base_controller_configuration_file_path, "r") as f:
        base_controller_root = yaml.load(f, Loader=yaml.FullLoader)
        base_controller_node = base_controller_root["/**"]
        base_controller_ros_params = base_controller_node["ros__parameters"]
        base_controller_ros_params["base_info"] = base_info
        base_controller_ros_params["controller"]["joints_prefix"] = joints_prefix

    with open(controller_yaml_filename, "w") as f:
        yaml.dump(base_controller_root, f)

    mobile_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        exec_name="mobile_base_controller_spawner",
        arguments=[
            controller_name,
            "--param-file",
            controller_yaml_filename,
            "--controller-manager",
            controller_manager_name,
            "--switch-timeout",
            "15"
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        exec_name="joint_state_broadcaster_spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--switch-timeout",
            "15"
        ],
        # output="screen",
    )

    return [joint_state_broadcaster, mobile_base_controller]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("controller_manager_name", default_value="controller_manager"),
            DeclareLaunchArgument("controller_name"),
            DeclareLaunchArgument("base_configuration_file_path"),
            DeclareLaunchArgument("base_controller_configuration_file_path"),
            DeclareLaunchArgument("joints_prefix", default_value=""),
            OpaqueFunction(function=launch_setup)
        ]
    )
