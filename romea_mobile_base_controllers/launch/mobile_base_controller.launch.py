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

    base_description_yaml_filename = LaunchConfiguration(
        "base_description_yaml_filename"
    ).perform(context)

    base_controller_yaml_filename = LaunchConfiguration(
        "base_controller_yaml_filename"
    ).perform(context)

    controller_yaml_filename = "/tmp/"+joints_prefix+"_base_controller.yaml"

    with open(base_description_yaml_filename, "r") as f:
        base_info = yaml.load(f, Loader=yaml.FullLoader)

    with open(base_controller_yaml_filename, "r") as f:
        base_controller_root = yaml.load(f, Loader=yaml.FullLoader)
        base_controller_node = base_controller_root["/**"]
        base_controller_ros_params = base_controller_node["ros__parameters"]
        base_controller_ros_params["base_info"] = base_info
        base_controller_ros_params["controller"]["joints_prefix"] = joints_prefix

    with open(controller_yaml_filename, "w") as f:
        yaml.dump(base_controller_root, f)

    mobile_base_controller = Node(
        package="romea_mobile_base_controllers",
        # package="controller_manager",
        # executable="spawner",
        executable="spawner.py",
        exec_name="mobile_base_controller_spawner",
        arguments=[
            controller_name,
            "--param-file",
            controller_yaml_filename,
            "--controller-manager",
            controller_manager_name,
        ],
        # output="screen",
    )

    joint_state_broadcaster = Node(
        package="romea_mobile_base_controllers",
        executable="spawner.py",
        exec_name="joint_state_broadcaster_spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
        ],
        # output="screen",
    )

    return [joint_state_broadcaster, mobile_base_controller]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("controller_manager_name"))

    declared_arguments.append(DeclareLaunchArgument("controller_name"))

    declared_arguments.append(DeclareLaunchArgument("base_description_yaml_filename"))

    declared_arguments.append(DeclareLaunchArgument("base_controller_yaml_filename"))

    declared_arguments.append(DeclareLaunchArgument("joints_prefix", default_value=""))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
