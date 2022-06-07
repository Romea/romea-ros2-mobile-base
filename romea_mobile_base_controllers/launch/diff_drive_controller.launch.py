from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import yaml
import sys


def launch_setup(context, *args, **kwargs):

    joints_prefix = LaunchConfiguration("joints_prefix").perform(context)

    controller_manager_name = LaunchConfiguration(
       "controller_manager_name"
    ).perform(context)

    base_description_yaml_filename = LaunchConfiguration(
        "base_description_yaml_filename"
    ).perform(context)

    base_controller_yaml_filename = LaunchConfiguration(
        "base_controller_yaml_filename"
    ).perform(context)

    controller_yaml_filename = "/tmp/controller.yaml"

    with open(base_description_yaml_filename, "r") as f:
        base_description_root = yaml.load(f, Loader=yaml.FullLoader)
        base_description_node = base_description_root["/**"]
        base_description_ros_params = base_description_node["ros__parameters"]
        base_info = base_description_ros_params["base_info"]

    with open(base_controller_yaml_filename, "r") as f:
        base_controller_root = yaml.load(f, Loader=yaml.FullLoader)
        base_controller_node = base_controller_root["/**"]
        base_controller_ros_params = base_controller_node["ros__parameters"]
        base_controller = base_controller_ros_params["controller"]

    config = {}

    config["left_wheel_names"] = [
        joints_prefix + base_info["joints"]["front_left_wheel_spinning_joint_name"],
        joints_prefix + base_info["joints"]["rear_left_wheel_spinning_joint_name"],
    ]

    config["left_wheel_names"] = [
        joints_prefix + base_info["joints"]["front_right_wheel_spinning_joint_name"],
        joints_prefix + base_info["joints"]["rear_right_wheel_spinning_joint_name"],
    ]

    assert (
        base_info["front_axle"]["wheels_distance"]
        == base_info["rear_axle"]["wheels_distance"]
    )
    config["wheel_separation"] = base_info["front_axle"]["wheels_distance"]

    assert (
        base_info["front_axle"]["wheels"]["radius"]
        == base_info["rear_axle"]["wheels"]["radius"]
    )
    config["wheel_radius"] = base_info["front_axle"]["wheels"]["radius"]

    config["wheels_per_side"] = 2
    config["wheel_separation_multiplier"] = 1.0
    config["left_wheel_radius_multiplier"] = 1.0
    config["right_wheel_radius_multiplier"]: 1.0

    config["publish_rate"]: base_controller["publish_rate"]
    config["odom_frame_id"]: base_controller["odom_frame_id"]
    config["enable_odom_tf"] = base_controller["enable_odom_tf"]
    config["cmd_vel_timeout"] = base_controller["timeout"]
    config["open_loop"] = True
    config["position_feedback"] = False
    config["use_stamped_vel"] = False

    config["pose_covariance_diagonal"] = [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    config["twist_covariance_diagonal"] = [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

    wheel_speed_control_info = base_info["wheels_speed_control"]
    wheel_speed_command_info = wheel_speed_control_info["command"]
    maximal_longitudinal_speed = wheel_command_info["maximal_speed"]
    minimal_longitudinal_speed = -wheel_command_info["maximal_speed"]
    maximal_longitudinal_acceleration = wheel_command_info["maximal_acceleration"]
    maximal_angular_speed = 2 * maximal_wheel_speed / config["wheel_separation"]
    maximal_angular_acceleration = 0.0  # TODO

    command_limits = base_controller.get("command_limits", {})
    user_minimal_longitudinal_speed = command_limits.get(
        "minimal_longitudinal_speed", -sys.float_info.max
    )
    user_maximal_longitudinal_speed = command_limits.get(
        "minimal_longitudinal_speed", sys.float_info.max
    )
    user_maximal_angular_speed = command_limits.get(
        "minimal_longitudinal_speed", sys.float_info.max
    )

    minimal_longitudinal_speed = max(
        minimal_longitudinal_speed, user_minimal_longitudinal_speed
    )

    maximal_longitudinal_speed = min(
        maximal_longitudinal_speed, user_maxiam_longitudinal_speed
    )

    maximal_angular_speed = max(
        maximal_longitudinal_speed, user_maximal_longitudinal_speed
    )

    config["linear"]["x"]["has_velocity_limits"] = True
    config["linear"]["x"]["has_acceleration_limits"] = True
    config["linear"]["x"]["has_jerk_limits"] = False
    config["linear"]["x"]["max_velocity"] = maximal_longitudinal_speed
    config["linear"]["x"]["min_velocity"] = minimal_longitudinal_speed
    config["linear"]["x"]["max_acceleration"]: maximal_longitudinal_acceleration
    config["linear"]["x"]["min_acceleration"]: -maximal_longitudinal_acceleration
    config["linear"]["x"]["max_jerk"]: 0.0
    config["linear"]["x"]["min_jerk"]: 0.0

    config["angular"]["x"]["has_velocity_limits"] = True
    config["angular"]["x"]["has_acceleration_limits"] = False
    config["angular"]["x"]["has_jerk_limits"] = False
    config["angular"]["x"]["max_velocity"] = maximal_angular_speed
    config["angular"]["x"]["min_velocity"] = -maximal_angular_speed
    config["angular"]["x"]["max_acceleration"]: maximal_angular_acceleration
    config["angular"]["x"]["min_acceleration"]: -maximal_angular_acceleration
    config["angular"]["x"]["max_jerk"]: 0.0
    config["angular"]["x"]["min_jerk"]: 0.0

    with open(controller_yaml_filename, "w") as f:
        yaml.dump(base_controller_root, f)

    mobile_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mobile_base_controller",
            "--param-file",
            controller_yaml_filename,
            "--controller-manager",
            controller_manager_name,
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "-c", controller_manager_name],
        output="screen",
    )

    return [mobile_base_controller, joint_state_broadcaster]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("controller_manager_name"))

    declared_arguments.append(DeclareLaunchArgument("base_description_yaml_filename"))

    declared_arguments.append(DeclareLaunchArgument("base_controller_yaml_filename"))

    declared_arguments.append(DeclareLaunchArgument("joints_prefix", default_value=""))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
