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

    controller_name = LaunchConfiguration(
        "controller_name"
    ).perform(context)

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
    config["/**"]={}
    config["/**"]["ros__parameters"]={}

    params = config["/**"]["ros__parameters"];
    params["update_rate"] = base_controller_ros_params["update_rate"]

    params["left_wheel_names"] = [
        joints_prefix + base_info["joints"]["front_left_wheel_spinning_joint_name"],
        joints_prefix + base_info["joints"]["rear_left_wheel_spinning_joint_name"],
    ]

    params["right_wheel_names"] = [
        joints_prefix + base_info["joints"]["front_right_wheel_spinning_joint_name"],
        joints_prefix + base_info["joints"]["rear_right_wheel_spinning_joint_name"],
    ]

    assert (
        base_info["geometry"]["front_axle"]["wheels_distance"]
        == base_info["geometry"]["rear_axle"]["wheels_distance"]
    )
    params["wheel_separation"] = base_info["geometry"]["front_axle"]["wheels_distance"]

    assert (
        base_info["geometry"]["front_axle"]["wheels"]["radius"]
        == base_info["geometry"]["rear_axle"]["wheels"]["radius"]
    )
    params["wheel_radius"] = base_info["geometry"]["front_axle"]["wheels"]["radius"]

    params["wheels_per_side"] = 2
    params["wheel_separation_multiplier"] = 1.0
    params["left_wheel_radius_multiplier"] = 1.0
    params["right_wheel_radius_multiplier"]: 1.0

    params["publish_rate"]: base_controller["publish_rate"]
    params["odom_frame_id"]: base_controller["odom_frame_id"]
    params["enable_odom_tf"] = base_controller["enable_odom_tf"]
    params["cmd_vel_timeout"] = base_controller["timeout"]
    params["open_loop"] = True
    params["position_feedback"] = False
    params["use_stamped_vel"] = False

    params["pose_covariance_diagonal"] = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
    params["twist_covariance_diagonal"] = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]

    wheel_speed_control_info = base_info["wheels_speed_control"]
    wheel_speed_command_info = wheel_speed_control_info["command"]
    maximal_wheel_speed = wheel_speed_command_info["maximal_speed"]
    maximal_wheel_acceleration = wheel_speed_command_info["maximal_acceleration"]

    maximal_longitudinal_speed = maximal_wheel_speed
    minimal_longitudinal_speed = - maximal_wheel_speed
    maximal_longitudinal_acceleration = maximal_wheel_acceleration
    maximal_angular_speed = 2 * maximal_wheel_speed / params["wheel_separation"]
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
        maximal_longitudinal_speed, user_maximal_longitudinal_speed
    )

    maximal_angular_speed = min(
        maximal_angular_speed, user_maximal_angular_speed
    )

    params["linear"]={}
    params["linear"]["x"]={}
    params["linear"]["x"]["has_velocity_limits"] = True
    params["linear"]["x"]["has_acceleration_limits"] = True
    params["linear"]["x"]["has_jerk_limits"] = False
    params["linear"]["x"]["max_velocity"] = maximal_longitudinal_speed
    params["linear"]["x"]["min_velocity"] = minimal_longitudinal_speed
    params["linear"]["x"]["max_acceleration"]: maximal_longitudinal_acceleration
    params["linear"]["x"]["min_acceleration"]: -maximal_longitudinal_acceleration
    params["linear"]["x"]["max_jerk"]: 0.0
    params["linear"]["x"]["min_jerk"]: 0.0

    params["angular"]={}
    params["angular"]["z"]={}
    params["angular"]["z"]["has_velocity_limits"] = True
    params["angular"]["z"]["has_acceleration_limits"] = False
    params["angular"]["z"]["has_jerk_limits"] = False
    params["angular"]["z"]["max_velocity"] = maximal_angular_speed
    params["angular"]["z"]["min_velocity"] = -maximal_angular_speed
    params["angular"]["z"]["max_acceleration"]: maximal_angular_acceleration
    params["angular"]["z"]["min_acceleration"]: -maximal_angular_acceleration
    params["angular"]["z"]["max_jerk"]: 0.0
    params["angular"]["z"]["min_jerk"]: 0.0

    with open(controller_yaml_filename, "w") as f:
        yaml.dump(config, f)

    mobile_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
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
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name],
        output="screen",
    )

    return [mobile_base_controller, joint_state_broadcaster]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("controller_name"))

    declared_arguments.append(DeclareLaunchArgument("controller_manager_name"))

    declared_arguments.append(DeclareLaunchArgument("base_description_yaml_filename"))

    declared_arguments.append(DeclareLaunchArgument("base_controller_yaml_filename"))

    declared_arguments.append(DeclareLaunchArgument("joints_prefix", default_value=""))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
