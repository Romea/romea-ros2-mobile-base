# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys
import math
import yaml

from ament_index_python.packages import get_package_share_directory

from romea_mobile_base_description import (
    get_track,
    get_wheelbase,
    get_maximal_linear_speed,
    get_maximal_wheel_angle,
    get_maximal_steering_angle,
    get_maximal_angular_speed,
    get_kinematic_type,
    get_command_type,
)

from romea_joystick_utils import apply_joystick_remapping


def skid_steering_teleop_cmd_range_clamp_(
    maximal_linear_speed, maximal_angular_speed, user_cmd_range
):
    cmd_range = {}
    cmd_range["maximal_linear_speed"] = {}
    cmd_range["maximal_linear_speed"]["slow_mode"] = min(
        user_cmd_range["maximal_linear_speed"].get("slow_mode", sys.float_info.max),
        maximal_linear_speed,
    )

    cmd_range["maximal_linear_speed"]["turbo_mode"] = min(
        user_cmd_range["maximal_linear_speed"].get("turbo_mode", sys.float_info.max),
        maximal_linear_speed,
    )

    cmd_range["maximal_angular_speed"] = {}

    if "maximal_angular_speed" in user_cmd_range:
        cmd_range["maximal_angular_speed"]["slow_mode"] = min(
            user_cmd_range["maximal_angular_speed"].get("slow_mode", sys.float_info.max),
            maximal_angular_speed,
        )
        cmd_range["maximal_angular_speed"]["turbo_mode"] = min(
            user_cmd_range["maximal_angular_speed"].get("turbo_mode", sys.float_info.max),
            maximal_angular_speed,
        )
    else:
        cmd_range["maximal_angular_speed"]["slow_mode"] = (
            maximal_angular_speed
            / maximal_linear_speed
            * user_cmd_range["maximal_linear_speed"]["slow_mode"]
        )
        cmd_range["maximal_angular_speed"]["turbo_mode"] = (
            maximal_angular_speed
            / maximal_linear_speed
            * user_cmd_range["maximal_linear_speed"]["turbo_mode"]
        )

    return cmd_range


def skid_steering_teleop_cmd_range_clamp(mobile_base_configuration, user_cmd_range):

    # track = get_track(mobile_base_configuration)
    # maximal_linear_speed = get_maximal_linear_speed(mobile_base_configuration)
    # maximal_angular_speed = 2 * maximal_linear_speed / track

    maximal_linear_speed = get_maximal_linear_speed(mobile_base_configuration)
    maximal_angular_speed = get_maximal_angular_speed(mobile_base_configuration)

    return skid_steering_teleop_cmd_range_clamp_(
        maximal_linear_speed, maximal_angular_speed, user_cmd_range
    )


def omni_steering_teleop_cmd_range_clamp_(
    maximal_linear_speed, maximal_angular_speed, user_cmd_range
):
    cmd_range = skid_steering_teleop_cmd_range_clamp_(
        maximal_linear_speed, maximal_angular_speed, user_cmd_range
    )

    cmd_range["maximal_lateral_speed"] = {}
    cmd_range["maximal_lateral_speed"]["slow_mode"] = min(
        user_cmd_range["maximal_lateral_speed"].get("slow_mode", sys.float_info.max),
        maximal_linear_speed,
    )
    cmd_range["maximal_lateral_speed"]["turbo_mode"] = min(
        user_cmd_range["maximal_lateral_speed"].get("turbo_mode", sys.float_info.max),
        maximal_linear_speed,
    )

    return cmd_range


def omni_steering_teleop_cmd_range_clamp(mobile_base_configuration, user_cmd_range):

    # track = get_track(mobile_base_configuration)
    # maximal_linear_speed = get_maximal_linear_speed(mobile_base_configuration)
    # maximal_angular_speed = 2 * maximal_linear_speed / track

    maximal_linear_speed = get_maximal_linear_speed(mobile_base_configuration)
    maximal_angular_speed = get_maximal_angular_speed(mobile_base_configuration)

    return omni_steering_teleop_cmd_range_clamp_(
        maximal_linear_speed, maximal_angular_speed, user_cmd_range
    )


def one_axle_steering_teleop_cmd_range_clamp_(
    maximal_linear_speed, maximal_steering_angle, user_cmd_range
):

    cmd_range = {}
    cmd_range["maximal_linear_speed"] = {}
    cmd_range["maximal_linear_speed"]["slow_mode"] = min(
        user_cmd_range["maximal_linear_speed"].get("slow_mode", sys.float_info.max),
        maximal_linear_speed,
    )
    cmd_range["maximal_linear_speed"]["turbo_mode"] = min(
        user_cmd_range["maximal_linear_speed"].get("turbo_mode", sys.float_info.max),
        maximal_linear_speed,
    )
    cmd_range["maximal_steering_angle"] = min(
        user_cmd_range.get("maximal_steering_angle", sys.float_info.max),
        maximal_steering_angle,
    )

    return cmd_range


def one_axle_steering_teleop_cmd_range_clamp(mobile_base_configuration, user_cmd_range):
    maximal_linear_speed = get_maximal_linear_speed(mobile_base_configuration)
    maximal_steering_angle = get_maximal_steering_angle(mobile_base_configuration)

    return one_axle_steering_teleop_cmd_range_clamp_(
        maximal_linear_speed, maximal_steering_angle, user_cmd_range
    )


def two_wheel_steering_teleop_cmd_range_clamp(mobile_base_configuration, user_cmd_range):
    track = get_track(mobile_base_configuration)
    wheelbase = get_wheelbase(mobile_base_configuration)
    maximal_wheel_angle = get_maximal_wheel_angle(mobile_base_configuration)
    maximal_linear_speed = get_maximal_linear_speed(mobile_base_configuration)

    maximal_steering_angle = math.atan(
        math.tan(maximal_wheel_angle)
        / (1 + math.tan(maximal_wheel_angle) * track / (2.0 * wheelbase))
    )

    return one_axle_steering_teleop_cmd_range_clamp_(
        maximal_linear_speed, maximal_steering_angle, user_cmd_range
    )


def two_axle_steering_teleop_cmd_range_clamp_(
    maximal_linear_speed, maximal_steering_angle, user_cmd_range
):
    cmd_range = {}
    cmd_range["maximal_linear_speed"] = {}
    cmd_range["maximal_linear_speed"]["slow_mode"] = min(
        user_cmd_range["maximal_linear_speed"].get("slow_mode", sys.float_info.max),
        maximal_linear_speed,
    )
    cmd_range["maximal_linear_speed"]["turbo_mode"] = min(
        user_cmd_range["maximal_linear_speed"].get("turbo_mode", sys.float_info.max),
        maximal_linear_speed,
    )

    cmd_range["maximal_front_steering_angle"] = min(
        user_cmd_range.get("maximal_front_steering_angle", sys.float_info.max),
        maximal_steering_angle,
    )
    cmd_range["maximal_rear_steering_angle"] = min(
        user_cmd_range.get("maximal_rear_steering_angle", sys.float_info.max),
        maximal_steering_angle,
    )

    return cmd_range


def two_axle_steering_teleop_cmd_range_clamp(mobile_base_configuration, user_cmd_range):

    maximal_linear_speed = get_maximal_linear_speed(mobile_base_configuration)
    maximal_steering_angle = get_maximal_steering_angle(mobile_base_configuration)

    return two_axle_steering_teleop_cmd_range_clamp_(
        maximal_linear_speed, maximal_steering_angle, user_cmd_range
    )


def four_wheel_steering_teleop_cmd_range_clamp(mobile_base_configuration, user_cmd_range):

    maximal_wheel_angle = get_maximal_wheel_angle(mobile_base_configuration)
    maximal_linear_speed = get_maximal_linear_speed(mobile_base_configuration)

    return two_axle_steering_teleop_cmd_range_clamp_(
        maximal_linear_speed, maximal_wheel_angle, user_cmd_range
    )


def cmd_range_clamp(mobile_base_info, cmd_range):

    kinematic_type = get_kinematic_type(mobile_base_info)

    if kinematic_type == "four_wheel_steering":
        return four_wheel_steering_teleop_cmd_range_clamp(mobile_base_info, cmd_range)
    elif kinematic_type == "two_wheel_steering":
        return two_wheel_steering_teleop_cmd_range_clamp(mobile_base_info, cmd_range)
    elif kinematic_type == "two_axle_steering":
        return two_axle_steering_teleop_cmd_range_clamp(mobile_base_info, cmd_range)
    elif kinematic_type == "one_axle_steering":
        return one_axle_steering_teleop_cmd_range_clamp(mobile_base_info, cmd_range)
    elif kinematic_type == "omni_steering":
        return omni_steering_teleop_cmd_range_clamp(mobile_base_info, cmd_range)
    elif kinematic_type == "skid_steering":
        return skid_steering_teleop_cmd_range_clamp(mobile_base_info, cmd_range)
    else:
        raise ValueError("Unknow kinematic type " + kinematic_type)


def get_default_joystick_remapping(mobile_base_configuration, joystick_configuration):

    command_type = get_command_type(mobile_base_configuration)

    default_joystick_remapping_yaml_file = (
        get_package_share_directory("romea_mobile_base_teleop")
        + "/config/"
        + joystick_configuration["type"]
        + "_"
        + command_type
        + "_remappings.yaml"
    )

    with open(default_joystick_remapping_yaml_file) as f:
        return yaml.safe_load(f)


# def get_default_joystick_implement_remapping(joystick_type):

#     default_joystick_remapping_yaml_file = (
#         get_package_share_directory("romea_teleop_description")
#         + "/config/"
#         + joystick_type
#         + "_implement_remappings.yaml"
#     )

#     with open(default_joystick_remapping_yaml_file) as f:
#         return yaml.safe_load(f)


def get_teleop_complete_configuration(
    teleop_configuration, mobile_base_info, joystick_configuration
):

    if "joystick_remapping" in teleop_configuration:
        joystick_remapping = teleop_configuration["joystick_remapping"]
    else:
        joystick_remapping = get_default_joystick_remapping(
            mobile_base_info, joystick_configuration
        )

    joystick_mapping = apply_joystick_remapping(joystick_configuration, joystick_remapping)

    cmd_range = cmd_range_clamp(mobile_base_info, teleop_configuration["cmd_range"])

    return {
        "cmd_output": teleop_configuration["cmd_output"],
        "cmd_range": cmd_range,
        "joystick_mapping": joystick_mapping}
