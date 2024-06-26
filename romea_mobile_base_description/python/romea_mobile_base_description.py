#!/usr/bin/env python3


from ament_index_python.packages import get_package_share_directory
import yaml
import math


def robot_full_name(robot_type, robot_model):
    if robot_model != "":
        return robot_type + "_" + robot_model
    else:
        return robot_type


def get_mobile_base_description_file_path(robot_type, robot_model):
    robot_name = robot_full_name(robot_type, robot_model)
    return (
        get_package_share_directory(robot_type + "_description")
        + "/config/"
        + robot_name
        + ".yaml"
    )


def get_mobile_base_description(robot_type, robot_model):
    with open(get_mobile_base_description_file_path(robot_type, robot_model)) as f:
        return yaml.safe_load(f)


def get_default_teleop_configuration_file_path(robot_type):
    return (
        get_package_share_directory(robot_type + "_description") + "/config/teleop.yaml"
    )


def get_default_teleop_configuration(robot_type):
    with open(get_mobile_base_description_file_path(robot_type)) as f:
        return yaml.safe_load(f)


def get_type(base_description):
    return base_description["type"]


def get_kinematic_type(base_description):
    type = get_type(base_description)
    if type == "2WD" or type == "4WD" or "2T" in type:
        return "skid_steering"
    elif "1FAS" in type:
        return "one_axle_steering"
    elif "2AS" in type:
        return "two_axle_steering"
    elif "2FWS" in type:
        return "two_wheel_steering"
    elif type == "4WS4WD":
        return "four_wheel_steering"
    elif type == "4WMD":
        return "omni_steering"
    else:
        raise LookupError("Robot type found in base info is not available")


def get_command_type(base_description):
    kinematic_type = get_kinematic_type(base_description)
    if kinematic_type == "four_wheel_steering":
        return "two_axle_steering"
    elif kinematic_type == "two_wheel_steering":
        return "one_axle_steering"
    else:
        return kinematic_type


def get_wheelbase(base_description):

    if "axles_distance" in base_description["geometry"]:
        return base_description["geometry"]["axles_distance"]
    else:
        raise LookupError(
            "No axles distance description found in base info : "
            + "cannot get wheelbase"
        )


def get_wheelbase_or(base_description, default_value):
    try:
        return get_wheelbase(base_description)
    except LookupError:
        return default_value


def get_inertia(base_description):
    return base_description["inertia"]


def get_track(base_description):

    if (
        "front_wheels_steering_control" in base_description
        or "front_axle_steering_control" in base_description
    ):
        track = base_description["geometry"]["front_axle"]["wheels_distance"]

    elif (
        "rear_wheels_steering_control" in base_description
        or "rear_axle_steering_control" in base_description
    ):
        track = base_description["geometry"]["rear_axle"]["wheels_distance"]

    elif (
        "wheels_steering_control" in base_description
        or "axles_steering_control" in base_description
    ):
        assert (
            base_description["geometry"]["front_axle"]["wheels_distance"]
            == base_description["geometry"]["rear_axle"]["wheels_distance"]
        )
        track = base_description["geometry"]["front_axle"]["wheels_distance"]

    else:

        if "wheels_speed_control" in base_description:
            if "wheels_distance" in base_description["geometry"]:
                track = base_description["geometry"]["wheels_distance"]
            else:
                assert (
                    base_description["geometry"]["front_axle"]["wheels_distance"]
                    == base_description["geometry"]["rear_axle"]["wheels_distance"]
                )

                track = base_description["geometry"]["front_axle"]["wheels_distance"]

        if "tracks_speed_control" in base_description:
            track = base_description["geometry"]["tracks_distance"]

    return track


def get_maximal_linear_speed(base_description):

    if "wheels_speed_control" in base_description:
        speed_control_info = base_description["wheels_speed_control"]

    elif "front_wheels_speed_control" in base_description:
        speed_control_info = base_description["front_wheels_speed_control"]

    elif "rear_wheels_speed_control" in base_description:
        speed_control_info = base_description["rear_wheels_speed_control"]

    elif "tracks_speed_control" in base_description:
        speed_control_info = base_description["tracks_speed_control"]

    else:
        raise LookupError(
            "No wheels or tracks speed control description found in base info : "
            + "cannot get maximal linear speed"
        )

    return speed_control_info["command"]["maximal_speed"]


def get_maximal_wheel_angle(base_description):

    if "front_wheels_steering_control" in base_description:
        wheels_steering_control_info = base_description["front_wheels_steering_control"]

    elif "rear_wheels_steering_control" in base_description:
        wheels_steering_control_info = base_description["rear_wheels_steering_control"]

    elif "wheels_steering_control" in base_description:
        wheels_steering_control_info = base_description["wheels_steering_control"]

    else:
        raise LookupError(
            "No wheel steering control description found in base : "
            + "info cannot get maximal wheel angle"
        )

    return wheels_steering_control_info["command"]["maximal_angle"]


def get_maximal_steering_angle(base_description):

    if "front_axle_steering_control" in base_description:
        steering_control_info = base_description["front_axle_steering_control"]

    elif "rear_axle_steering_control" in base_description:
        steering_control_info = base_description["rear_axle_steering_control"]

    elif "axles_steering_control" in base_description:
        steering_control_info = base_description["axles_steering_control"]

    else:
        raise LookupError(
            "No axle steering control description found in base info : "
            + "cannot get maximal steering angle"
        )

    return steering_control_info["command"]["maximal_angle"]


def get_maximal_angular_speed(base_description):

    kinematic = get_kinematic_type(base_description)
    if kinematic == "skid_steering" or "omni_steering":
        track = get_track(base_description)
        maximal_linear_speed = get_maximal_linear_speed(base_description)
        return 2 * maximal_linear_speed / track

    else:
        raise LookupError(
            "No maximal angular speed computation is implemented for this kind of vehicule : "
        )


def get_skid_steering_command_limits(base_description):
    return {
        "minimal_longitudinal_speed": -get_maximal_linear_speed(base_description),
        "maximal_longitudinal_speed": get_maximal_linear_speed(base_description),
        "maximal_angular_speed": get_maximal_angular_speed(base_description),
    }


def get_one_axle_steering_command_limits(base_description):
    return {
        "minimal_longitudinal_speed": -get_maximal_linear_speed(base_description),
        "maximal_longitudinal_speed": get_maximal_linear_speed(base_description),
        "maximal_steering_angle": get_maximal_steering_angle(base_description),
    }


def get_two_axle_steering_command_limits(base_description):
    return {
        "minimal_longitudinal_speed": -get_maximal_linear_speed(base_description),
        "maximal_longitudinal_speed": get_maximal_linear_speed(base_description),
        "maximal_front_steering_angle": get_maximal_steering_angle(base_description),
        "maximal_rear_steering_angle": get_maximal_steering_angle(base_description),
    }


def get_two_wheel_steering_command_limits(base_description):

    track = get_track(base_description)
    wheelbase = get_wheelbase(base_description)
    maximal_wheel_angle = get_maximal_wheel_angle(base_description)

    maximal_steering_angle = math.atan(
        math.tan(maximal_wheel_angle)
        / (1 + math.tan(maximal_wheel_angle) * track / (2.0 * wheelbase))
    )

    return {
        "minimal_longitudinal_speed": -get_maximal_linear_speed(base_description),
        "maximal_longitudinal_speed": get_maximal_linear_speed(base_description),
        "maximal_front_steering_angle": maximal_steering_angle,
    }


def get_four_wheel_steering_command_limits(base_description):
    return {
        "minimal_longitudinal_speed": -get_maximal_linear_speed(base_description),
        "maximal_longitudinal_speed": get_maximal_linear_speed(base_description),
        "maximal_front_steering_angle": get_maximal_wheel_angle(base_description),
        "maximal_rear_steering_angle": get_maximal_wheel_angle(base_description),
    }


def get_omni_steering_command_limits(base_description):
    return {
        "minimal_longitudinal_speed": -get_maximal_linear_speed(base_description),
        "maximal_longitudinal_speed": get_maximal_linear_speed(base_description),
        "maximal_lateral_speed": get_maximal_linear_speed(base_description),
        "maximal_angular_speed": get_maximal_angular_speed(base_description),
    }


def get_command_limits(base_description):
    kinematic_type = get_kinematic_type(base_description)
    if kinematic_type == "skid_steering":
        return get_skid_steering_command_limits(base_description)
    elif kinematic_type == "omni_steering":
        return get_omni_steering_command_limits(base_description)
    elif kinematic_type == "one_axle_steering":
        return get_one_axle_steering_command_limits(base_description)
    elif kinematic_type == "two_axle_steering":
        return get_two_axle_steering_command_limits(base_description)
    elif kinematic_type == "two_wheel_steering":
        return get_two_wheel_steering_command_limits(base_description)
    else:
        return get_four_wheel_steering_command_limits(base_description)
