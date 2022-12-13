#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
import yaml


def robot_full_name(robot_type, robot_model):
    if robot_model != "":
        return robot_type + "_" + robot_model
    else:
        return robot_type


def get_mobile_base_description_filename(robot_type, robot_model):
    robot_name = robot_full_name(robot_type, robot_model)
    return (
        get_package_share_directory(robot_type + "_description")
        + "/config/"
        + robot_name
        + ".yaml"
    )


def get_mobile_base_description(robot_type, robot_model):
    with open(get_mobile_base_description_filename(robot_type, robot_model)) as f:
        return yaml.safe_load(f)


def get_default_teleop_configuration_filename(robot_type):
    return (
        get_package_share_directory(robot_type + "_description") + "/config/teleop.yaml"
    )


def get_default_teleop_configuration(robot_type):
    with open(get_mobile_base_description_filename(robot_type)) as f:
        return yaml.safe_load(f)


def get_wheelbase(base_description):
    return base_description["geometry"]["axles_distance"]


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
        raise AttributeError(
            "No wheels or tracks speed control description found in base info cannot get maximal linear speed"
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
        raise AttributeError(
            "No wheel steering control description found in base info cannot get maximal wheel angle"
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
        raise AttributeError(
            "No axle steering control description found in base info cannot get maximal steering angle"
        )

    return steering_control_info["command"]["maximal_angle"]


def get_track(base_description):

    if "front_wheels_steering_control" in base_description:
        track = base_description["geometry"]["front_axle"]["wheels_distance"]

    elif "rear_wheels_steering_control" in base_description:
        track = base_description["geometry"]["rear_axle"]["wheels_distance"]

    elif "wheels_steering_control" in base_description:
        assert (
            base_description["geometry"]["front_axle"]["wheels_distance"]
            == base_description["geometry"]["rear_axle"]["wheels_distance"]
        )
        track = base_description["geometry"]["front_axle"]["wheels_distance"]

    else:

        if "wheels_speed_control" in base_description:
            assert (
                base_description["geometry"]["front_axle"]["wheels_distance"]
                == base_description["geometry"]["rear_axle"]["wheels_distance"]
            )

            track = base_description["geometry"]["front_axle"]["wheels_distance"]

        if "tracks_speed_control" in base_description:
            track = base_description["geometry"]["tracks_distance"]

    return track


def get_kinematic_type(base_description):
    return base_description["kinematic"]


def get_command_type(base_description):
    kinematic_type = get_kinematic_type(base_description)
    if kinematic_type == "four_wheel_steering":
        return "two_axle_steering"
    elif kinematic_type == "two_wheel_steering":
        return "one_axle_steering"
    else:
        return kinematic_type
