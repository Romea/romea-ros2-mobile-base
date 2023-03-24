#!/usr/bin/env python3


from ament_index_python.packages import get_package_share_directory
import yaml


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
    print("type", type)
    if type == "2WD" or type == "4WD" or type == "2TD":
        return "skid_steering"
    elif type == "1FAS2RWD" or type == "1FAS2FWD" or type == "1FAS4WD":
        return "one_axle_steering"
    elif type == "1RAS2RWD" or type == "1RAS2FWD" or type == "1RAS4WD":
        return "one_axle_steering"
    elif type == "2AS4WD" or type == "2AS2FWD" or type == "2AS2RWD":
        return "two_axle_steering"
    elif type == "2FWS2FWD" or type == "2FWS2RWD" or type == "2FWS4WD":
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

    # if get_kinematic_type(base_description) != "skid_steering":
    #     return base_description["geometry"]["axles_distance"]
    # else:
    #     raise LookupError(
    #         "No axles distance description found in base info : "
    #         + "cannot get wheelbase"
    #     )


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
            "No maximal angular speed computation is implement for this type of vehicule : "
        )
