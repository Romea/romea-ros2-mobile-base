# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license


import pytest
import yaml

# import xml.etree.ElementTree as ET
from romea_mobile_base_description import (
    get_kinematic_type,
    get_command_type,
    get_wheelbase,
    get_track,
    get_maximal_linear_speed,
    get_maximal_wheel_angle,
    get_maximal_steering_angle,
    get_maximal_angular_speed,
)


def get_configuration_(type):
    with open("test/test_mobile_base_parameters_" + type + ".yaml") as f:
        return yaml.safe_load(f)


def get_kinematic_type_(type):
    configuration = get_configuration_(type)
    return get_kinematic_type(configuration)


def get_command_type_(type):
    configuration = get_configuration_(type)
    return get_command_type(configuration)


def get_wheelbase_(type):
    configuration = get_configuration_(type)
    return get_wheelbase(configuration)


def get_track_(type):
    configuration = get_configuration_(type)
    return get_track(configuration)


def get_maximal_linear_speed_(type):
    configuration = get_configuration_(type)
    return get_maximal_linear_speed(configuration)


def get_maximal_wheel_angle_(type):
    configuration = get_configuration_(type)
    return get_maximal_wheel_angle(configuration)


def get_maximal_steering_angle_(type):
    configuration = get_configuration_(type)
    return get_maximal_steering_angle(configuration)


def test_get_kinematic_types():
    assert get_kinematic_type_("1FAS2RWD") == "one_axle_steering"
    assert get_kinematic_type_("2AS4WD") == "two_axle_steering"
    assert get_kinematic_type_("2FWS2FWD") == "two_wheel_steering"
    assert get_kinematic_type_("2FWS2RWD") == "two_wheel_steering"
    assert get_kinematic_type_("2FWS4WD") == "two_wheel_steering"
    assert get_kinematic_type_("2TD") == "skid_steering"
    assert get_kinematic_type_("2WD") == "skid_steering"
    assert get_kinematic_type_("4WD") == "skid_steering"
    assert get_kinematic_type_("4WS4WD") == "four_wheel_steering"


def test_get_command_types():
    assert get_command_type_("1FAS2RWD") == "one_axle_steering"
    assert get_command_type_("2AS4WD") == "two_axle_steering"
    assert get_command_type_("2FWS2FWD") == "one_axle_steering"
    assert get_command_type_("2FWS2RWD") == "one_axle_steering"
    assert get_command_type_("2FWS4WD") == "one_axle_steering"
    assert get_command_type_("2TD") == "skid_steering"
    assert get_command_type_("2WD") == "skid_steering"
    assert get_command_type_("4WD") == "skid_steering"
    assert get_command_type_("4WS4WD") == "two_axle_steering"


def test_get_wheelbases():
    assert get_wheelbase_("1FAS2RWD") == 1.0
    assert get_wheelbase_("2AS4WD") == 1.0
    assert get_wheelbase_("2FWS2FWD") == 1.0
    assert get_wheelbase_("2FWS2RWD") == 1.0
    assert get_wheelbase_("2FWS4WD") == 1.0
    assert get_wheelbase_("4WD") == 1.0
    assert get_wheelbase_("4WS4WD") == 1.0

    with pytest.raises(LookupError):
        get_wheelbase_("2TD")
    with pytest.raises(LookupError):
        get_wheelbase_("2WD")


def test_get_tracks():
    assert get_track_("1FAS2RWD") == 2.0
    assert get_track_("2AS4WD") == 2.0
    assert get_track_("2FWS2FWD") == 2.0
    assert get_track_("2FWS2RWD") == 2.0
    assert get_track_("2FWS4WD") == 2.0
    assert get_track_("2TD") == 2.0
    assert get_track_("2WD") == 2.0
    assert get_track_("4WD") == 2.0
    assert get_track_("4WS4WD") == 2.0


def get_maximal_linear_speeds():
    assert get_maximal_linear_speed_("1FAS2RWD") == 4.0
    assert get_maximal_linear_speed_("2AS4WD") == 4.0
    assert get_maximal_linear_speed_("1FAS2RWD") == 4.0
    assert get_maximal_linear_speed_("2FWS2FWD") == 4.0
    assert get_maximal_linear_speed_("2FWS2RWD") == 4.0
    assert get_maximal_linear_speed_("2FWS4WD") == 4.0
    assert get_maximal_linear_speed_("2TD") == 4.0
    assert get_maximal_linear_speed_("2WD") == 4.0
    assert get_maximal_linear_speed_("4WD") == 4.0
    assert get_maximal_linear_speed_("4WS4WD") == 4.0


def get_maximal_wheel_angles():

    assert get_maximal_wheel_angle_("2FWS2FWD") == 5.0
    assert get_maximal_wheel_angle_("2FWS2RWD") == 5.0
    assert get_maximal_wheel_angle_("2FWS4WD") == 5.0
    assert get_maximal_wheel_angle_("4WS4WD") == 5.0

    with pytest.raises(LookupError):
        get_maximal_wheel_angle_("1FAS2RWD")
    with pytest.raises(LookupError):
        get_maximal_wheel_angle_("2AS4WD")
    with pytest.raises(LookupError):
        get_maximal_wheel_angle_("2TD")
    with pytest.raises(LookupError):
        get_maximal_wheel_angle_("2WD")
    with pytest.raises(LookupError):
        get_maximal_wheel_angle_("4WD")


def get_maximal_steering_angles():

    assert get_maximal_steering_angle_("1FAS2RWD") == 5.0
    assert get_maximal_steering_angle_("2AS4WD") == 5.0

    with pytest.raises(LookupError):
        get_maximal_steering_angle_("2FWS2FWD")
    with pytest.raises(LookupError):
        get_maximal_steering_angle_("2FWS2RWD")
    with pytest.raises(LookupError):
        get_maximal_steering_angle_("2FWS4WD")
    with pytest.raises(LookupError):
        get_maximal_steering_angle_("4WS4WD")
    with pytest.raises(LookupError):
        get_maximal_steering_angle_("2TD")
    with pytest.raises(LookupError):
        get_maximal_steering_angle_("2WD")
    with pytest.raises(LookupError):
        get_maximal_steering_angle_("4WD")


def compute_maximal_angular_speed():
    with pytest.raises(LookupError):
        get_maximal_angular_speed("1FAS2RWD")
    with pytest.raises(LookupError):
        get_maximal_angular_speed("2AS4WD")
    with pytest.raises(LookupError):
        get_maximal_angular_speed("2FWS2FWD")
    with pytest.raises(LookupError):
        get_maximal_angular_speed("2FWS2RWD")
    with pytest.raises(LookupError):
        get_maximal_angular_speed("2FWS4WD")
    with pytest.raises(LookupError):
        get_maximal_angular_speed("4WS4WD")

    assert get_maximal_angular_speed("2TD") == 4.0
    assert get_maximal_angular_speed("2WD") == 4.0
    assert get_maximal_angular_speed("4WD") == 4.0
