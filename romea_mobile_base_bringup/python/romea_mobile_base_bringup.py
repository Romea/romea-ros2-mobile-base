#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from romea_common_bringup import robot_urdf_prefix
import importlib
import yaml


def get_base_name(base_meta_description):
    return base_meta_description["name"]


def get_base_type(base_meta_description):
    return base_meta_description["configuration"]["type"]


def get_base_model(base_meta_description):
    return base_meta_description["configuration"].get("model", "")


def base_full_name(type, model=""):
    if model != "":
        return type + "_" + model
    else:
        return type


def urdf_description(prefix, mode, meta_description_filename):

    with open(meta_description_filename) as f:
        meta_description = yaml.safe_load(f)

    base_type = get_base_type(meta_description)
    base_model = get_base_model(meta_description)
    base_bringup = importlib.import_module(base_type + "_bringup")

    if base_model == "":
        return base_bringup.urdf_description(prefix, mode)
    else:
        return base_bringup.urdf_description(prefix, mode, base_model)
