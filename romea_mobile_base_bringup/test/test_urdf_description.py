# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import os
import pytest
from ament_index_python import get_package_prefix
import xml.etree.ElementTree as ET
import subprocess


@pytest.fixture(scope="module")
def urdf():

    exe = (
        get_package_prefix("romea_mobile_base_bringup")
        + "/lib/romea_mobile_base_bringup/urdf_description.py"
    )

    meta_description_filename = os.path.join(
        os.getcwd(), "test_mobile_base_bringup.yaml"
    )

    return ET.fromstring(
        subprocess.check_output(
            [
                exe,
                "mode:simulation",
                "robot_namespace:robot",
                "meta_description_filename:" + meta_description_filename,
            ],
            encoding="utf-8",
        )
    )


def test_mobile_base_name(urdf):
    assert urdf.get("name") == "adap2e_fat"


def test_mobile_base_link_name(urdf):
    assert urdf.find("link").get("name") == "robot_base_footprint"
