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

    meta_description_file_path = os.path.join(
        os.getcwd(), "test_mobile_base_bringup.yaml"
    )

    return ET.fromstring(
        subprocess.check_output(
            [
                exe,
                "mode:simulation",
                "robot_namespace:robot",
                "meta_description_file_path:" + meta_description_file_path,
            ],
            encoding="utf-8",
        )
    )


def test_mobile_base_name(urdf):
    assert urdf.get("name") == "adap2e_fat"


def test_mobile_base_link_name(urdf):
    assert urdf.find("link").get("name") == "robot_base_footprint"
