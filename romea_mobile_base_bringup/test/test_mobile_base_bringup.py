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

from romea_mobile_base_bringup import MobileBaseMetaDescription


@pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(
        os.getcwd(), "test_mobile_base_bringup.yaml"
    )
    return MobileBaseMetaDescription(meta_description_file_path)


def test_get_name(meta_description):
    assert meta_description.get_name() == "base"


def test_get_type(meta_description):
    assert meta_description.get_type() == "adap2e"


def test_get_model(meta_description):
    assert meta_description.get_model() == "fat"


def test_get_simulation_initial_xyz(meta_description):
    assert meta_description.get_simulation_initial_xyz() == [1.0, 2.0, 3.0]


def test_get_simulation_initial_rpy(meta_description):
    assert meta_description.get_simulation_initial_rpy() == [4.0, 5.0, 6.0]


def test_get_records(meta_description):
    records = meta_description.get_records()
    assert records["joint_states"] is True
    assert records["controller/odom"] is True
    assert records["controller/odometry"] is True
    assert records["controller/kinematic"] is True
