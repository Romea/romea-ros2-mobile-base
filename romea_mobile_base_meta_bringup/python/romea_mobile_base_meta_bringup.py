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

import importlib
from romea_common_meta_bringup import SensorMetaDescription, LaunchFileGenerator, robot_prefix


class MobileBaseMetaDescription(SensorMetaDescription):
    def __init__(self, meta_description_file_path, robot_name=None):
        super().__init__("mobile_base", meta_description_file_path, robot_name)

    def get_simulation_initial_xyz(self):
        return self._get("initial_xyz", "simulation")

    def get_simulation_initial_rpy(self):
        return self._get("initial_rpy", "simulation")


def load_meta_description(meta_description_file_path, robot_name=None):
    return MobileBaseMetaDescription(meta_description_file_path, robot_name)


# def get_sensor_specifications(meta_description):
#     return romea_imu_description.get_imu_specifications(
#         meta_description.get_manufacturer(), meta_description.get_model()
#     )


# def get_sensor_geometry(meta_description):
#     return romea_imu_description.get_imu_geometry(
#         meta_description.get_manufacturer(), meta_description.get_model()
#     )


# def get_complete_sensor_configuration(meta_description):
#     return romea_imu_description.get_imu_complete_configuration(
#         meta_description.get_name(), meta_description.get_configuration()
#     )


# def generate_configuration_file(meta_description, extended):
#     configuration = get_complete_sensor_configuration(meta_description)
#     units = romea_imu_description.get_imu_specification_units()
#     return romea_common_description.generate_configuration_file(
#        configuration, units, extended)


def generate_launch_file(meta_description):

    launch_arguments = [
        {"name": "mode", "default": "live"},
        {"name": "joystick_topic"},
        {"name": "joystick_configuration_file_path"},
    ]

    namespaces = [meta_description.get_robot_name(), meta_description.get_name()]

    configuration = {
        "frame_id": meta_description.get_link(),
        "tf_prefix": meta_description.get_urdf_prefix(),
        "model": meta_description.get_model(),
        "version": meta_description.get_version(),
        "name": meta_description.get_name() 
    }

    return LaunchFileGenerator("mobile_base").generate(
        meta_description.get_launch_file(), launch_arguments, namespaces, configuration
    )


def generate_urdf_description(mode, meta_description):

    base_name = meta_description.get_name()
    base_type = meta_description.get_model()
    base_model = meta_description.get_version()
    base_bringup = importlib.import_module(base_type + "_bringup")
    urdf_prefix = meta_description.get_urdf_prefix()
    ros_prefix = robot_prefix(meta_description.get_robot_name())

    if not base_model:
        return base_bringup.generate_urdf_description(
            urdf_prefix, mode, base_name, ros_prefix
        )
    else:
        return base_bringup.generate_urdf_description(
            urdf_prefix, mode, base_name, base_model, ros_prefix
        )
