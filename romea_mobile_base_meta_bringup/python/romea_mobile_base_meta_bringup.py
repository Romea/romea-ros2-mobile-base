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

    def get_bringup_package(self):
        return importlib.import_module(self.get_model() + "_bringup")


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


def get_configuration(meta_description):
    base_model = meta_description.get_version()
    base_bringup = meta_description.get_bringup_package()
    if not base_model:
        return base_bringup.get_configuration()
    else:
        return base_bringup.get_configuration(base_model)


def generate_configuration_file(meta_description, extended):
    base_model = meta_description.get_version()
    base_bringup = meta_description.get_bringup_package()

    if not base_model:
        return base_bringup.generate_configuration_file(extended)
    else:
        return base_bringup.generate_configuration_file(base_model, extended)


def generate_launch_file(meta_description):

    launch_arguments = [
        {"name": "mode", "default": "live"},
        {"name": "joystick_topic"},
        {"name": "joystick_configuration_file_path"},
    ]

    namespaces = [meta_description.get_robot_name(), meta_description.get_name()]

    configuration = get_configuration(meta_description)
    configuration["tf_prefix"] = meta_description.get_urdf_prefix()
    configuration["frame_id"] = meta_description.get_link()
    configuration["name"] = meta_description.get_name()

    return LaunchFileGenerator("mobile_base").generate(
        meta_description.get_launch_file(), launch_arguments, namespaces, configuration
    )


def generate_urdf_description(mode, meta_description):

    base_name = meta_description.get_name()
    base_model = meta_description.get_version()
    base_bringup = meta_description.get_bringup_package()
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


def generate_ros2_control_description(mode, meta_description):

    base_name = meta_description.get_name()
    base_model = meta_description.get_version()
    base_bringup = meta_description.get_bringup_package()
    urdf_prefix = meta_description.get_urdf_prefix()

    if not base_model:
        return base_bringup.generate_ros2_control_description(
            urdf_prefix, mode, base_name
        )
    else:
        return base_bringup.generate_ros2_control_description(
            urdf_prefix, mode, base_name, base_model
        )
