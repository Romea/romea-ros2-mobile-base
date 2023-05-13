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


from romea_common_bringup import MetaDescription, robot_urdf_prefix, robot_prefix
import importlib


class MobileBaseMetaDescription:
    def __init__(self, meta_description_file_path):
        self.meta_description = MetaDescription(
            "mobile_base", meta_description_file_path
        )

    def get_name(self):
        return self.meta_description.get("name")

    def get_type(self):
        return self.meta_description.get("type", "configuration")

    def get_model(self):
        return self.meta_description.get_or("model", "configuration")

    def get_simulation_initial_xyz(self):
        return self.meta_description.get("initial_xyz", "simulation")

    def get_simulation_initial_rpy(self):
        return self.meta_description.get("initial_rpy", "simulation")


def urdf_description(robot_name, mode, meta_description_file_path):

    meta_description = MobileBaseMetaDescription(meta_description_file_path)

    base_type = meta_description.get_type()
    base_model = meta_description.get_model()
    base_bringup = importlib.import_module(base_type + "_bringup")

    urdf_prefix = robot_urdf_prefix(robot_name)
    ros_prefix = robot_prefix(robot_name)

    if not base_model:
        return base_bringup.urdf_description(urdf_prefix, mode, ros_prefix)
    else:
        return base_bringup.urdf_description(urdf_prefix, mode, base_model, ros_prefix)
