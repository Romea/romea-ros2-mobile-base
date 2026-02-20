#!/usr/bin/env python3

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

import sys

from romea_common_meta_bringup import complete_mode
from romea_mobile_base_meta_bringup.meta_description import (
    generate_xml_ros2_control_description_str,
    MobileBaseMetaDescription,
)

if __name__ == "__main__":

    argv = sys.argv

    parameters = {}
    for argument in argv[1:]:
        name, value = argument.split(":")
        parameters[name] = value

    mode = complete_mode(parameters["mode"])
    robot_namespace = parameters["robot_namespace"]
    meta_description_file_path = parameters["meta_description_file_path"]
    meta_description = MobileBaseMetaDescription(meta_description_file_path, robot_namespace)
    print(generate_xml_ros2_control_description_str(mode, meta_description))
