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


from launch.substitutions import LaunchConfiguration

import romea_common_meta_bringup.ros_launch as common
from romea_mobile_base_meta_bringup.meta_description import load_meta_description


def get_meta_description(context):
    robot_namespace = common.get_robot_namespace(context)
    meta_description_file_path = common.get_meta_description_file_path(context)
    return load_meta_description(meta_description_file_path, robot_namespace)


def declare_base_name(default_value=None):
    return common.declare_argument(
        {
            "name": "base_name",
            "description": "Name of the mobile base",
        },
        default_value
    )


def get_base_name(context):
    return LaunchConfiguration("base_name").perform(context)
