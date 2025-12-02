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


from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from romea_joystick_meta_bringup.meta_description import (
    generate_launch_file as generate_joystick_launch_file,
)
from romea_joystick_meta_bringup.meta_description import JoystickMetaDescription
from romea_joystick_utils import get_joystick_configuration_file_path

from romea_mobile_base_meta_bringup.meta_description import (
    generate_launch_file as generate_mobile_base_launch_file,
)
from romea_mobile_base_meta_bringup.meta_description import MobileBaseMetaDescription


def get_mode(context):
    mode = LaunchConfiguration("mode").perform(context)
    return "simulation_gazebo_classic" if mode == "simulation" else mode


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_mobile_base_meta_description(context):
    meta_description_file_path = LaunchConfiguration(
        "mobile_base_meta_description_file_path"
    ).perform(context)

    return MobileBaseMetaDescription(meta_description_file_path, get_robot_namespace(context))


def get_joystick_meta_description(context):
    meta_description_file_path = LaunchConfiguration(
        "joystick_meta_description_file_path"
    ).perform(context)

    return JoystickMetaDescription(meta_description_file_path, get_robot_namespace(context))


def launch_setup(context, *args, **kwargs):
    mode = get_mode(context)

    actions = []

    joystick_meta_description = get_joystick_meta_description(context)
    joystick_launch_filename = (
        f"/tmp/{joystick_meta_description.get_filename_prefix()}driver.launch.yaml"
    )
    with open(joystick_launch_filename, "w") as f:
        f.write(generate_joystick_launch_file(joystick_meta_description))

    actions.append(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(joystick_launch_filename),
            launch_arguments={
                "mode": mode,
            }.items(),
        )
    )

    mobile_base_meta_description = get_mobile_base_meta_description(context)
    mobile_base_launch_filename = (
        f"/tmp/{mobile_base_meta_description.get_filename_prefix()}driver.launch.yaml"
    )
    with open(mobile_base_launch_filename, "w") as f:
        f.write(generate_mobile_base_launch_file(mobile_base_meta_description))

    actions.append(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mobile_base_launch_filename),
            launch_arguments={
                "mode": mode,
                "joystick_topic": joystick_meta_description.get_full_namespace() + "/joy",
                "joystick_configuration_file_path": get_joystick_configuration_file_path(
                    joystick_meta_description.get_msg_layout()
                ),
            }.items(),
        )
    )

    return actions


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("mobile_base_meta_description_file_path"),
            DeclareLaunchArgument("joystick_meta_description_file_path"),
            DeclareLaunchArgument("robot_namespace", default_value=""),
            DeclareLaunchArgument("mode", default_value="live"),
            OpaqueFunction(function=launch_setup),
        ]
    )
