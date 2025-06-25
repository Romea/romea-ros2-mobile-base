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
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    robot_version = LaunchConfiguration("version").perform(context)
    base_name = LaunchConfiguration("name").perform(context)

    joystick_topic = LaunchConfiguration("joystick_topic").perform(context)
    joystick_configuration_file_path = LaunchConfiguration(
        "joystick_configuration_file_path"
    ).perform(context)

    launch = LaunchDescription()

    launch.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("adap2e_bringup")
                + "/launch/adap2e_base.launch.py"
            ),
            launch_arguments={
                "mode": mode,
                "tf_prefix": tf_prefix,
                "robot_model": robot_version,
                "base_name": base_name,
            }.items(),
        )
    )

    launch.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("adap2e_bringup")
                + "/launch/adap2e_teleop.launch.py"
            ),
            launch_arguments={
                "mode": mode,
                "robot_model": robot_version,
                "joystick_topic": joystick_topic,
                "joystick_configuration_file_path": joystick_configuration_file_path,
            }.items(),
        )
    )

    return [launch]


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument("joystick_configuration_file_path"),
        DeclareLaunchArgument("joystick_topic"),
        DeclareLaunchArgument("mode", default_value="live"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
