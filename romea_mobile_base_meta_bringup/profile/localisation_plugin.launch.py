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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, LoadComposableNodes
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    container = LaunchConfiguration("container").perform(context)
    restamping = LaunchConfiguration("restamping").perform(context)

    common_arguments = {
        "package": "romea_localisation_odo_plugin",
        "name": "localisation_plugin",
        "parameters": [
            {
                "restamping": bool(restamping),
                "controller_topic": "kinematic",
            }
        ],
        "remappings": [
            ("vehicle_controller/odom", "controller/odom"),
            ("vehicle_controller/kinematic", "controller/kinematic")
        ]
    }

    launch = LaunchDescription()
    if container == "":
        executable = "odo_localisation_plugin_node"
        launch.add_action(Node(**common_arguments, executable=executable))
    else:
        plugin = "romea::ros2::OdoLocalisationPlugin"
        launch.add_action(
            LoadComposableNodes(
                target_container=container,
                composable_node_descriptions=[
                    ComposableNode(**common_arguments, plugin=plugin)
                ],
            )
        )

    return [launch]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("restamping", default_value="false"),
            DeclareLaunchArgument("container", default_value=""),
            OpaqueFunction(function=launch_setup)
        ]
    )
