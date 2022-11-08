from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
    PythonExpression,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter, PushRosNamespace

from launch_ros.substitutions import (
    FindPackageShare,
    FindPackagePrefix,
    ExecutableInPackage,
)

from ament_index_python.packages import get_package_share_directory

import yaml
import xml.etree.ElementTree as ET


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    robot_type = LaunchConfiguration("robot_type").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    joystick_type = LaunchConfiguration("joystick_type").perform(context)
    devices_description = LaunchConfiguration("devices_description").perform(context).strip('[]').split(", ")
    launch_gazebo = LaunchConfiguration("launch_gazebo").perform(context)

    if robot_model != "":
        robot_name = robot_type + "_" + robot_model
    else:
        robot_name = robot_type

    base_urdf_description = Command(
        [
            ExecutableInPackage("urdf_description.py", robot_type + "_bringup"),
            " robot_namespace:",
            robot_namespace,
            " robot_model:",
            robot_model,
            " mode:",
            mode,
        ]
    )

    urdf = ET.fromstring(base_urdf_description.perform(context))


    for device_description in devices_description:

        device_type = device_description.split(".")[1]

        device_urdf_description = Command(
            [
                ExecutableInPackage(
                    "urdf_description.py", "romea_" + device_type + "_bringup"
                ),
                " robot_namespace:",
                robot_namespace,
                " description_yaml_file:",
                device_description.strip("'"),
            ]
        )

#        print( device_urdf_description.perform(context))
        urdf.extend(ET.fromstring(device_urdf_description.perform(context)))

#    print(ET.tostring(urdf, encoding="unicode"))
    robot=[]

    robot.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare(robot_type + "_bringup"),
                            "launch",
                            robot_name + ".launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "mode": mode,
                "robot_namespace": robot_namespace,
                "joystick_type": joystick_type,
                "launch_gazebo": launch_gazebo,
                "urdf_description": ET.tostring(urdf, encoding="unicode"),
            }.items(),
        )
    )


    if mode == "live" :

       for device_description in devices_description:

           device_type = device_description.split(".")[1]

           robot.append(
               IncludeLaunchDescription(
                   PythonLaunchDescriptionSource(
                       [
                           PathJoinSubstitution(
                               [
                                   FindPackageShare("romea_"+device_type + "_bringup"),
                                   "launch",
                                   device_type + "_driver.launch.py",
                               ]
                           )
                       ]
                   ),
                   launch_arguments={
                       "robot_namespace": robot_namespace,
                       "description_yaml_file": device_description.strip("'"),
                   }.items(),
               )
           )

    return robot

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_type"))

    declared_arguments.append(DeclareLaunchArgument("robot_model", default_value=""))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="robot")
    )

    declared_arguments.append(
        DeclareLaunchArgument("joystick_type", default_value="xbox")
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_gazebo", default_value="true")
    )

    declared_arguments.append(DeclareLaunchArgument("devices_description"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
