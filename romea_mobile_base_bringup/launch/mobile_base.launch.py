from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import ExecutableInPackage
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from romea_mobile_base_bringup import MobileBaseMetaDescription


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_meta_description(context):

    meta_description_filename = LaunchConfiguration(
        "meta_description_filename"
    ).perform(context)

    return MobileBaseMetaDescription(meta_description_filename)


def get_urdf_description(context):
    return LaunchConfiguration("urdf_description").perform(context)


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    meta_description = get_meta_description(context)
    urdf_description = get_urdf_description(context)

    base_type = meta_description.get_type()
    base_model = meta_description.get_model()

    launch_arguments = {
        "mode": mode,
        "robot_namespace": robot_namespace,
        "urdf_description": urdf_description
        # initial xyz intial rpy
    }

    if base_model is not None:
        launch_arguments["robot_model"] = base_model

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(base_type + "_bringup")
            + "/launch/"
            + base_type
            + "_base.launch.py"
        ),
        launch_arguments=launch_arguments.items(),
    )

    return [base]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode", default_value=""))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="")
    )

    declared_arguments.append(DeclareLaunchArgument("meta_description_filename"))

    urdf_description = Command(
        [
            ExecutableInPackage("urdf_description.py", "romea_mobile_base_bringup"),
            " mode:",
            LaunchConfiguration("mode"),
            " robot_namespace:",
            LaunchConfiguration("robot_namespace"),
            " meta_description_filename:",
            LaunchConfiguration("meta_description_filename"),
        ]
    )

    declared_arguments.append(
        DeclareLaunchArgument("urdf_description", default_value=urdf_description)
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
