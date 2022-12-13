from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):

    host = LaunchConfiguration("host").perform(context)
    port = LaunchConfiguration("port").perform(context)
    mountpoint = LaunchConfiguration("mountpoint").perform(context)
    username = LaunchConfiguration("username").perform(context)
    password = LaunchConfiguration("password").perform(context)

    driver = LaunchDescription()

    ntrip_client_node = Node(
        package='ntrip_client',
        executable='ntrip_ros.py',
        output='screen',
        name="ntrip_client",
        parameters= [
          {"host" : host},
          {"port" : int(port)},
          {"mountpoint": mountpoint},
          {"username" : username},
          {"password" : password},
          {"authenticate" : username!='' and password!=''},
        ],
        remappings=[("nmea", "ntrip/nmea"), ("rtcm", "ntrip/rtcm")],
    )

    driver.add_action(ntrip_client_node)

    return [driver]

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("host"))
    declared_arguments.append(DeclareLaunchArgument("port"))
    declared_arguments.append(DeclareLaunchArgument("mountpoint"))
    declared_arguments.append(DeclareLaunchArgument("username"))
    declared_arguments.append(DeclareLaunchArgument("password"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
