from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):

    device = LaunchConfiguration("device").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    rate = LaunchConfiguration("rate").perform(context)

    driver = LaunchDescription()

    driver_node = Node(
            package='romea_ublox',
            executable='ublox_driver_node',
            name='driver',
            output='screen',
            parameters= [
              {"device" : device},
              {"baudrate" : int(baudrate)},
              {"frame_id": frame_id},
              {"rate" : int(rate)},
            ]
    )

    driver.add_action(driver_node)

    return [driver]

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("device"))
    declared_arguments.append(DeclareLaunchArgument("baudrate"))
    declared_arguments.append(DeclareLaunchArgument("frame_id"))
    declared_arguments.append(DeclareLaunchArgument("rate"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
