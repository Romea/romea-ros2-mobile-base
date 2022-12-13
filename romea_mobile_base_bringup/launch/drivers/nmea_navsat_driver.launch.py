from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):

    port = LaunchConfiguration("device").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)

    drivers = LaunchDescription()

    nmea_driver_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_topic_serial_reader',
        output='screen',
        name="nmea_driver",
        parameters=[
          {"port" : port},
          {"baud" : int(baudrate)},
          {"frame_id": frame_id},
        ],
        remappings=[("nmea_sentence","nmea")],
    )

    drivers.add_action(nmea_driver_node)

    topic_driver_node = Node(
            package='nmea_navsat_driver',
            executable='nmea_topic_driver',
            name='topic_driver',
            output='screen',
            remappings=[("nmea_sentence","nmea")],
    )

    drivers.add_action(topic_driver_node)

    return [drivers]

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("device"))
    declared_arguments.append(DeclareLaunchArgument("baudrate"))
    declared_arguments.append(DeclareLaunchArgument("frame_id"))
    declared_arguments.append(DeclareLaunchArgument("rate")) # just to be compatible


    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
