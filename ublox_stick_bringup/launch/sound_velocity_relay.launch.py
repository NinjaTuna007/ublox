"""Relay PVPTU sound_vel (Float64) to waraps/sensor/sound_velocity (String)."""
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')

    return launch.LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='stick_1'),
        DeclareLaunchArgument('input_topic', default_value='sound_vel'),
        DeclareLaunchArgument('output_topic', default_value='waraps/sensor/sound_velocity'),
        Node(
            package='ublox_stick_bringup',
            executable='sound_velocity_relay.py',
            name='sound_velocity_relay',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,
            }],
        ),
    ])
