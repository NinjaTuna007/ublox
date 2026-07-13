"""Launch ublox_dgnss NTRIP client (publishes to /ntrip_client/rtcm)."""
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_https = LaunchConfiguration('use_https')
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    mountpoint = LaunchConfiguration('mountpoint')
    username = LaunchConfiguration('username')
    password = LaunchConfiguration('password')
    log_level = LaunchConfiguration('log_level')
    maxage_conn = LaunchConfiguration('maxage_conn')
    ntrip_version = LaunchConfiguration('ntrip_version')
    gga_fix_topic = LaunchConfiguration('gga_fix_topic')
    gga_send_period_sec = LaunchConfiguration('gga_send_period_sec')

    return launch.LaunchDescription([
        DeclareLaunchArgument('use_https', default_value=TextSubstitution(text='false')),
        DeclareLaunchArgument('host', default_value=TextSubstitution(text='nrtk-swepos.lm.se')),
        DeclareLaunchArgument('port', default_value=TextSubstitution(text='80')),
        DeclareLaunchArgument('mountpoint', default_value=TextSubstitution(text='MSM_GNSS')),
        DeclareLaunchArgument('username', default_value=TextSubstitution(text='')),
        DeclareLaunchArgument('password', default_value=TextSubstitution(text='')),
        DeclareLaunchArgument('log_level', default_value=TextSubstitution(text='INFO')),
        DeclareLaunchArgument('maxage_conn', default_value=TextSubstitution(text='30')),
        DeclareLaunchArgument('ntrip_version', default_value=TextSubstitution(text='')),
        DeclareLaunchArgument('gga_fix_topic', default_value=TextSubstitution(text='')),
        DeclareLaunchArgument('gga_send_period_sec', default_value=TextSubstitution(text='1.0')),
        ComposableNodeContainer(
            name='ntrip_client_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            arguments=['--ros-args', '--log-level', log_level],
            composable_node_descriptions=[
                ComposableNode(
                    package='ntrip_client_node',
                    plugin='ublox_dgnss::NTRIPClientNode',
                    name='ntrip_client',
                    parameters=[{
                        'use_https': use_https,
                        'host': host,
                        'port': port,
                        'mountpoint': mountpoint,
                        'username': username,
                        'password': password,
                        'log_level': log_level,
                        'maxage_conn': maxage_conn,
                        'ntrip_version': ntrip_version,
                        'gga_fix_topic': gga_fix_topic,
                        'gga_send_period_sec': gga_send_period_sec,
                    }],
                ),
            ],
        ),
    ])
