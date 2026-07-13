"""Launch stick GPS stack: ublox_dgnss + NavSatFix + SMARC + TF + NTRIP."""
import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    if not frame_id:
        frame_id = '{}/base_link'.format(robot_name)
    device_family = LaunchConfiguration('device_family').perform(context)
    device_serial_string = LaunchConfiguration('device_serial_string').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    ubx_config_file = LaunchConfiguration('ubx_config_file').perform(context)
    modem_z_offset = float(LaunchConfiguration('modem_z_offset').perform(context))

    dgnss_params = [
        {'UBX_CONFIG_FILE': ubx_config_file},
        {'DEVICE_FAMILY': device_family},
        {'DEVICE_SERIAL_STRING': device_serial_string},
        {'FRAME_ID': frame_id},
        {'CFG_NAVSPG_DYNMODEL': 5},
        {'CFG_NAVSPG_FIXMODE': 3},
        {'CFG_NAVHPG_DGNSSMODE': 3},
        {'CFG_RATE_MEAS': 40},
        {'CFG_RATE_NAV': 1},
        {'CFG_USBOUTPROT_NMEA': False},
        {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 5},
        {'CFG_MSGOUT_UBX_NAV_COV_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_VELNED_USB': 1},
        {'CFG_MSGOUT_UBX_RXM_COR_USB': 1},
        {'CFG_MSGOUT_UBX_RXM_RTCM_USB': 0},
        {'RTCM_REPUBLISH_ENABLED': False},
        {'UBX_RXM_RTCM_ENABLED': False},
    ]

    ublox_dgnss_container = ComposableNodeContainer(
        name='ublox_dgnss_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', log_level],
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_dgnss_node',
                plugin='ublox_dgnss::UbloxDGNSSNode',
                name='ublox_dgnss',
                namespace=robot_name,
                parameters=dgnss_params,
            ),
        ],
    )

    navsatfix_container = ComposableNodeContainer(
        name='ublox_nav_sat_fix_hp_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', log_level],
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_nav_sat_fix_hp_node',
                plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
                name='ublox_nav_sat_fix_hp',
                namespace=robot_name,
                remappings=[
                    ('fix', 'ublox_gps_node/fix'),
                ],
            ),
        ],
    )

    smarc_node = Node(
        package='ublox_stick_smarc',
        executable='stick_smarc_publisher',
        name='stick_smarc_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'fix_topic': 'ublox_gps_node/fix',
            'velned_topic': 'ubx_nav_vel_ned',
            'status_topic': 'ubx_nav_status',
        }],
    )

    odom_init_node = Node(
        package='ublox_stick_tf',
        executable='gps_odom_initializer',
        name='gps_odom_initializer',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'frame_prefix': robot_name,
            'latlon_topic': 'smarc/latlon',
            'modem_z_offset': modem_z_offset,
        }],
    )

    tf_node = Node(
        package='ublox_stick_tf',
        executable='gps_tf_publisher',
        name='gps_tf_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'frame_prefix': robot_name,
            'latlon_topic': 'smarc/latlon',
            'heading_topic': 'smarc/heading',
        }],
    )

    ntrip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ublox_stick_bringup'),
                'launch',
                'ntrip_client.launch.py',
            ]),
        ]),
        launch_arguments={
            'use_https': LaunchConfiguration('use_https'),
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'mountpoint': LaunchConfiguration('mountpoint'),
            'username': LaunchConfiguration('username'),
            'password': LaunchConfiguration('password'),
            'log_level': log_level,
            'ntrip_version': LaunchConfiguration('ntrip_version'),
            'gga_fix_topic': '/{}/ublox_gps_node/fix'.format(robot_name),
            'gga_send_period_sec': LaunchConfiguration('gga_send_period_sec'),
        }.items(),
    )

    return [
        ublox_dgnss_container,
        navsatfix_container,
        smarc_node,
        odom_init_node,
        tf_node,
        ntrip_launch,
    ]


def generate_launch_description():
    pkg_dgnss = get_package_share_directory('ublox_dgnss')
    default_ubx_config = os.path.join(
        pkg_dgnss, 'config', 'x20p_stick_sea_rover.toml')

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='stick_1',
            description='Robot namespace (e.g. stick_1)',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='',
            description='frame_id in GPS message headers (default: {robot_name}/base_link)',
        ),
        DeclareLaunchArgument('device_family', default_value='X20P'),
        DeclareLaunchArgument('device_serial_string', default_value=''),
        DeclareLaunchArgument('log_level', default_value='INFO'),
        DeclareLaunchArgument(
            'ubx_config_file',
            default_value=default_ubx_config,
            description='UBX TOML config (x20p_stick_sea_rover.toml for stick)',
        ),
        DeclareLaunchArgument('modem_z_offset', default_value='-1.57'),
        DeclareLaunchArgument('use_https', default_value='false'),
        DeclareLaunchArgument('host', default_value='nrtk-swepos.lm.se'),
        DeclareLaunchArgument('port', default_value='80'),
        DeclareLaunchArgument('mountpoint', default_value='MSM_GNSS'),
        DeclareLaunchArgument('username', default_value=''),
        DeclareLaunchArgument('password', default_value=''),
        DeclareLaunchArgument('ntrip_version', default_value=''),
        DeclareLaunchArgument('gga_send_period_sec', default_value='1.0'),
        OpaqueFunction(function=_launch_setup),
    ])
