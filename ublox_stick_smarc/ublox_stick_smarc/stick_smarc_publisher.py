#!/usr/bin/env python3
"""Publish smarc/latlon, smarc/speed, smarc/heading from ublox_dgnss fix + VELNED."""

import math
from typing import Optional

import rclpy
from geographic_msgs.msg import GeoPoint
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32
from ublox_ubx_msgs.msg import UBXNavStatus, UBXNavVelNED


class StickSmarcPublisher(Node):
    """Convert ublox_dgnss NavSatFix and UBX-NAV-VELNED to SMARC topics."""

    def __init__(self):
        super().__init__('stick_smarc_publisher')

        self.declare_parameter('fix_topic', 'ublox_nav_sat_fix_hp/fix')
        self.declare_parameter('velned_topic', 'ubx_nav_vel_ned')
        self.declare_parameter('status_topic', 'ubx_nav_status')
        self.declare_parameter('nmea_heading_topic', 'gnss_nmea_heading')

        fix_topic = self.get_parameter('fix_topic').value
        velned_topic = self.get_parameter('velned_topic').value
        status_topic = self.get_parameter('status_topic').value
        nmea_heading_topic = self.get_parameter('nmea_heading_topic').value

        gps_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.fix_sub = self.create_subscription(
            NavSatFix, fix_topic, self.fix_callback, gps_qos)
        self.velned_sub = self.create_subscription(
            UBXNavVelNED, velned_topic, self.velned_callback, gps_qos)
        self.status_sub = self.create_subscription(
            UBXNavStatus, status_topic, self.status_callback, gps_qos)
        self.nmea_heading_sub = self.create_subscription(
            Float32, nmea_heading_topic, self.nmea_heading_callback, gps_qos)

        self.latlon_pub = self.create_publisher(GeoPoint, 'smarc/latlon', 10)
        self.altitude_pub = self.create_publisher(Float32, 'smarc/altitude', 10)
        self.speed_pub = self.create_publisher(Float32, 'smarc/speed', 10)
        self.heading_pub = self.create_publisher(Float32, 'smarc/heading', 10)

        self._rtk_active = False
        self._velned_seen = False
        self.get_logger().info('Stick SMARC publisher initialized')

    def status_callback(self, msg: UBXNavStatus):
        self._rtk_active = bool(msg.diff_soln and msg.carr_soln_valid)

    def fix_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            return

        latlon = GeoPoint()
        latlon.latitude = msg.latitude
        latlon.longitude = msg.longitude
        latlon.altitude = msg.altitude
        self.latlon_pub.publish(latlon)

        alt_msg = Float32()
        alt_msg.data = float(msg.altitude)
        self.altitude_pub.publish(alt_msg)

        if self._rtk_active and msg.status.status < NavSatStatus.STATUS_GBAS_FIX:
            self.get_logger().debug(
                f'RTK active in ubx_nav_status but NavSatFix status={msg.status.status}')

    def velned_callback(self, msg: UBXNavVelNED):
        self._velned_seen = True
        vel_n = msg.vel_n * 1e-3
        vel_e = msg.vel_e * 1e-3
        vel_d = msg.vel_d * 1e-3

        if msg.g_speed > 0:
            speed_mps = msg.g_speed * 0.01
        else:
            speed_mps = math.sqrt(vel_n * vel_n + vel_e * vel_e + vel_d * vel_d)

        speed_msg = Float32()
        speed_msg.data = float(speed_mps)
        self.speed_pub.publish(speed_msg)

        if msg.heading != -1:
            heading_deg = msg.heading * 1e-5
            heading_rad = math.radians(heading_deg)
        else:
            heading_rad = math.atan2(vel_e, vel_n)

        heading_msg = Float32()
        heading_msg.data = float(heading_rad)
        self.heading_pub.publish(heading_msg)

    def nmea_heading_callback(self, msg: Float32):
        if self._velned_seen:
            return
        self.heading_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StickSmarcPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
