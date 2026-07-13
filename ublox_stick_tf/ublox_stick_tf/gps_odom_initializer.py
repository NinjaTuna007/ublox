#!/usr/bin/env python3
"""Anchor utm_{zone} -> utm -> {robot}/odom from the first valid GPS fix."""

import math

from geometry_msgs.msg import TransformStamped
from geographic_msgs.msg import GeoPoint
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from ublox_stick_tf import stick_transforms as st


class GpsOdomInitializer(Node):
    """Lock a local odom datum on the first smarc/latlon fix."""

    def __init__(self):
        super().__init__('gps_odom_initializer')

        self.declare_parameter('frame_prefix', 'stick_1')
        self.declare_parameter('latlon_topic', 'smarc/latlon')
        self.declare_parameter('update_rate', 1.0)
        self.declare_parameter('verbose', False)

        self.frame_prefix = self.get_parameter('frame_prefix').value
        latlon_topic = self.get_parameter('latlon_topic').value
        update_rate = float(self.get_parameter('update_rate').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        self.odom_frame = '{}/odom'.format(self.frame_prefix)
        self.origin_set = False
        self.transforms = []

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(GeoPoint, latlon_topic, self.latlon_callback, 10)

        period = 1.0 / update_rate if update_rate > 0.0 else 1.0
        self.timer = self.create_timer(period, self.republish)

        self.get_logger().info(
            'GPS odom initializer waiting for fix on "{}"'.format(latlon_topic))

    def latlon_callback(self, msg: GeoPoint):
        if self.origin_set:
            return
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        if abs(msg.latitude) < 1e-9 and abs(msg.longitude) < 1e-9:
            return

        easting, northing, zone, letter = st.latlon_to_utm(
            msg.latitude, msg.longitude)
        utm_zone_frame = st.utm_zone_frame_id(zone, letter)
        now = self.get_clock().now().to_msg()

        zone_to_utm = TransformStamped()
        zone_to_utm.header.stamp = now
        zone_to_utm.header.frame_id = utm_zone_frame
        zone_to_utm.child_frame_id = 'utm'
        zone_to_utm.transform.rotation.w = 1.0

        utm_to_odom = TransformStamped()
        utm_to_odom.header.stamp = now
        utm_to_odom.header.frame_id = 'utm'
        utm_to_odom.child_frame_id = self.odom_frame
        utm_to_odom.transform.translation.x = easting
        utm_to_odom.transform.translation.y = northing
        utm_to_odom.transform.translation.z = 0.0
        utm_to_odom.transform.rotation.w = 1.0

        self.transforms = [zone_to_utm, utm_to_odom]
        self.origin_set = True
        self.static_broadcaster.sendTransform(self.transforms)

        self.get_logger().info(
            'Datum locked at lat={:.7f} lon={:.7f} -> {} '
            'easting={:.2f} northing={:.2f}; broadcasting {} -> utm -> {}'.format(
                msg.latitude, msg.longitude, utm_zone_frame,
                easting, northing, utm_zone_frame, self.odom_frame))

    def republish(self):
        if not self.origin_set:
            return
        now = self.get_clock().now().to_msg()
        for tf in self.transforms:
            tf.header.stamp = now
        self.static_broadcaster.sendTransform(self.transforms)
        if self.verbose:
            self.get_logger().debug('Republished static datum TF')


def main(args=None):
    rclpy.init(args=args)
    node = GpsOdomInitializer()
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
