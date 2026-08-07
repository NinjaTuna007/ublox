#!/usr/bin/env python3
"""Publish static UTM datum chain and base_link->modem_link in one /tf_static message."""

import math

from geometry_msgs.msg import TransformStamped
from geographic_msgs.msg import GeoPoint
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from ublox_stick_tf import stick_transforms as st


class GpsOdomInitializer(Node):
    """Lock utm_{zone} -> utm -> {robot}/odom and publish modem_link offset."""

    def __init__(self):
        super().__init__('gps_odom_initializer')

        self.declare_parameter('frame_prefix', 'stick_1')
        self.declare_parameter('latlon_topic', 'smarc/latlon')
        self.declare_parameter('update_rate', 1.0)
        self.declare_parameter('verbose', False)
        self.declare_parameter('modem_x_offset', 0.0)
        self.declare_parameter('modem_y_offset', 0.0)
        self.declare_parameter('modem_z_offset', -2.4)

        self.frame_prefix = self.get_parameter('frame_prefix').value
        latlon_topic = self.get_parameter('latlon_topic').value
        update_rate = float(self.get_parameter('update_rate').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        self.odom_frame = '{}/odom'.format(self.frame_prefix)
        self.base_frame = '{}/base_link'.format(self.frame_prefix)
        self.modem_frame = st.modem_frame_id(self.frame_prefix)
        self.origin_set = False
        self.transforms = []

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(GeoPoint, latlon_topic, self.latlon_callback, 10)

        period = 1.0 / update_rate if update_rate > 0.0 else 1.0
        self.timer = self.create_timer(period, self.republish)

        self.get_logger().info(
            'GPS odom initializer waiting for fix on "{}"'.format(latlon_topic))

    def _modem_static_transform(self, x, y, z):
        tf = TransformStamped()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.modem_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        return [tf]

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

        self.transforms = [zone_to_utm, utm_to_odom] + self._modem_static_transform(
            float(self.get_parameter('modem_x_offset').value),
            float(self.get_parameter('modem_y_offset').value),
            float(self.get_parameter('modem_z_offset').value),
        )
        self.origin_set = True
        self._publish_static()

        self.get_logger().info(
            'Datum locked at lat={:.7f} lon={:.7f} -> {} '
            'easting={:.2f} northing={:.2f}; broadcasting {} -> utm -> {} -> {} -> {}'.format(
                msg.latitude, msg.longitude, utm_zone_frame,
                easting, northing, utm_zone_frame, self.odom_frame,
                self.base_frame, self.modem_frame))

    def _publish_static(self):
        now = self.get_clock().now().to_msg()
        for tf in self.transforms:
            tf.header.stamp = now
        self.static_broadcaster.sendTransform(self.transforms)

    def republish(self):
        if not self.origin_set:
            return
        self._publish_static()
        if self.verbose:
            self.get_logger().debug('Republished static UTM + modem_link TF')


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
