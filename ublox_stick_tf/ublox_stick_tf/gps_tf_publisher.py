#!/usr/bin/env python3
"""Publish dynamic odom->base_link TF from SMARC GPS (UTM datum from gps_odom_initializer)."""

from geometry_msgs.msg import TransformStamped
from geographic_msgs.msg import GeoPoint
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ublox_stick_tf import stick_transforms as st


class GpsTfPublisher(Node):
    """Broadcast {robot}/odom -> {robot}/base_link in local ENU coordinates."""

    def __init__(self):
        super().__init__('gps_tf_publisher')

        self.declare_parameter('frame_prefix', 'stick_1')
        self.declare_parameter('latlon_topic', 'smarc/latlon')
        self.declare_parameter('heading_topic', 'smarc/heading')
        self.declare_parameter('publish_rate', 10.0)

        prefix = self.get_parameter('frame_prefix').value
        latlon_topic = self.get_parameter('latlon_topic').value
        heading_topic = self.get_parameter('heading_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.odom_frame = '{}/odom'.format(prefix)
        self.base_frame = '{}/base_link'.format(prefix)
        self.modem_frame = st.modem_frame_id(prefix)

        self.utm_ready = False
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.z_offset = None
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.heading_rad = 0.0
        self.have_position = False
        self.have_heading = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(GeoPoint, latlon_topic, self.latlon_callback, 10)
        self.create_subscription(Float32, heading_topic, self.heading_callback, 10)

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.1
        self.timer = self.create_timer(period, self.publish_tf)

        self.get_logger().info(
            'GPS TF publisher: {} -> {} (modem at {}; waiting for utm -> {} datum)'.format(
                self.odom_frame, self.base_frame, self.modem_frame, self.odom_frame))

    def _ensure_utm_offset(self):
        if self.utm_ready:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(
                'utm', self.odom_frame, rclpy.time.Time())
        except Exception:
            return False
        self.x_offset = tf.transform.translation.x
        self.y_offset = tf.transform.translation.y
        self.utm_ready = True
        self.get_logger().info(
            'Datum offset acquired: easting={:.2f} northing={:.2f}'.format(
                self.x_offset, self.y_offset))
        return True

    def _ensure_z_offset(self):
        if self.z_offset is not None:
            return True
        if not math.isfinite(self.altitude):
            return False
        self.z_offset = self.altitude
        self.get_logger().info(
            'Altitude datum locked: {:.2f} m (odom Z will be relative)'.format(
                self.z_offset))
        return True

    def latlon_callback(self, msg: GeoPoint):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        if not self._ensure_utm_offset():
            return
        easting, northing, _, _ = st.latlon_to_utm(msg.latitude, msg.longitude)
        self.x = easting - self.x_offset
        self.y = northing - self.y_offset
        self.have_position = True

    def heading_callback(self, msg: Float32):
        self.heading_rad = msg.data
        self.have_heading = True

    def publish_tf(self):
        if not (self.utm_ready and self._ensure_z_offset() and self.have_position and self.have_heading):
            return

        yaw = st.heading_rad_to_yaw_enu(self.heading_rad)
        qx, qy, qz, qw = st.yaw_to_quat_xyzw(yaw)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.altitude - self.z_offset
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = GpsTfPublisher()
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
