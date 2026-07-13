#!/usr/bin/env python3
"""Publish dynamic odom->base_link and static base_link->modem TF from SMARC GPS."""

from geometry_msgs.msg import TransformStamped
from geographic_msgs.msg import GeoPoint
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ublox_stick_tf import stick_transforms as st


class GpsTfPublisher(Node):
    """Broadcast {robot}/odom -> {robot}/base_link + modem offset."""

    def __init__(self):
        super().__init__('gps_tf_publisher')

        self.declare_parameter('frame_prefix', 'stick_1')
        self.declare_parameter('latlon_topic', 'smarc/latlon')
        self.declare_parameter('heading_topic', 'smarc/heading')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('modem_x_offset', 0.0)
        self.declare_parameter('modem_y_offset', 0.0)
        self.declare_parameter('modem_z_offset', -1.57)

        prefix = self.get_parameter('frame_prefix').value
        latlon_topic = self.get_parameter('latlon_topic').value
        heading_topic = self.get_parameter('heading_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        modem_x = float(self.get_parameter('modem_x_offset').value)
        modem_y = float(self.get_parameter('modem_y_offset').value)
        modem_z = float(self.get_parameter('modem_z_offset').value)

        self.odom_frame = '{}/odom'.format(prefix)
        self.base_frame = '{}/base_link'.format(prefix)
        self.modem_frame = '{}/modem'.format(prefix)

        self.utm_ready = False
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.heading_rad = 0.0
        self.have_position = False
        self.have_heading = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(GeoPoint, latlon_topic, self.latlon_callback, 10)
        self.create_subscription(Float32, heading_topic, self.heading_callback, 10)

        self._send_modem_static_tf(modem_x, modem_y, modem_z)

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.1
        self.timer = self.create_timer(period, self.publish_tf)

        self.get_logger().info(
            'GPS TF publisher: {} -> {} -> {} (waiting for utm -> {} datum)'.format(
                self.odom_frame, self.base_frame, self.modem_frame, self.odom_frame))

    def _send_modem_static_tf(self, x, y, z):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.modem_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(tf)

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
        if not (self.utm_ready and self.have_position and self.have_heading):
            return

        yaw = st.heading_rad_to_yaw_enu(self.heading_rad)
        qx, qy, qz, qw = st.yaw_to_quat_xyzw(yaw)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.altitude
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
