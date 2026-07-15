#!/usr/bin/env python3
"""Publish dynamic odom->base_link TF from GPS fix + SMARC heading."""

from geometry_msgs.msg import TransformStamped
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
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
        self.declare_parameter('fix_topic', 'ublox_gps_node/fix')
        self.declare_parameter('heading_topic', 'smarc/heading')
        self.declare_parameter('publish_rate', 40.0)

        prefix = self.get_parameter('frame_prefix').value
        fix_topic = self.get_parameter('fix_topic').value
        heading_topic = self.get_parameter('heading_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.odom_frame = '{}/odom'.format(prefix)
        self.base_frame = '{}/base_link'.format(prefix)
        self.modem_frame = st.modem_frame_id(prefix)

        self.utm_ready = False
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.z_offset = None
        self.x = 0.0
        self.y = 0.0
        self.altitude = 0.0
        self.heading_rad = 0.0
        self.have_position = False
        self.have_heading = False
        self.have_fix_stamp = False
        self.last_fix_stamp = None
        self._heading_fallback_logged = False

        gps_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(NavSatFix, fix_topic, self.fix_callback, gps_qos)
        self.create_subscription(Float32, heading_topic, self.heading_callback, 10)

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.025
        self.timer = self.create_timer(period, self.publish_tf)

        self.get_logger().info(
            'GPS TF publisher: {} -> {} (modem at {}; fix topic: {})'.format(
                self.odom_frame, self.base_frame, self.modem_frame, fix_topic))

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

    def fix_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        if not self._ensure_utm_offset():
            return
        easting, northing, _, _ = st.latlon_to_utm(msg.latitude, msg.longitude)
        self.x = easting - self.x_offset
        self.y = northing - self.y_offset
        self.altitude = msg.altitude
        self.have_position = True
        self.last_fix_stamp = msg.header.stamp
        self.have_fix_stamp = True

    def heading_callback(self, msg: Float32):
        self.heading_rad = msg.data
        self.have_heading = True

    def publish_tf(self):
        if not (self.utm_ready and self._ensure_z_offset() and self.have_position):
            return

        if self.have_heading:
            yaw = st.heading_rad_to_yaw_enu(self.heading_rad)
        else:
            yaw = 0.0
            if not self._heading_fallback_logged:
                self.get_logger().warn(
                    'Publishing {} -> {} without heading; yaw=0 until smarc/heading arrives'.format(
                        self.odom_frame, self.base_frame))
                self._heading_fallback_logged = True

        qx, qy, qz, qw = st.yaw_to_quat_xyzw(yaw)

        tf = TransformStamped()
        if self.have_fix_stamp:
            tf.header.stamp = self.last_fix_stamp
        else:
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
