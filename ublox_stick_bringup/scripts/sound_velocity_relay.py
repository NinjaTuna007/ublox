#!/usr/bin/env python3
"""Relay sound_vel Float64 to waraps/sensor/sound_velocity String."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String


class SoundVelocityRelay(Node):
    def __init__(self):
        super().__init__('sound_velocity_relay')
        self.declare_parameter('input_topic', 'sound_vel')
        self.declare_parameter('output_topic', 'waraps/sensor/sound_velocity')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(String, output_topic, 10)
        self.create_subscription(Float64, input_topic, self.callback, 10)

    def callback(self, msg: Float64):
        out = String()
        out.data = '{:.3f}'.format(msg.data)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SoundVelocityRelay()
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
