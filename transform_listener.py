#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import geometry_msgs.msg

class TFEcho(Node):

    def __init__(self, reference_frame, target_frame):
        super().__init__('tf2_echo')
        self.tf_buffer = Buffer()
        self.reference_frame = reference_frame
        self.target_frame = target_frame

        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher_ = self.create_publisher(geometry_msgs.msg.TransformStamped, 'wrist_mounted_camera_transform', 10)
        self.create_timer(0.1, self.publish_tf)

    def publish_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.reference_frame, self.target_frame, rclpy.time.Time())
            self.publisher_.publish(trans)
            self.get_logger().info("Translation: {}, Rotation: {}".format(trans.transform.translation, trans.transform.rotation))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warning(str(e))


def main(args=None):
    rclpy.init(args=args)

    reference_frame = "world"
    target_frame = "wrist_mounted_camera_color_frame"

    tf_echo_node = TFEcho(reference_frame, target_frame)

    rclpy.spin(tf_echo_node)

    tf_echo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
