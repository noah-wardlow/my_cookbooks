import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg

class TFListenerNode(Node):

    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher_ = self.create_publisher(geometry_msgs.msg.TransformStamped, 'base_link_transform', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'world', self.get_clock().now().to_msg())
            self.publisher_.publish(transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

def main(args=None):
    rclpy.init(args=args)

    node = TFListenerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

