import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'markers', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        marker= Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ""

        marker.type = 1 # Cuboid
        marker.id = 0
        marker.action = 0


        # Scale
        marker.scale.x = 10.0
        marker.scale.y = 10.0
        marker.scale.z = 10.0

        # Color
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Pose
        marker.pose.position.x = 3.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        self.publisher_.publish(marker)
        self.get_logger().info('Publishing: "%s"' % marker)


def main(args=None):
    rclpy.init(args=args)

    node = MinimalPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
