import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

import random


class DataPublisher(Node):

    def __init__(self):
        super().__init__('data_publisher')

        self.publisher_ = self.create_publisher(
            Int32,
            '/random_number',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_data
        )

    def publish_data(self):

        msg = Int32()
        msg.data = random.randint(1, 10)

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'publish : {msg.data}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = DataPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()