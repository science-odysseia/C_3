import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class DataSubscriber(Node):

    def __init__(self):
        super().__init__('data_subscriber')

        self.subscription = self.create_subscription(
            Int32,
            '/random_number',
            self.callback,
            10
        )

    def callback(self, msg):

        self.get_logger().info(
            f'receive : {msg.data}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = DataSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()