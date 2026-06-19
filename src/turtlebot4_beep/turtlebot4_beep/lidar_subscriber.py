import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')

        self.sub = self.create_subscription(
            LaserScan,
            '/robot3/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        # 정면 거리
        front_index = len(msg.ranges) // 2
        front_distance = msg.ranges[front_index]

        self.get_logger().info(f'Front distance: {front_distance:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()