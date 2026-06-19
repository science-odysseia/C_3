#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CompressedViewer(Node):

    def __init__(self):
        super().__init__('compressed_viewer')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot3/oakd/rgb/image_raw/compressed',
            self.callback,
            10
        )

        self.get_logger().info("Compressed image viewer started")

    def callback(self, msg):
        # CompressedImage -> OpenCV 이미지
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        cv2.imshow("TurtleBot4 OAK-D RGB", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = CompressedViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()