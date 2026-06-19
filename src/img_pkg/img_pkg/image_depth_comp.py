#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CompressedDepthViewer(Node):

    def __init__(self):
        super().__init__('compressed_depth_viewer')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot3/oakd/stereo/image_raw/compressedDepth',
            self.callback,
            10
        )

        self.get_logger().info("CompressedDepth viewer started")

    def callback(self, msg):
        if len(msg.data) <= 12:
            return

        # compressedDepth 헤더 제거
        depth_data = np.frombuffer(msg.data[12:], np.uint8)

        depth_img = cv2.imdecode(depth_data, cv2.IMREAD_UNCHANGED)

        if depth_img is None:
            return

        # 표시용으로만 8bit 변환
        depth_vis = cv2.normalize(
            depth_img,
            None,
            0,
            255,
            cv2.NORM_MINMAX
        ).astype(np.uint8)

        cv2.imshow("CompressedDepth", depth_vis)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = CompressedDepthViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()