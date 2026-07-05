#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class RGBDepthClickViewer(Node):

    def __init__(self):
        super().__init__('rgb_depth_click_viewer')

        self.bridge = CvBridge()
        self.latest_depth = None
        self.depth_encoding = None

        self.rgb_sub = self.create_subscription(
            CompressedImage,
            '/robot3/oakd/rgb/image_raw/compressed',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/robot3/oakd/stereo/image_raw',
            self.depth_callback,
            10
        )

        self.window_name = "TurtleBot4 RGB - click point"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info("RGB + Depth click viewer started")

    def depth_callback(self, msg):
        self.depth_encoding = msg.encoding
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

    def rgb_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.latest_depth is None:
            print("아직 depth 이미지가 안 들어옴")
            return

        depth = self.latest_depth

        h, w = depth.shape[:2]

        if x < 0 or x >= w or y < 0 or y >= h:
            print("클릭 좌표가 depth 이미지 범위를 벗어남")
            return

        value = depth[y, x]

        if value == 0 or np.isnan(value):
            print(f"({x}, {y}) 거리값 없음")
            return

        if self.depth_encoding == '16UC1':
            distance_m = value / 1000.0
        elif self.depth_encoding == '32FC1':
            distance_m = float(value)
        else:
            distance_m = float(value)
            print(f"알 수 없는 encoding: {self.depth_encoding}")

        print(f"클릭 좌표 ({x}, {y}) 거리: {distance_m:.3f} m")


def main(args=None):
    rclpy.init(args=args)

    node = RGBDepthClickViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()