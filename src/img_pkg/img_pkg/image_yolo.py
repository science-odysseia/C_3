#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


class YoloCompressedViewer(Node):

    def __init__(self):
        super().__init__('yolo_compressed_viewer')

        # YOLO 모델 로드
        self.model = YOLO(r'/home/yswbulb/turtlebot4_ws/best.pt')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot3/oakd/rgb/image_raw/compressed',
            self.callback,
            10
        )

        self.get_logger().info("YOLO compressed image viewer started")

    def callback(self, msg):
        # CompressedImage -> OpenCV 이미지
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        # YOLO 추론
        results = self.model(frame, verbose=False, conf=0.8)

        # 탐지 결과 그리기
        annotated_frame = results[0].plot()

        cv2.imshow("TurtleBot4 YOLO Detection", annotated_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = YoloCompressedViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()