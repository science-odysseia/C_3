#!/usr/bin/env python3

import os
import signal
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


class YoloCompressedViewer(Node):

    def __init__(self):
        super().__init__('yolo_compressed_viewer')

        self.model = YOLO('/home/yswbulb/turtlebot4_ws/best.pt')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot3/oakd/rgb/image_raw/compressed',
            self.callback,
            10
        )

        self.get_logger().info("YOLO compressed image viewer started")

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        results = self.model(frame, verbose=False, conf=0.8)
        annotated_frame = results[0].plot()

        cv2.imshow("TurtleBot4 YOLO Detection", annotated_frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            self.get_logger().info("C key pressed. Shutting down...")
            cv2.destroyAllWindows()
            os.kill(os.getpid(), signal.SIGINT)


def main(args=None):
    rclpy.init(args=args)
    node = YoloCompressedViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()