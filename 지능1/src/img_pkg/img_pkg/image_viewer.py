#!/usr/bin/env python3

import os
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageViewer(Node):

    def __init__(self):
        super().__init__('image_viewer')

        self.bridge = CvBridge()

        # 이미지 저장 폴더
        self.save_dir = r"/home/yswbulb/turtlebot4_ws/img_savings"
        os.makedirs(self.save_dir, exist_ok=True)

        self.subscription = self.create_subscription(
            Image,
            '/robot3/oakd/rgb/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Subscribed to /robot3/oakd/rgb/image_raw')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            cv2.imshow("OAK-D Camera", frame)

            key = cv2.waitKey(1) & 0xFF

            # ESC 종료
            if key == 27:
                self.get_logger().info("ESC pressed. Shutting down...")
                rclpy.shutdown()

            # R 키 저장
            elif key == ord('r'):
                filename = datetime.now().strftime("%Y%m%d_%H%M%S.jpg")
                filepath = os.path.join(self.save_dir, filename)

                cv2.imwrite(filepath, frame)

                self.get_logger().info(
                    f"Image saved: {filepath}"
                )

        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = ImageViewer()

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