#!/usr/bin/env python3

import os
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

        # 저장 폴더
        self.save_dir = os.path.expanduser("~/turtlebot4_images")
        os.makedirs(self.save_dir, exist_ok=True)

        self.latest_frame = None
        self.image_count = 0
        self.max_images = 100

        self.get_logger().info("Compressed image viewer started")
        self.get_logger().info(f"Save directory: {self.save_dir}")

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        self.latest_frame = frame

        cv2.imshow("TurtleBot4 OAK-D RGB", frame)

        key = cv2.waitKey(1) & 0xFF

        # r 키 누르면 저장
        if key == ord('r'):

            self.image_count += 1

            filename = os.path.join(
                self.save_dir,
                f"image_{self.image_count:03d}.jpg"
            )

            cv2.imwrite(filename, self.latest_frame)

            self.get_logger().info(
                f"Saved [{self.image_count}/{self.max_images}] : {filename}"
            )

            # 100장 저장되면 자동 종료
            if self.image_count >= self.max_images:
                self.get_logger().info("100 images saved. Shutting down...")
                rclpy.shutdown()

        # q 키 누르면 종료
        elif key == ord('q'):
            self.get_logger().info("User requested shutdown.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = CompressedViewer()

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