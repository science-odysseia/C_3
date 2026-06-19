#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
import os
from datetime import datetime

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CompressedViewer(Node):

    def __init__(self):
        super().__init__('compressed_viewer')

        self.save_dir = "/home/yswbulb/turtlebot4_images"
        os.makedirs(self.save_dir, exist_ok=True)

        self.recording = False
        self.video_writer = None
        self.video_path = None

        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot3/oakd/rgb/image_raw/compressed',
            self.callback,
            10
        )

        self.get_logger().info("Compressed image viewer started")
        self.get_logger().info("Press 'r' to start/stop recording")

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        cv2.imshow("TurtleBot4 OAK-D RGB", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            if not self.recording:
                self.start_recording(frame)
            else:
                self.stop_recording()

        if self.recording and self.video_writer is not None:
            self.video_writer.write(frame)

    def start_recording(self, frame):
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"oakd_rgb_record_{now}.mp4"
        self.video_path = os.path.join(self.save_dir, filename)

        height, width, _ = frame.shape

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = 30.0

        self.video_writer = cv2.VideoWriter(
            self.video_path,
            fourcc,
            fps,
            (width, height)
        )

        if not self.video_writer.isOpened():
            self.get_logger().error("VideoWriter open failed")
            self.video_writer = None
            self.video_path = None
            return

        self.recording = True
        self.get_logger().info(f"Recording started: {self.video_path}")

    def stop_recording(self):
        self.recording = False

        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None

        self.get_logger().info(f"Recording saved: {self.video_path}")
        self.video_path = None


def main(args=None):
    rclpy.init(args=args)

    node = CompressedViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node.recording:
        node.stop_recording()

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()