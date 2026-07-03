#!/usr/bin/env python3

import os
import signal

import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


MY_CAR_CLASS_ID = 0
WHITE_THRESHOLD = 180
CONF_THRESHOLD = 0.8


class WhiteRatioChecker(Node):

    def __init__(self):
        super().__init__('white_ratio_checker')

        self.model = YOLO('/home/yswbulb/turtlebot4_ws/best_amr.pt')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot3/oakd/rgb/image_raw/compressed',
            self.callback,
            10
        )

        self.get_logger().info('White ratio checker started')

    def get_white_ratio(self, frame, xyxy):
        h, w = frame.shape[:2]

        x1, y1, x2, y2 = map(int, xyxy)

        x1 = max(0, min(x1, w - 1))
        x2 = max(0, min(x2, w - 1))
        y1 = max(0, min(y1, h - 1))
        y2 = max(0, min(y2, h - 1))

        if x2 <= x1 or y2 <= y1:
            return 0.0

        roi = frame[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        white_mask = gray >= WHITE_THRESHOLD

        return np.sum(white_mask) / white_mask.size

    def callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        results = self.model(
            frame,
            verbose=False,
            conf=CONF_THRESHOLD
        )

        annotated_frame = frame.copy()

        boxes = results[0].boxes

        if boxes is not None and len(boxes) > 0:

            for i, box in enumerate(boxes):

                cls_id = int(box.cls[0])

                if cls_id != MY_CAR_CLASS_ID:
                    continue

                conf = float(box.conf[0])

                xyxy = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = map(int, xyxy)

                white_ratio = self.get_white_ratio(
                    frame,
                    xyxy
                )

                self.get_logger().info(
                    f'car index={i}, conf={conf:.2f}, white_ratio={white_ratio:.3f}'
                )

                cv2.rectangle(
                    annotated_frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    annotated_frame,
                    f'white={white_ratio:.3f}',
                    (x1, max(30, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

        cv2.imshow(
            'White Ratio Checker',
            annotated_frame
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            self.get_logger().info(
                'C key pressed. Shutting down...'
            )

            cv2.destroyAllWindows()
            os.kill(
                os.getpid(),
                signal.SIGINT
            )


def main(args=None):

    rclpy.init(args=args)

    node = WhiteRatioChecker()

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