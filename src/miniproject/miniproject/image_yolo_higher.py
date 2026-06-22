#!/usr/bin/env python3

import os
import signal
import rclpy
import cv2
import numpy as np
import torch

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from ultralytics import YOLO


MY_CAR_CLASS_ID = 0
WHITE_THRESHOLD = 180
MIN_WHITE_RATIO = 0.1
DETECT_SECONDS = 0.5


class YoloCompressedViewer(Node):

    def __init__(self):
        super().__init__('yolo_compressed_viewer')

        self.model = YOLO('/home/yswbulb/turtlebot4_ws/best_amr.pt')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot3/oakd/rgb/image_raw/compressed',
            self.callback,
            10
        )

        self.detect_pub = self.create_publisher(
            Bool,
            '/robot3/is_detected',
            10
        )

        self.center_pub = self.create_publisher(
            Point,
            '/robot3/detected_center',
            10
        )

        self.detect_start_time = None
        self.is_detected = False

        self.timer = self.create_timer(
            0.1,
            self.publish_detect_state
        )

        self.get_logger().info("YOLO compressed image viewer started")

    def publish_detect_state(self):
        msg = Bool()
        msg.data = self.is_detected
        self.detect_pub.publish(msg)

    def publish_detected_center(self, center_x, center_y):
        if not self.is_detected:
            return

        msg = Point()
        msg.x = float(center_x)
        msg.y = float(center_y)
        msg.z = 0.0

        self.center_pub.publish(msg)

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
        white_ratio = np.sum(white_mask) / white_mask.size

        return white_ratio

    def get_box_center(self, xyxy):
        x1, y1, x2, y2 = xyxy
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0

        return center_x, center_y

    def update_detection_state(self, detected_now):
        now = self.get_clock().now().nanoseconds / 1e9

        if detected_now:
            if self.detect_start_time is None:
                self.detect_start_time = now

            elapsed = now - self.detect_start_time

            if elapsed >= DETECT_SECONDS:
                self.is_detected = True
        else:
            self.detect_start_time = None
            self.is_detected = False

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.update_detection_state(False)
            return

        results = self.model(frame, verbose=False, conf=0.8)
        result = results[0]
        boxes = result.boxes

        detected_now = False
        target_center = None

        if boxes is not None and len(boxes) > 0:
            my_car_indices = []

            for i, box in enumerate(boxes):
                cls_id = int(box.cls[0])

                if cls_id == MY_CAR_CLASS_ID:
                    my_car_indices.append(i)

            if len(my_car_indices) > 0:
                best_idx = None
                best_white_ratio = -1.0

                for i in my_car_indices:
                    xyxy = boxes[i].xyxy[0].cpu().numpy()
                    white_ratio = self.get_white_ratio(frame, xyxy)

                    if white_ratio < MIN_WHITE_RATIO:
                        continue

                    if white_ratio > best_white_ratio:
                        best_white_ratio = white_ratio
                        best_idx = i

                if best_idx is not None:
                    detected_now = True

                    xyxy = boxes[best_idx].xyxy[0].cpu().numpy()
                    target_center = self.get_box_center(xyxy)

                    keep_indices = []

                    for i in range(len(boxes)):
                        cls_id = int(boxes[i].cls[0])

                        if cls_id == MY_CAR_CLASS_ID and i != best_idx:
                            continue

                        keep_indices.append(i)

                    keep_indices = torch.tensor(
                        keep_indices,
                        dtype=torch.long,
                        device=boxes.data.device
                    )

                    result.boxes = boxes[keep_indices]
                else:
                    keep_indices = []

                    for i in range(len(boxes)):
                        cls_id = int(boxes[i].cls[0])

                        if cls_id == MY_CAR_CLASS_ID:
                            continue

                        keep_indices.append(i)

                    keep_indices = torch.tensor(
                        keep_indices,
                        dtype=torch.long,
                        device=boxes.data.device
                    )

                    result.boxes = boxes[keep_indices]

        self.update_detection_state(detected_now)
        self.publish_detect_state()

        if self.is_detected and target_center is not None:
            center_x, center_y = target_center
            self.publish_detected_center(center_x, center_y)

        annotated_frame = result.plot()

        status_text = f"is_detected: {self.is_detected}"
        cv2.putText(
            annotated_frame,
            status_text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0) if self.is_detected else (0, 0, 255),
            2
        )

        if self.is_detected and target_center is not None:
            center_x, center_y = target_center

            cv2.circle(
                annotated_frame,
                (int(center_x), int(center_y)),
                6,
                (255, 0, 0),
                -1
            )

            center_text = f"center: ({int(center_x)}, {int(center_y)})"
            cv2.putText(
                annotated_frame,
                center_text,
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 0, 0),
                2
            )

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