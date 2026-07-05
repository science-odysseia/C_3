#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import CameraInfo, CompressedImage
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions

import numpy as np
import cv2
import threading


class DepthToMap(Node):
    def __init__(self):
        super().__init__('depth_to_map_node')

        self.K = None
        self.lock = threading.Lock()

        ns = self.get_namespace()

        self.depth_topic = f'{ns}/oakd/stereo/image_raw/compressedDepth'
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'

        self.depth_image = None
        self.rgb_image = None
        self.clicked_point = None
        self.shutdown_requested = False

        self.display_image = None
        self.gui_thread_stop = threading.Event()
        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

        self.navigator = TurtleBot4Navigator()

        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped(
            [0.0, 0.0],
            TurtleBot4Directions.NORTH
        )
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        self.logged_intrinsics = False
        self.logged_rgb_shape = False
        self.logged_depth_shape = False

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        self.create_subscription(CompressedImage, self.depth_topic, self.depth_callback, 1)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 1)

        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")
        self.timer = self.create_timer(0.2, self.display_images)
        self.start_timer.cancel()

    def camera_info_callback(self, msg):
        with self.lock:
            self.K = np.array(msg.k).reshape(3, 3)

            if not self.logged_intrinsics:
                self.get_logger().info(
                    f"Camera intrinsics received: "
                    f"fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, "
                    f"cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}"
                )
                self.logged_intrinsics = True

    def depth_callback(self, msg):
        try:
            if len(msg.data) <= 12:
                return

            depth_data = np.frombuffer(msg.data[12:], np.uint8)
            depth = cv2.imdecode(depth_data, cv2.IMREAD_UNCHANGED)

            if depth is None or depth.size == 0:
                self.get_logger().error("Failed to decode compressedDepth image")
                return

            if not self.logged_depth_shape:
                self.get_logger().info(
                    f"CompressedDepth decoded: shape={depth.shape}, dtype={depth.dtype}"
                )
                self.logged_depth_shape = True

            with self.lock:
                self.depth_image = depth

        except Exception as e:
            self.get_logger().error(f"CompressedDepth decode failed: {e}")

    def rgb_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if rgb is None or rgb.size == 0:
                self.get_logger().error("Decoded RGB image is empty")
                return

            if not self.logged_rgb_shape:
                self.get_logger().info(f"RGB image decoded: {rgb.shape}")
                self.logged_rgb_shape = True

            with self.lock:
                self.rgb_image = rgb

        except Exception as e:
            self.get_logger().error(f"Compressed RGB decode failed: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            with self.lock:
                self.clicked_point = (x, y)

            self.get_logger().info(f"Clicked RGB pixel: ({x}, {y})")

    def display_images(self):
        with self.lock:
            rgb = self.rgb_image.copy() if self.rgb_image is not None else None
            depth = self.depth_image.copy() if self.depth_image is not None else None
            click = self.clicked_point

        if rgb is None or depth is None:
            return

        try:
            rgb_display = rgb.copy()
            depth_display = depth.copy()

            if len(depth_display.shape) == 3:
                depth_display = depth_display[:, :, 0]

            # Depth 표시용 흑백 8bit 이미지
            depth_gray = cv2.normalize(
                depth_display,
                None,
                0,
                255,
                cv2.NORM_MINMAX
            ).astype(np.uint8)

            # RGB와 hstack하려면 depth도 3채널로 변환해야 함
            depth_bgr = cv2.cvtColor(depth_gray, cv2.COLOR_GRAY2BGR)

            if depth_bgr.shape[:2] != rgb_display.shape[:2]:
                depth_bgr = cv2.resize(
                    depth_bgr,
                    (rgb_display.shape[1], rgb_display.shape[0])
                )

            if click:
                x, y = click

                if 0 <= x < rgb_display.shape[1] and 0 <= y < rgb_display.shape[0]:
                    dx = int(x * depth_display.shape[1] / rgb_display.shape[1])
                    dy = int(y * depth_display.shape[0] / rgb_display.shape[0])

                    if 0 <= dx < depth_display.shape[1] and 0 <= dy < depth_display.shape[0]:
                        z = float(depth_display[dy, dx]) / 1000.0
                        text = f"{z:.2f} m" if 0.2 < z < 5.0 else "Invalid"

                        cv2.putText(
                            rgb_display,
                            '+',
                            (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 255, 0),
                            2
                        )
                        cv2.circle(rgb_display, (x, y), 4, (0, 255, 0), -1)

                        cv2.putText(
                            depth_bgr,
                            text,
                            (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 255, 255),
                            2
                        )
                        cv2.circle(depth_bgr, (x, y), 4, (255, 255, 255), -1)

            combined = np.hstack((rgb_display, depth_bgr))

            with self.lock:
                self.display_image = combined.copy()

        except Exception as e:
            self.get_logger().warn(f"Image display error: {e}")

    def gui_loop(self):
        window_name = 'RGB (left) | Depth Gray (right)'

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 480)
        cv2.moveWindow(window_name, 100, 100)
        cv2.setMouseCallback(window_name, self.mouse_callback)

        while not self.gui_thread_stop.is_set():
            img = None

            with self.lock:
                if self.display_image is not None:
                    img = self.display_image.copy()

            if img is not None:
                cv2.imshow(window_name, img)
                key = cv2.waitKey(1)

                if key == ord('q'):
                    self.get_logger().info("Shutdown requested by user.")
                    self.navigator.dock()
                    self.shutdown_requested = True
                    self.gui_thread_stop.set()

                    if rclpy.ok():
                        rclpy.shutdown()
            else:
                cv2.waitKey(10)


def main():
    rclpy.init()

    node = DepthToMap()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.gui_thread_stop.set()
    node.gui_thread.join()
    node.destroy_node()
    cv2.destroyAllWindows()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()