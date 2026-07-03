#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion

import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

import numpy as np
import cv2
import threading
import math


ROBOT_NS = '/robot3'


class DepthToMap(Node):

    def __init__(self):
        super().__init__('depth_to_map_node')

        self.K = None
        self.lock = threading.Lock()

        self.rgb_topic = f'{ROBOT_NS}/oakd/rgb/image_raw/compressed'
        self.depth_topic = f'{ROBOT_NS}/oakd/stereo/image_raw/compressedDepth'
        self.info_topic = f'{ROBOT_NS}/oakd/rgb/camera_info'

        self.depth_image = None
        self.rgb_image = None
        self.clicked_point = None
        self.display_image = None

        # compressedDepth는 msg header에 frame_id가 있음
        self.camera_frame = None

        self.logged_intrinsics = False
        self.logged_rgb_shape = False
        self.logged_depth_shape = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.navigator = TurtleBot4Navigator()
        self.get_logger().info('Navigator created. Click RGB image to send goal.')

        self.create_subscription(
            CameraInfo,
            self.info_topic,
            self.camera_info_callback,
            1
        )

        self.create_subscription(
            CompressedImage,
            self.rgb_topic,
            self.rgb_callback,
            10
        )

        self.create_subscription(
            CompressedImage,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.gui_thread_stop = threading.Event()
        self.gui_thread = threading.Thread(
            target=self.gui_loop,
            daemon=True
        )
        self.gui_thread.start()

        self.get_logger().info(f'Subscribed RGB topic: {self.rgb_topic}')
        self.get_logger().info(f'Subscribed depth topic: {self.depth_topic}')
        self.get_logger().info(f'Subscribed camera info topic: {self.info_topic}')

        self.get_logger().info('TF Tree 안정화 시작. 5초 후 클릭 변환을 시작합니다.')
        self.start_timer = self.create_timer(5.0, self.start_transform)

    def start_transform(self):
        self.get_logger().info('TF Tree 안정화 완료. 클릭하면 goal을 전송합니다.')
        self.timer = self.create_timer(0.2, self.display_images)
        self.start_timer.cancel()

    def camera_info_callback(self, msg):
        with self.lock:
            self.K = np.array(msg.k).reshape(3, 3)

        if not self.logged_intrinsics:
            self.get_logger().info(
                f'Camera intrinsics received: '
                f'fx={self.K[0, 0]:.2f}, '
                f'fy={self.K[1, 1]:.2f}, '
                f'cx={self.K[0, 2]:.2f}, '
                f'cy={self.K[1, 2]:.2f}'
            )
            self.logged_intrinsics = True

    def rgb_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if rgb is None or rgb.size == 0:
                return

            with self.lock:
                self.rgb_image = rgb

            if not self.logged_rgb_shape:
                self.get_logger().info(f'RGB image decoded: {rgb.shape}')
                self.logged_rgb_shape = True

        except Exception as e:
            self.get_logger().error(f'Compressed RGB decode failed: {e}')

    def depth_callback(self, msg):
        try:
            if len(msg.data) <= 12:
                return

            # compressedDepth 헤더 12바이트 제거
            depth_data = np.frombuffer(msg.data[12:], np.uint8)
            depth = cv2.imdecode(depth_data, cv2.IMREAD_UNCHANGED)

            if depth is None or depth.size == 0:
                return

            with self.lock:
                self.depth_image = depth
                self.camera_frame = msg.header.frame_id

            if not self.logged_depth_shape:
                self.get_logger().info(
                    f'CompressedDepth image decoded: {depth.shape}, dtype={depth.dtype}, frame={msg.header.frame_id}'
                )
                self.logged_depth_shape = True

        except Exception as e:
            self.get_logger().error(f'CompressedDepth decode failed: {e}')

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        with self.lock:
            if self.rgb_image is None:
                return

            rgb_h, rgb_w = self.rgb_image.shape[:2]

            if x >= rgb_w:
                self.get_logger().warn('왼쪽 RGB 화면을 클릭하세요.')
                return

            if y < 0 or y >= rgb_h:
                return

            self.clicked_point = (x, y)

        self.get_logger().info(f'Clicked RGB pixel: ({x}, {y})')

    def display_images(self):
        with self.lock:
            rgb = self.rgb_image.copy() if self.rgb_image is not None else None
            depth = self.depth_image.copy() if self.depth_image is not None else None
            click = self.clicked_point
            frame_id = self.camera_frame
            K = self.K.copy() if self.K is not None else None

        if rgb is None or depth is None or frame_id is None or K is None:
            return

        try:
            rgb_display = rgb.copy()

            depth_normalized = cv2.normalize(
                depth,
                None,
                0,
                255,
                cv2.NORM_MINMAX
            ).astype(np.uint8)

            depth_colored = cv2.cvtColor(
                depth_normalized,
                cv2.COLOR_GRAY2BGR
            )

            if click is not None:
                x, y = click

                h, w = depth.shape[:2]

                if x < 0 or x >= w or y < 0 or y >= h:
                    self.get_logger().warn('Clicked point is outside depth image.')
                    with self.lock:
                        self.clicked_point = None
                    return

                z = float(depth[y, x]) / 1000.0

                if 0.2 < z < 5.0:
                    fx, fy = K[0, 0], K[1, 1]
                    cx, cy = K[0, 2], K[1, 2]

                    X = (x - cx) * z / fx
                    Y = (y - cy) * z / fy
                    Z = z

                    pt_camera = PointStamped()
                    pt_camera.header.stamp = Time().to_msg()
                    pt_camera.header.frame_id = frame_id
                    pt_camera.point.x = X
                    pt_camera.point.y = Y
                    pt_camera.point.z = Z

                    pt_map = self.tf_buffer.transform(
                        pt_camera,
                        'map',
                        timeout=Duration(seconds=1.0)
                    )

                    self.get_logger().info(
                        f'Map coordinate: '
                        f'({pt_map.point.x:.2f}, '
                        f'{pt_map.point.y:.2f}, '
                        f'{pt_map.point.z:.2f})'
                    )

                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = 'map'
                    goal_pose.header.stamp = self.get_clock().now().to_msg()

                    goal_pose.pose.position.x = pt_map.point.x
                    goal_pose.pose.position.y = pt_map.point.y
                    goal_pose.pose.position.z = 0.0

                    yaw = 0.0
                    goal_pose.pose.orientation = Quaternion(
                        x=0.0,
                        y=0.0,
                        z=math.sin(yaw / 2.0),
                        w=math.cos(yaw / 2.0)
                    )

                    self.navigator.goToPose(goal_pose)
                    self.get_logger().info('Sent navigation goal.')

                else:
                    self.get_logger().warn(f'Invalid depth value: {z:.2f} m')

                cv2.circle(rgb_display, (x, y), 4, (0, 255, 0), -1)

                text = f'{z:.2f} m' if 0.2 < z < 5.0 else 'Invalid'
                cv2.putText(
                    depth_colored,
                    text,
                    (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )
                cv2.circle(depth_colored, (x, y), 4, (255, 255, 255), -1)

                with self.lock:
                    self.clicked_point = None

            combined = np.hstack((rgb_display, depth_colored))

            with self.lock:
                self.display_image = combined.copy()

        except Exception as e:
            self.get_logger().warn(f'TF or goal error: {e}')
            with self.lock:
                self.clicked_point = None

    def gui_loop(self):
        window_name = 'RGB (left) | CompressedDepth (right)'

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 480)
        cv2.moveWindow(window_name, 100, 100)
        cv2.setMouseCallback(window_name, self.mouse_callback)

        while not self.gui_thread_stop.is_set():
            with self.lock:
                img = self.display_image.copy() if self.display_image is not None else None

            if img is not None:
                cv2.imshow(window_name, img)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    self.get_logger().info('Shutdown requested by user.')
                    self.gui_thread_stop.set()
                    rclpy.shutdown()
                    break
            else:
                cv2.waitKey(10)


def main():
    rclpy.init(args=[
        '--ros-args',
        '-r', '__ns:=/robot3',
        '-r', '/tf:=/robot3/tf',
        '-r', '/tf_static:=/robot3/tf_static',
    ])

    node = DepthToMap()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.gui_thread_stop.set()

    if node.gui_thread.is_alive():
        node.gui_thread.join()

    node.destroy_node()
    cv2.destroyAllWindows()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()