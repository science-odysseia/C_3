#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion, Twist
from std_msgs.msg import Bool

import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

import numpy as np
import cv2
import threading
import math


ROBOT_NS = '/robot3'
STOP_DISTANCE = 1.2   # m, 이 거리 이하가 되면 정지


class CenterToNavGoal(Node):

    def __init__(self):
        super().__init__('center_to_nav_goal_node')

        self.K = None
        self.lock = threading.Lock()

        self.rgb_topic = f'{ROBOT_NS}/oakd/rgb/image_raw/compressed'
        self.depth_topic = f'{ROBOT_NS}/oakd/stereo/image_raw/compressedDepth'
        self.info_topic = f'{ROBOT_NS}/oakd/rgb/camera_info'
        self.detect_topic = f'{ROBOT_NS}/is_detected'
        self.center_topic = f'{ROBOT_NS}/detected_center'
        self.cmd_vel_topic = f'{ROBOT_NS}/cmd_vel'

        self.rgb_image = None
        self.depth_image = None
        self.camera_frame = None
        self.detected_center = None
        self.is_detected = False
        self.display_image = None

        self.goal_sent = False
        self.stopped = False
        self.current_distance = None

        self.logged_intrinsics = False
        self.logged_rgb_shape = False
        self.logged_depth_shape = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.navigator = TurtleBot4Navigator()
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_logger().info('Navigator created. First YOLO center will be used as goal.')

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 10)
        self.create_subscription(CompressedImage, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(Bool, self.detect_topic, self.detect_callback, 10)
        self.create_subscription(Point, self.center_topic, self.center_callback, 10)

        self.gui_thread_stop = threading.Event()
        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

        self.get_logger().info(f'Subscribed RGB topic: {self.rgb_topic}')
        self.get_logger().info(f'Subscribed depth topic: {self.depth_topic}')
        self.get_logger().info(f'Subscribed camera info topic: {self.info_topic}')
        self.get_logger().info(f'Subscribed detected topic: {self.detect_topic}')
        self.get_logger().info(f'Subscribed center topic: {self.center_topic}')
        self.get_logger().info(f'Publishing stop cmd_vel topic: {self.cmd_vel_topic}')

        self.get_logger().info('TF Tree 안정화 시작. 5초 후 중심점 처리를 시작합니다.')
        self.start_timer = self.create_timer(5.0, self.start_transform)

    def start_transform(self):
        self.get_logger().info('TF Tree 안정화 완료. 최초 goal 전송 및 거리 감시 시작.')
        self.timer = self.create_timer(0.2, self.process_center)
        self.display_timer = self.create_timer(0.1, self.update_display)
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
        np_arr = np.frombuffer(msg.data, np.uint8)
        rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if rgb is None or rgb.size == 0:
            return

        with self.lock:
            self.rgb_image = rgb

        if not self.logged_rgb_shape:
            self.get_logger().info(f'RGB image decoded: {rgb.shape}')
            self.logged_rgb_shape = True

    def depth_callback(self, msg):
        if len(msg.data) <= 12:
            return

        depth_data = np.frombuffer(msg.data[12:], np.uint8)
        depth = cv2.imdecode(depth_data, cv2.IMREAD_UNCHANGED)

        if depth is None or depth.size == 0:
            return

        with self.lock:
            self.depth_image = depth
            self.camera_frame = msg.header.frame_id

        if not self.logged_depth_shape:
            self.get_logger().info(
                f'CompressedDepth image decoded: {depth.shape}, '
                f'dtype={depth.dtype}, frame={msg.header.frame_id}'
            )
            self.logged_depth_shape = True

    def detect_callback(self, msg):
        with self.lock:
            self.is_detected = msg.data

            if not self.is_detected:
                self.detected_center = None
                self.current_distance = None

    def center_callback(self, msg):
        with self.lock:
            if self.is_detected:
                self.detected_center = (int(msg.x), int(msg.y))

    def stop_robot(self):
        try:
            self.navigator.cancelTask()
        except Exception as e:
            self.get_logger().warn(f'cancelTask failed: {e}')

        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        self.stopped = True
        self.get_logger().info('Stop distance reached. Navigation cancelled and robot stopped.')

    def process_center(self):
        with self.lock:
            depth = self.depth_image.copy() if self.depth_image is not None else None
            center = self.detected_center
            frame_id = self.camera_frame
            K = self.K.copy() if self.K is not None else None
            is_detected = self.is_detected

        if self.stopped:
            self.cmd_vel_pub.publish(Twist())
            return

        if not is_detected or center is None:
            return

        if depth is None or frame_id is None or K is None:
            return

        x, y = center
        h, w = depth.shape[:2]

        if x < 0 or x >= w or y < 0 or y >= h:
            self.get_logger().warn('Detected center is outside depth image.')
            return

        z = float(depth[y, x]) / 1000.0

        with self.lock:
            self.current_distance = z

        # 추가
        self.get_logger().info(
            f'Current distance: {z:.2f} m'
        )

        if not (0.2 < z < 5.0):
            self.get_logger().warn(
                f'Invalid depth value at center: {z:.2f} m'
            )
            return

        if z <= STOP_DISTANCE:
            self.stop_robot()
            return

        if self.goal_sent:
            return

        try:
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
            self.goal_sent = True

            self.get_logger().info(
                f'Sent FIRST goal from YOLO center ({x}, {y}) -> '
                f'map ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}), '
                f'distance={z:.2f} m'
            )

        except Exception as e:
            self.get_logger().warn(f'TF or goal error: {e}')

    def update_display(self):
        with self.lock:
            rgb = self.rgb_image.copy() if self.rgb_image is not None else None
            depth = self.depth_image.copy() if self.depth_image is not None else None
            center = self.detected_center
            is_detected = self.is_detected
            distance = self.current_distance
            goal_sent = self.goal_sent
            stopped = self.stopped

        if rgb is None or depth is None:
            return

        rgb_display = rgb.copy()

        depth_normalized = cv2.normalize(
            depth,
            None,
            0,
            255,
            cv2.NORM_MINMAX
        ).astype(np.uint8)

        depth_display = cv2.cvtColor(
            depth_normalized,
            cv2.COLOR_GRAY2BGR
        )

        status = f'goal_sent: {goal_sent}, stopped: {stopped}'
        cv2.putText(
            rgb_display,
            status,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if goal_sent else (0, 0, 255),
            2
        )

        if distance is not None:
            cv2.putText(
                rgb_display,
                f'distance: {distance:.2f} m / stop: {STOP_DISTANCE:.2f} m',
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 0),
                2
            )

        if is_detected and center is not None:
            x, y = center

            cv2.circle(rgb_display, (x, y), 6, (0, 255, 0), -1)
            cv2.putText(
                rgb_display,
                f'center: ({x}, {y})',
                (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            if 0 <= x < depth.shape[1] and 0 <= y < depth.shape[0]:
                z = float(depth[y, x]) / 1000.0

                cv2.circle(depth_display, (x, y), 6, (255, 255, 255), -1)
                cv2.putText(
                    depth_display,
                    f'{z:.2f} m',
                    (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

        combined = np.hstack((rgb_display, depth_display))

        with self.lock:
            self.display_image = combined.copy()

    def gui_loop(self):
        window_name = 'RGB (left) | Depth Gray (right)'

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 480)
        cv2.moveWindow(window_name, 100, 100)

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

    node = CenterToNavGoal()
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