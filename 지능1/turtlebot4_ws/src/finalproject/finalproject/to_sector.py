#!/usr/bin/env python3

import math
import time
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage
from geometry_msgs.msg import Twist, Point, PointStamped, PoseStamped, Quaternion

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

import cv2
import numpy as np

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator
)

from nav2_simple_commander.robot_navigator import TaskResult


ROBOT_NS = '/robot3'

MODE_TOPIC = f'{ROBOT_NS}/mode'
TARGET_MODE = 'to_sector'

MODE_STABLE_SECONDS = 1.2
PRE_STOP_SECONDS = 1.0

AMCL_TOPIC = f'{ROBOT_NS}/amcl_pose'
THRESHOLD_TOPIC = f'{ROBOT_NS}/battery_threshold'
BATTERY_TOPIC = f'{ROBOT_NS}/battery_state'
GOHOME_SECTOR_TOPIC = f'{ROBOT_NS}/gohome_sector'
SECTOR_HOME_REQ_TOPIC = f'{ROBOT_NS}/sector_home_req'
CMD_VEL_TOPIC = f'{ROBOT_NS}/cmd_vel'
DETECTED_BOTTOM_TOPIC = f'{ROBOT_NS}/detected_bottom'

STOP_PUBLISH_HZ = 50.0
STOP_PERIOD = 1.0 / STOP_PUBLISH_HZ

STOP_DISTANCE = 1.2
PROCESS_PERIOD = 0.5
DISPLAY_PERIOD = 0.1

LOST_DETECT_SECONDS = 1.0
SEARCH_ANGULAR_SPEED = 0.3
SEARCH_TURNS = 2.0
SEARCH_MAX_ROTATION = SEARCH_TURNS * 2.0 * math.pi

FINAL_SEARCH_TURNS = 1.0
FINAL_SEARCH_MAX_ROTATION = FINAL_SEARCH_TURNS * 2.0 * math.pi


class MissionMonitorNode(Node):

    def __init__(self):
        super().__init__('mission_monitor_node')

        self.threshold = None
        self.battery_percentage = None

        self.paused = False
        self.pause_requested = False
        self.stop_requested = False

        self.tracking_requested = False

        self.current_mode = None
        self.target_mode_start_time = None

        self.mission_started = False
        self.mode_stop_requested = False
        self.mode_stop_handled = False

        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.gohome_sector_pub = self.create_publisher(Bool, GOHOME_SECTOR_TOPIC, 10)
        self.sector_home_req_pub = self.create_publisher(Bool, SECTOR_HOME_REQ_TOPIC, 10)

        self.create_subscription(String, MODE_TOPIC, self.mode_callback, 10)
        self.create_subscription(Float32, THRESHOLD_TOPIC, self.threshold_callback, 10)
        self.create_subscription(BatteryState, BATTERY_TOPIC, self.battery_callback, 10)
        self.create_subscription(Point, DETECTED_BOTTOM_TOPIC, self.detected_bottom_callback, 10)

        self.timer = self.create_timer(STOP_PERIOD, self.timer_callback)

    def mode_callback(self, msg):
        mode = msg.data.strip()
        now = self.get_clock().now()

        if mode != self.current_mode:
            self.get_logger().info(f'Mode received: {mode}')

        self.current_mode = mode

        if mode == TARGET_MODE:
            if self.target_mode_start_time is None:
                self.target_mode_start_time = now
                self.get_logger().info(
                    f'Mode "{TARGET_MODE}" received. Stability timer started.'
                )
        else:
            self.target_mode_start_time = None

            if self.mission_started and not self.mode_stop_requested:
                self.mode_stop_requested = True
                self.get_logger().warn(
                    f'Mode changed to "{mode}". Stop current to_sector mission immediately.'
                )

    def reset_for_waiting(self):
        self.mission_started = False
        self.mode_stop_requested = False
        self.mode_stop_handled = False
        self.tracking_requested = False
        self.pause_requested = False

        if self.current_mode == TARGET_MODE:
            self.target_mode_start_time = self.get_clock().now()
        else:
            self.target_mode_start_time = None

        self.publish_gohome_sector(False)
        self.publish_sector_home_req(False)

    def get_target_mode_elapsed(self):
        if self.current_mode != TARGET_MODE:
            return 0.0

        if self.target_mode_start_time is None:
            return 0.0

        return (
            self.get_clock().now() - self.target_mode_start_time
        ).nanoseconds / 1e9

    def is_target_mode_stable(self):
        return self.get_target_mode_elapsed() >= MODE_STABLE_SECONDS

    def mark_mission_started(self):
        self.mission_started = True
        self.mode_stop_requested = False
        self.mode_stop_handled = False
        self.tracking_requested = False
        self.publish_sector_home_req(False)
        self.get_logger().info('to_sector mission started.')

    def is_mode_stop_requested(self):
        return self.mode_stop_requested

    def threshold_callback(self, msg):
        self.threshold = float(msg.data)
        self.update_pause_state()

    def battery_callback(self, msg):
        self.battery_percentage = msg.percentage * 100.0
        self.update_pause_state()

    def detected_bottom_callback(self, msg):
        if not self.mission_started:
            return

        if self.mode_stop_requested:
            return

        if msg.x != -1.0 or msg.y != -1.0:
            if not self.tracking_requested:
                self.tracking_requested = True
                self.get_logger().warn(
                    f'Human detected: ({msg.x:.1f}, {msg.y:.1f}) -> tracking mode requested'
                )

    def has_battery_state(self):
        return self.threshold is not None and self.battery_percentage is not None

    def update_pause_state(self):
        if self.threshold is None or self.battery_percentage is None:
            return

        prev_paused = self.paused
        self.paused = self.battery_percentage <= self.threshold

        if self.paused and not prev_paused:
            self.pause_requested = True
            self.get_logger().warn(
                f'Battery pause ON: {self.battery_percentage:.1f}% <= {self.threshold:.1f}%'
            )

        elif not self.paused and prev_paused:
            self.get_logger().info(
                f'Battery pause OFF: {self.battery_percentage:.1f}% > {self.threshold:.1f}%'
            )

    def timer_callback(self):
        if self.mode_stop_requested:
            return

        if not self.mission_started:
            return

        self.publish_gohome_sector(self.paused)

        if self.paused:
            self.publish_zero_cmd()

    def publish_zero_cmd(self):
        self.cmd_vel_pub.publish(Twist())

    def publish_gohome_sector(self, value):
        msg = Bool()
        msg.data = value
        self.gohome_sector_pub.publish(msg)

    def publish_sector_home_req(self, value):
        msg = Bool()
        msg.data = value
        self.sector_home_req_pub.publish(msg)

    def force_stop_robot(self, duration_sec=0.2, gohome_value=False):
        count = int(duration_sec * STOP_PUBLISH_HZ)

        for _ in range(count):
            self.publish_zero_cmd()
            self.publish_gohome_sector(gohome_value)
            time.sleep(STOP_PERIOD)

    def is_paused(self):
        return self.paused

    def is_tracking_requested(self):
        return self.tracking_requested


def spin_monitor_node(executor, monitor_node):
    while rclpy.ok() and not monitor_node.stop_requested:
        executor.spin_once(timeout_sec=0.01)


def wait_for_to_sector_mode(monitor_node):
    monitor_node.get_logger().info(
        f'Waiting until /robot3/mode is "{TARGET_MODE}" for {MODE_STABLE_SECONDS:.1f}s...'
    )

    while rclpy.ok():
        if monitor_node.current_mode == TARGET_MODE:
            elapsed = monitor_node.get_target_mode_elapsed()

            if elapsed <= PRE_STOP_SECONDS:
                monitor_node.publish_zero_cmd()
                monitor_node.publish_gohome_sector(False)
                monitor_node.publish_sector_home_req(False)

            if elapsed >= MODE_STABLE_SECONDS:
                monitor_node.get_logger().info(
                    f'Mode "{TARGET_MODE}" stable for {MODE_STABLE_SECONDS:.1f}s.'
                )
                return True

        time.sleep(0.05)

    return False


def cancel_current_goal_now(navigator, monitor_node, nav_lock, stop_sec=1.0):
    try:
        with nav_lock:
            navigator.cancelTask()
    except Exception as e:
        navigator.info(f'cancelTask failed: {e}')

    monitor_node.force_stop_robot(stop_sec, gohome_value=False)


def cancel_monitor(navigator, monitor_node, nav_lock):
    while rclpy.ok() and not monitor_node.stop_requested:

        if monitor_node.is_mode_stop_requested() and not monitor_node.mode_stop_handled:
            monitor_node.mode_stop_handled = True
            navigator.info('Mode monitor: cancel current goal immediately.')
            cancel_current_goal_now(navigator, monitor_node, nav_lock, stop_sec=1.0)
            monitor_node.publish_sector_home_req(False)

        if monitor_node.tracking_requested and not monitor_node.is_mode_stop_requested():
            navigator.info('Tracking monitor: cancel current goal immediately.')

            try:
                with nav_lock:
                    navigator.cancelTask()
            except Exception as e:
                navigator.info(f'cancelTask failed: {e}')

            monitor_node.force_stop_robot(1.0, gohome_value=True)

        if monitor_node.pause_requested and not monitor_node.is_mode_stop_requested():
            monitor_node.pause_requested = False

            navigator.info('Battery monitor: cancel current goal immediately.')

            try:
                with nav_lock:
                    navigator.cancelTask()
            except Exception as e:
                navigator.info(f'cancelTask failed: {e}')

            monitor_node.force_stop_robot(1.0, gohome_value=True)

        if monitor_node.is_paused() and monitor_node.mission_started and not monitor_node.is_mode_stop_requested():
            monitor_node.publish_zero_cmd()
            monitor_node.publish_gohome_sector(True)

        time.sleep(0.02)


def get_current_xy_once():
    result = subprocess.run(
        ['ros2', 'topic', 'echo', AMCL_TOPIC, '--once'],
        capture_output=True,
        text=True
    )

    output = result.stdout.splitlines()

    x = None
    y = None

    for i, line in enumerate(output):
        stripped = line.strip()

        if stripped.startswith('position:'):
            for j in range(i + 1, min(i + 5, len(output))):
                pos_line = output[j].strip()

                if pos_line.startswith('x:'):
                    x = float(pos_line.split(':')[1].strip())
                elif pos_line.startswith('y:'):
                    y = float(pos_line.split(':')[1].strip())

        if x is not None and y is not None:
            break

    return x, y


def wait_for_current_xy(navigator, monitor_node):
    current_x = None
    current_y = None

    while rclpy.ok():
        if monitor_node.is_mode_stop_requested():
            return None, None

        navigator.info(f'Reading current pose from {AMCL_TOPIC}...')

        current_x, current_y = get_current_xy_once()

        if current_x is not None and current_y is not None:
            navigator.info(f'Current Position: x={current_x:.3f}, y={current_y:.3f}')
            return current_x, current_y

        navigator.info('AMCL pose not received yet. Waiting...')
        monitor_node.publish_zero_cmd()
        monitor_node.publish_gohome_sector(False)
        monitor_node.publish_sector_home_req(False)
        time.sleep(0.5)

    return None, None


def distance_xy(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def go_to_pose_with_interrupt(navigator, monitor_node, nav_lock, pose, name):
    while rclpy.ok():

        if monitor_node.is_mode_stop_requested():
            cancel_current_goal_now(navigator, monitor_node, nav_lock, stop_sec=1.0)
            return 'WAIT_MODE'

        if monitor_node.is_tracking_requested():
            return 'TRACKING'

        while rclpy.ok() and monitor_node.is_paused():
            if monitor_node.is_mode_stop_requested():
                cancel_current_goal_now(navigator, monitor_node, nav_lock, stop_sec=1.0)
                return 'WAIT_MODE'

            if monitor_node.is_tracking_requested():
                return 'TRACKING'

            monitor_node.force_stop_robot(0.2, gohome_value=True)
            time.sleep(0.05)

        monitor_node.publish_gohome_sector(False)
        monitor_node.publish_sector_home_req(False)

        navigator.info(f'Going to {name}...')

        with nav_lock:
            navigator.startToPose(pose)

        was_paused = False

        while rclpy.ok():

            if monitor_node.is_mode_stop_requested():
                navigator.info('Mode changed during navigation. Cancel immediately.')
                cancel_current_goal_now(navigator, monitor_node, nav_lock, stop_sec=1.0)
                return 'WAIT_MODE'

            if monitor_node.is_tracking_requested():
                navigator.info('Human detected during navigation. Switching to tracking mode.')
                return 'TRACKING'

            if monitor_node.is_paused():
                was_paused = True
                monitor_node.force_stop_robot(0.2, gohome_value=True)
                time.sleep(0.05)
                continue

            if was_paused:
                was_paused = False
                navigator.info(f'Restarting current goal: {name}')

                with nav_lock:
                    navigator.startToPose(pose)

            if navigator.isTaskComplete():
                result = navigator.getResult()
                navigator.info(f'{name} navigation result: {result}')

                if result == TaskResult.SUCCEEDED:
                    return 'SUCCEEDED'
                return 'FAILED'

            time.sleep(0.02)

    return 'FAILED'


def run_pose_sequence(navigator, monitor_node, nav_lock, route_names, point_data):
    for name in route_names:
        if monitor_node.is_mode_stop_requested():
            cancel_current_goal_now(navigator, monitor_node, nav_lock, stop_sec=1.0)
            return 'WAIT_MODE'

        pose = navigator.getPoseStamped(
            point_data[name]['xy'],
            point_data[name]['direction']
        )

        result = go_to_pose_with_interrupt(
            navigator,
            monitor_node,
            nav_lock,
            pose,
            name
        )

        if result != 'SUCCEEDED':
            return result

    return 'SUCCEEDED'


def final_sector_search(navigator, monitor_node, nav_lock):
    navigator.info(
        'Target reached without detection. Start one-turn final sector search.'
    )

    try:
        with nav_lock:
            navigator.cancelTask()
    except Exception as e:
        navigator.info(f'cancelTask failed before final sector search: {e}')

    monitor_node.publish_sector_home_req(False)

    start_time = time.time()
    max_search_time = FINAL_SEARCH_MAX_ROTATION / abs(SEARCH_ANGULAR_SPEED)

    while rclpy.ok():
        if monitor_node.is_mode_stop_requested():
            cancel_current_goal_now(navigator, monitor_node, nav_lock, stop_sec=1.0)
            monitor_node.publish_sector_home_req(False)
            return 'WAIT_MODE'

        if monitor_node.is_tracking_requested():
            navigator.info('Human detected during final sector search.')
            cancel_current_goal_now(navigator, monitor_node, nav_lock, stop_sec=1.0)
            monitor_node.publish_sector_home_req(False)
            return 'TRACKING'

        elapsed = time.time() - start_time

        if elapsed >= max_search_time:
            break

        msg = Twist()
        msg.angular.z = SEARCH_ANGULAR_SPEED
        monitor_node.cmd_vel_pub.publish(msg)
        monitor_node.publish_sector_home_req(False)

        time.sleep(STOP_PERIOD)

    navigator.info(
        'Final sector search finished. Human not detected. '
        '/robot3/sector_home_req=True and waiting.'
    )

    monitor_node.force_stop_robot(1.0, gohome_value=False)

    while rclpy.ok():
        if monitor_node.current_mode != TARGET_MODE:
            monitor_node.publish_sector_home_req(False)
            monitor_node.force_stop_robot(0.5, gohome_value=False)
            return 'WAIT_MODE'

        if monitor_node.is_tracking_requested():
            monitor_node.publish_sector_home_req(False)
            return 'TRACKING'

        monitor_node.publish_zero_cmd()
        monitor_node.publish_sector_home_req(True)

        time.sleep(0.05)

    return 'WAIT_MODE'


class TrackingModeNode(Node):

    def __init__(self):
        super().__init__('tracking_mode_node')

        self.K = None
        self.lock = threading.Lock()

        self.rgb_topic = f'{ROBOT_NS}/oakd/rgb/image_raw/compressed'
        self.depth_topic = f'{ROBOT_NS}/oakd/stereo/image_raw/compressedDepth'
        self.info_topic = f'{ROBOT_NS}/oakd/rgb/camera_info'
        self.bottom_topic = f'{ROBOT_NS}/detected_bottom'
        self.cmd_vel_topic = f'{ROBOT_NS}/cmd_vel'

        self.rgb_image = None
        self.depth_image = None
        self.camera_frame = None
        self.detected_bottom = None
        self.is_detected = False
        self.display_image = None

        self.goal_sent = False
        self.stopped = False
        self.follow_mode = False
        self.current_distance = None

        self.last_detected_bottom = None
        self.last_seen_time = None

        self.searching = False
        self.search_start_time = None
        self.search_direction = 0.0

        self.final_searching = False
        self.final_search_start_time = None
        self.final_gohome_waiting = False

        self.mode_stop_requested = False
        self.mode_stop_handled = False
        self.tracking_finished = False
        self.finish_reason = None

        self.logged_intrinsics = False
        self.logged_rgb_shape = False
        self.logged_depth_shape = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.navigator = TurtleBot4Navigator()
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.gohome_sector_pub = self.create_publisher(Bool, GOHOME_SECTOR_TOPIC, 10)
        self.sector_home_req_pub = self.create_publisher(Bool, SECTOR_HOME_REQ_TOPIC, 10)

        self.create_subscription(String, MODE_TOPIC, self.mode_callback, 10)
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 10)
        self.create_subscription(CompressedImage, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(Point, self.bottom_topic, self.bottom_callback, 10)

        self.gohome_timer = self.create_timer(STOP_PERIOD, self.publish_gohome_status)
        self.mode_stop_timer = self.create_timer(0.05, self.handle_mode_stop)

        self.gui_thread_stop = threading.Event()
        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

        self.get_logger().info('Tracking mode started.')
        self.get_logger().info(f'Subscribed bottom topic: {self.bottom_topic}')
        self.get_logger().info('TF Tree 안정화 시작. 5초 후 추적을 시작합니다.')

        self.start_timer = self.create_timer(5.0, self.start_transform)

    def mode_callback(self, msg):
        mode = msg.data.strip()

        if mode != TARGET_MODE and not self.mode_stop_requested:
            self.mode_stop_requested = True
            self.get_logger().warn(
                f'Mode changed to "{mode}". Stop tracking mode.'
            )

    def handle_mode_stop(self):
        if not self.mode_stop_requested:
            return

        if self.mode_stop_handled:
            return

        self.mode_stop_handled = True

        try:
            self.navigator.cancelTask()
        except Exception as e:
            self.get_logger().warn(f'Mode stop cancelTask failed: {e}')

        count = int(1.0 * STOP_PUBLISH_HZ)

        for _ in range(count):
            self.cmd_vel_pub.publish(Twist())
            self.publish_gohome_sector(False)
            self.publish_sector_home_req(False)
            time.sleep(STOP_PERIOD)

        self.gui_thread_stop.set()
        self.tracking_finished = True
        self.finish_reason = 'WAIT_MODE'

        self.get_logger().warn('Tracking mode stopped by mode change. Return to waiting mode.')

    def start_transform(self):
        self.get_logger().info('TF Tree 안정화 완료. 추적 시작.')

        self.timer = self.create_timer(PROCESS_PERIOD, self.process_bottom)
        self.display_timer = self.create_timer(DISPLAY_PERIOD, self.update_display)

        self.start_timer.cancel()

    def publish_gohome_sector(self, value):
        msg = Bool()
        msg.data = value
        self.gohome_sector_pub.publish(msg)

    def publish_sector_home_req(self, value):
        msg = Bool()
        msg.data = value
        self.sector_home_req_pub.publish(msg)

    def publish_gohome_status(self):
        if self.mode_stop_requested:
            return

        self.publish_gohome_sector(self.final_gohome_waiting)

    def camera_info_callback(self, msg):
        with self.lock:
            self.K = np.array(msg.k).reshape(3, 3)

        if not self.logged_intrinsics:
            self.get_logger().info(
                f'Camera intrinsics received: '
                f'fx={self.K[0, 0]:.2f}, fy={self.K[1, 1]:.2f}, '
                f'cx={self.K[0, 2]:.2f}, cy={self.K[1, 2]:.2f}'
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

    def bottom_callback(self, msg):
        if self.mode_stop_requested:
            return

        with self.lock:
            if msg.x == -1.0 and msg.y == -1.0:
                self.is_detected = False
                self.detected_bottom = None
                self.current_distance = None
                return

            bottom = (int(msg.x), int(msg.y))

            self.is_detected = True
            self.detected_bottom = bottom
            self.last_detected_bottom = bottom
            self.last_seen_time = self.get_clock().now()

            self.searching = False
            self.search_start_time = None
            self.search_direction = 0.0

    def get_current_robot_orientation(self):
        try:
            tf_robot = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=0.5)
            )

            return tf_robot.transform.rotation

        except Exception as e:
            self.get_logger().warn(
                f'Current robot orientation lookup failed. Use default orientation: {e}'
            )

            return Quaternion(
                x=0.0,
                y=0.0,
                z=0.0,
                w=1.0
            )

    def make_goal_from_bottom(self, x, y, z, frame_id, K):
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
        goal_pose.pose.orientation = self.get_current_robot_orientation()

        return goal_pose, pt_map

    def stop_robot(self):
        try:
            self.navigator.cancelTask()
        except Exception as e:
            self.get_logger().warn(f'cancelTask failed: {e}')

        self.cmd_vel_pub.publish(Twist())

        self.stopped = True
        self.follow_mode = True

    def shutdown_after_search_fail(self):
        try:
            self.navigator.cancelTask()
        except Exception as e:
            self.get_logger().warn(f'Search fail cancelTask failed: {e}')

        self.cmd_vel_pub.publish(Twist())

        self.get_logger().warn(
            f'Target not found after {SEARCH_TURNS:.1f} turns. Tracking finished.'
        )

        self.gui_thread_stop.set()
        self.tracking_finished = True
        self.finish_reason = 'FINISHED'

    def search_by_last_bottom(self, last_bottom, image_width):
        now = self.get_clock().now()

        if not self.searching:
            last_x, _ = last_bottom
            image_center_x = image_width / 2.0

            if last_x < image_center_x:
                self.search_direction = 1.0
            else:
                self.search_direction = -1.0

            self.search_start_time = now
            self.searching = True

        elapsed_search = (now - self.search_start_time).nanoseconds / 1e9
        rotated_angle = elapsed_search * abs(SEARCH_ANGULAR_SPEED)

        if rotated_angle >= SEARCH_MAX_ROTATION:
            self.shutdown_after_search_fail()
            return

        msg = Twist()
        msg.angular.z = self.search_direction * SEARCH_ANGULAR_SPEED
        self.cmd_vel_pub.publish(msg)

    def start_final_search(self):
        self.final_searching = True
        self.final_search_start_time = self.get_clock().now()

        try:
            self.navigator.cancelTask()
        except Exception:
            pass

        self.cmd_vel_pub.publish(Twist())

    def final_search_step(self):
        if self.mode_stop_requested:
            return

        if self.final_gohome_waiting:
            self.cmd_vel_pub.publish(Twist())
            self.publish_gohome_sector(True)
            return

        if not self.final_searching:
            self.start_final_search()

        now = self.get_clock().now()
        elapsed_search = (now - self.final_search_start_time).nanoseconds / 1e9
        rotated_angle = elapsed_search * abs(SEARCH_ANGULAR_SPEED)

        if rotated_angle >= FINAL_SEARCH_MAX_ROTATION:
            self.cmd_vel_pub.publish(Twist())
            self.final_searching = False
            self.final_search_start_time = None
            self.final_gohome_waiting = True
            self.publish_gohome_sector(True)
            return

        msg = Twist()
        msg.angular.z = SEARCH_ANGULAR_SPEED
        self.cmd_vel_pub.publish(msg)
        self.publish_gohome_sector(False)

    def process_bottom(self):
        if self.mode_stop_requested or self.tracking_finished:
            return

        with self.lock:
            depth = self.depth_image.copy() if self.depth_image is not None else None
            rgb = self.rgb_image.copy() if self.rgb_image is not None else None
            bottom = self.detected_bottom
            last_bottom = self.last_detected_bottom
            last_seen_time = self.last_seen_time
            frame_id = self.camera_frame
            K = self.K.copy() if self.K is not None else None
            is_detected = self.is_detected
            follow_mode = self.follow_mode

        if not is_detected or bottom is None:
            if self.final_gohome_waiting:
                self.cmd_vel_pub.publish(Twist())
                self.publish_gohome_sector(True)
                return

            if self.final_searching:
                self.final_search_step()
                return

            if self.goal_sent and not follow_mode:
                if self.navigator.isTaskComplete():
                    self.final_search_step()
                    return

            if follow_mode:
                if last_bottom is not None and last_seen_time is not None:
                    elapsed_lost = (
                        self.get_clock().now() - last_seen_time
                    ).nanoseconds / 1e9

                    if elapsed_lost >= LOST_DETECT_SECONDS:
                        if rgb is not None:
                            image_width = rgb.shape[1]
                        elif depth is not None:
                            image_width = depth.shape[1]
                        else:
                            return

                        self.search_by_last_bottom(last_bottom, image_width)
                        return

                self.cmd_vel_pub.publish(Twist())

            elif self.stopped:
                self.cmd_vel_pub.publish(Twist())

            return

        if depth is None or frame_id is None or K is None:
            return

        if self.final_gohome_waiting:
            self.cmd_vel_pub.publish(Twist())
            self.publish_gohome_sector(True)
            return

        if self.final_searching:
            self.cmd_vel_pub.publish(Twist())
            self.final_searching = False
            self.final_search_start_time = None
            self.follow_mode = True
            self.stopped = False
            self.publish_gohome_sector(False)

        if self.searching:
            self.cmd_vel_pub.publish(Twist())
            self.searching = False
            self.search_start_time = None
            self.search_direction = 0.0

        x, y = bottom
        h, w = depth.shape[:2]

        if x < 0 or x >= w or y < 0 or y >= h:
            return

        z = float(depth[y, x]) / 1000.0

        with self.lock:
            self.current_distance = z
            self.last_detected_bottom = bottom
            self.last_seen_time = self.get_clock().now()

        if not (0.2 < z < 5.0):
            return

        if self.follow_mode:
            if z <= STOP_DISTANCE:
                try:
                    self.navigator.cancelTask()
                except Exception:
                    pass

                self.cmd_vel_pub.publish(Twist())
                self.stopped = True
                return

            try:
                goal_pose, _ = self.make_goal_from_bottom(x, y, z, frame_id, K)
                self.navigator.goToPose(goal_pose)
                self.stopped = False
            except Exception as e:
                self.get_logger().warn(f'Follow mode TF or goal error: {e}')

            return

        if z <= STOP_DISTANCE:
            self.stop_robot()
            return

        if self.goal_sent:
            return

        try:
            goal_pose, _ = self.make_goal_from_bottom(x, y, z, frame_id, K)
            self.navigator.goToPose(goal_pose)
            self.goal_sent = True
        except Exception as e:
            self.get_logger().warn(f'TF or goal error: {e}')

    def update_display(self):
        with self.lock:
            rgb = self.rgb_image.copy() if self.rgb_image is not None else None
            depth = self.depth_image.copy() if self.depth_image is not None else None
            bottom = self.detected_bottom
            is_detected = self.is_detected
            distance = self.current_distance
            goal_sent = self.goal_sent
            stopped = self.stopped
            follow_mode = self.follow_mode
            searching = self.searching

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

        depth_display = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)

        status = (
            f'goal_sent: {goal_sent}, stopped: {stopped}, '
            f'follow: {follow_mode}, search: {searching}'
        )

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

        if is_detected and bottom is not None:
            x, y = bottom
            cv2.circle(rgb_display, (x, y), 6, (0, 255, 0), -1)

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
        window_name = 'Tracking Mode | RGB left / Depth right'

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
                    self.gui_thread_stop.set()
                    self.tracking_finished = True
                    self.finish_reason = 'FINISHED'
                    break
            else:
                cv2.waitKey(10)


def run_tracking_mode():
    node = TrackingModeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.tracking_finished:
            executor.spin_once(timeout_sec=0.05)
    except KeyboardInterrupt:
        node.finish_reason = 'FINISHED'

    node.gui_thread_stop.set()

    if node.gui_thread.is_alive():
        node.gui_thread.join(timeout=1.0)

    reason = node.finish_reason

    executor.shutdown()
    node.destroy_node()
    cv2.destroyAllWindows()

    return reason


def build_point_data():
    point_data = {
        'point1': {
            'xy': [-2.8649349212646484, -0.10677920281887054],
            'direction': TurtleBot4Directions.SOUTH
        },
        'point2': {
            'xy': [-2.5, 1.05],
            'direction': TurtleBot4Directions.WEST
        },
        'point3': {
            'xy': [-4.667, 1.275],
            'direction': TurtleBot4Directions.WEST
        },
        'point4': {
            'xy': [-2.557, 4.094],
            'direction': TurtleBot4Directions.EAST
        },
        'point5': {
            'xy': [-4.740, 4.403],
            'direction': TurtleBot4Directions.NORTH
        },
        'point24_mid': {
            'xy': [-2.434, 3.028],
            'direction': TurtleBot4Directions.SOUTH
        },
        'point35_mid': {
            'xy': [-4.700, 2.830],
            'direction': TurtleBot4Directions.NORTH
        },
    }

    target_xy = [
        (point_data['point24_mid']['xy'][0] + point_data['point35_mid']['xy'][0]) / 2.0,
        (point_data['point24_mid']['xy'][1] + point_data['point35_mid']['xy'][1]) / 2.0,
    ]

    point_data['target'] = {
        'xy': target_xy,
        'direction': TurtleBot4Directions.SOUTH
    }

    return point_data


def run_to_sector_mission(navigator, monitor_node, nav_lock):
    if navigator.getDockedStatus():

        navigator.info('Robot is docked.')

        while rclpy.ok() and not monitor_node.has_battery_state():
            if monitor_node.is_mode_stop_requested():
                return 'WAIT_MODE'

            navigator.info('Waiting for battery threshold and battery state...')
            time.sleep(0.5)

        while rclpy.ok() and monitor_node.is_paused():
            if monitor_node.is_mode_stop_requested():
                return 'WAIT_MODE'

            navigator.info('Battery is below threshold. Waiting while staying docked...')
            time.sleep(0.5)

        if monitor_node.is_mode_stop_requested():
            return 'WAIT_MODE'

        navigator.info('Battery threshold cleared. Undocking...')
        navigator.undock()
        navigator.info('Undock finished. Continue mission.')

    else:
        navigator.info('Robot is already undocked. Continue mission.')

    if monitor_node.is_mode_stop_requested():
        return 'WAIT_MODE'

    current_x, current_y = wait_for_current_xy(navigator, monitor_node)

    if current_x is None or current_y is None:
        return 'WAIT_MODE'

    point_data = build_point_data()

    target_xy = point_data['target']['xy']
    navigator.info(f'Target point: x={target_xy[0]:.3f}, y={target_xy[1]:.3f}')

    candidate_names = [
        'point1',
        'point2',
        'point3',
        'point4',
        'point5',
        'point24_mid',
        'point35_mid'
    ]

    nearest_name = None
    nearest_distance = None

    for name in candidate_names:
        px, py = point_data[name]['xy']
        dist = distance_xy(current_x, current_y, px, py)

        navigator.info(f'{name}: distance = {dist:.3f} m')

        if nearest_distance is None or dist < nearest_distance:
            nearest_name = name
            nearest_distance = dist

    navigator.info(f'Nearest point: {nearest_name}, distance={nearest_distance:.3f} m')

    navigator.info('Waiting for NavigateToPose action server...')
    while not navigator.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
        if monitor_node.is_mode_stop_requested():
            return 'WAIT_MODE'

        navigator.info('NavigateToPose action server not available, waiting...')

    route_names = [nearest_name]

    if nearest_name in ['point2', 'point4']:
        route_names += ['point24_mid', 'target']

    elif nearest_name in ['point3', 'point5']:
        route_names += ['point35_mid', 'target']

    elif nearest_name == 'point1':
        route_names += ['point2', 'point24_mid', 'target']

    elif nearest_name in ['point24_mid', 'point35_mid']:
        route_names += ['target']

    navigator.info(f'Final route: {" -> ".join(route_names)}')

    mission_result = run_pose_sequence(
        navigator,
        monitor_node,
        nav_lock,
        route_names,
        point_data
    )

    if mission_result == 'SUCCEEDED':
        return final_sector_search(
            navigator,
            monitor_node,
            nav_lock
        )

    return mission_result


def cleanup_all(navigator, monitor_node, monitor_executor, monitor_thread, cancel_thread):
    monitor_node.stop_requested = True

    try:
        navigator.cancelTask()
    except Exception:
        pass

    monitor_node.force_stop_robot(1.0, gohome_value=False)
    monitor_node.publish_gohome_sector(False)
    monitor_node.publish_sector_home_req(False)

    monitor_executor.shutdown()
    monitor_thread.join(timeout=1.0)
    cancel_thread.join(timeout=1.0)

    monitor_node.destroy_node()
    navigator.destroy_node()


def main():
    rclpy.init(args=[
        '--ros-args',
        '-r', '__ns:=/robot3',
        '-r', '/tf:=/robot3/tf',
        '-r', '/tf_static:=/robot3/tf_static',
    ])

    navigator = TurtleBot4Navigator()
    monitor_node = MissionMonitorNode()
    nav_lock = threading.Lock()

    monitor_executor = SingleThreadedExecutor()
    monitor_executor.add_node(monitor_node)

    monitor_thread = threading.Thread(
        target=spin_monitor_node,
        args=(monitor_executor, monitor_node),
        daemon=True
    )
    monitor_thread.start()

    cancel_thread = threading.Thread(
        target=cancel_monitor,
        args=(navigator, monitor_node, nav_lock),
        daemon=True
    )
    cancel_thread.start()

    navigator.info('to_sector mission node started. Waiting mode enabled.')

    try:
        while rclpy.ok():

            monitor_node.reset_for_waiting()

            if not wait_for_to_sector_mode(monitor_node):
                break

            monitor_node.mark_mission_started()

            mission_result = run_to_sector_mission(
                navigator,
                monitor_node,
                nav_lock
            )

            if mission_result == 'WAIT_MODE':
                navigator.info('Mode changed or waiting interrupted. Return to waiting mode.')
                cancel_current_goal_now(
                    navigator,
                    monitor_node,
                    nav_lock,
                    stop_sec=1.0
                )
                monitor_node.reset_for_waiting()
                continue

            if mission_result == 'TRACKING':
                navigator.info('Switching to tracking mode.')

                tracking_result = run_tracking_mode()

                if tracking_result == 'WAIT_MODE':
                    navigator.info('Tracking stopped by mode change. Return to waiting mode.')
                    monitor_node.reset_for_waiting()
                    continue

                navigator.info('Tracking finished. Return to waiting mode.')
                monitor_node.reset_for_waiting()
                continue

            if mission_result == 'SUCCEEDED':
                navigator.info('Final target route completed. Return to waiting mode.')
                monitor_node.force_stop_robot(1.0, gohome_value=False)
                monitor_node.reset_for_waiting()
                continue

            navigator.info('to_sector mission failed. Return to waiting mode.')
            monitor_node.force_stop_robot(1.0, gohome_value=False)
            monitor_node.reset_for_waiting()

    except KeyboardInterrupt:
        pass

    cleanup_all(
        navigator,
        monitor_node,
        monitor_executor,
        monitor_thread,
        cancel_thread
    )

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()