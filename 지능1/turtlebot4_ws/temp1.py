#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    QoSReliabilityPolicy
)

import tf2_ros
import numpy as np
import cv2


MAP_TOPIC = '/robot3/map'
GLOBAL_COSTMAP_TOPIC = '/robot3/global_costmap/costmap'
LOCAL_COSTMAP_TOPIC = '/robot3/local_costmap/costmap'
PLAN_TOPIC = '/robot3/plan'
SCAN_TOPIC = '/robot3/scan'

MAP_FRAME = 'map'

ROBOT_FRAME_CANDIDATES = [
    'robot3/base_link',
    'base_link',
    'robot3/base_footprint',
    'base_footprint',
]

LASER_FRAME_CANDIDATES = [
    'robot3/rplidar_link',
    'rplidar_link',
    'robot3/base_scan',
    'base_scan',
]

SCALE = 5
DRAW_RATE = 0.1
WINDOW_NAME = 'RViz-like realtime view'


class RvizLikeViewer(Node):

    def __init__(self):
        super().__init__('rviz_like_viewer')

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.map_msg = None
        self.global_costmap_msg = None
        self.local_costmap_msg = None
        self.plan_msg = None
        self.scan_msg = None

        self.create_subscription(
            OccupancyGrid,
            MAP_TOPIC,
            self.map_callback,
            transient_qos
        )

        self.create_subscription(
            OccupancyGrid,
            GLOBAL_COSTMAP_TOPIC,
            self.global_costmap_callback,
            10
        )

        self.create_subscription(
            OccupancyGrid,
            LOCAL_COSTMAP_TOPIC,
            self.local_costmap_callback,
            10
        )

        self.create_subscription(
            Path,
            PLAN_TOPIC,
            self.plan_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self.scan_callback,
            sensor_qos
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.timer = self.create_timer(DRAW_RATE, self.draw)

        self.get_logger().info('RViz-like realtime viewer started')

    def map_callback(self, msg):
        self.map_msg = msg

    def global_costmap_callback(self, msg):
        self.global_costmap_msg = msg

    def local_costmap_callback(self, msg):
        self.local_costmap_msg = msg

    def plan_callback(self, msg):
        self.plan_msg = msg

    def scan_callback(self, msg):
        self.scan_msg = msg

    def world_to_pixel(self, x, y, info):
        ox = info.origin.position.x
        oy = info.origin.position.y
        res = info.resolution

        px = int((x - ox) / res)
        py = int((y - oy) / res)

        py = info.height - py

        return px, py

    def get_yaw_from_quaternion(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def lookup_first_tf(self, target_frame, frame_candidates):
        for frame in frame_candidates:
            try:
                tf = self.tf_buffer.lookup_transform(
                    target_frame,
                    frame,
                    rclpy.time.Time()
                )
                return tf
            except Exception:
                pass

        return None

    def make_base_map_image(self):
        msg = self.map_msg

        width = msg.info.width
        height = msg.info.height

        data = np.array(msg.data, dtype=np.int16)
        grid = data.reshape((height, width))

        image = np.zeros((height, width, 3), dtype=np.uint8)

        image[grid == -1] = (160, 160, 160)     # unknown
        image[grid == 0] = (245, 245, 245)      # free
        image[grid >= 100] = (20, 20, 20)       # wall

        image = cv2.flip(image, 0)

        return image

    def overlay_costmap(self, base, costmap_msg, color, alpha):
        if costmap_msg is None or self.map_msg is None:
            return base

        data = np.array(costmap_msg.data, dtype=np.int16)
        grid = data.reshape(
            (costmap_msg.info.height, costmap_msg.info.width)
        )

        overlay = base.copy()

        indices = np.argwhere(grid > 0)

        for cy, cx in indices:
            wx = costmap_msg.info.origin.position.x + cx * costmap_msg.info.resolution
            wy = costmap_msg.info.origin.position.y + cy * costmap_msg.info.resolution

            px, py = self.world_to_pixel(wx, wy, self.map_msg.info)

            if 0 <= px < base.shape[1] and 0 <= py < base.shape[0]:
                overlay[py, px] = color

        return cv2.addWeighted(overlay, alpha, base, 1.0 - alpha, 0)

    def draw_plan(self, image):
        if self.plan_msg is None or self.map_msg is None:
            return image

        points = []

        for pose in self.plan_msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y

            px, py = self.world_to_pixel(x, y, self.map_msg.info)

            if 0 <= px < image.shape[1] and 0 <= py < image.shape[0]:
                points.append((px, py))

        for i in range(len(points) - 1):
            cv2.line(
                image,
                points[i],
                points[i + 1],
                (255, 80, 0),
                2
            )

        return image

    def draw_robot(self, image):
        if self.map_msg is None:
            return image

        tf = self.lookup_first_tf(
            MAP_FRAME,
            ROBOT_FRAME_CANDIDATES
        )

        if tf is None:
            return image

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = self.get_yaw_from_quaternion(q)

        px, py = self.world_to_pixel(x, y, self.map_msg.info)

        if not (0 <= px < image.shape[1] and 0 <= py < image.shape[0]):
            return image

        size = 14

        front = (
            int(px + size * math.cos(yaw)),
            int(py - size * math.sin(yaw))
        )

        left = (
            int(px + size * math.cos(yaw + 2.4)),
            int(py - size * math.sin(yaw + 2.4))
        )

        right = (
            int(px + size * math.cos(yaw - 2.4)),
            int(py - size * math.sin(yaw - 2.4))
        )

        triangle = np.array(
            [front, left, right],
            dtype=np.int32
        )

        cv2.fillPoly(image, [triangle], (0, 0, 255))
        cv2.polylines(image, [triangle], True, (0, 0, 0), 1)

        cv2.circle(image, (px, py), 5, (255, 255, 255), -1)
        cv2.circle(image, (px, py), 8, (0, 0, 255), 2)

        return image

    def draw_scan(self, image):
        if self.scan_msg is None or self.map_msg is None:
            return image

        tf = self.lookup_first_tf(
            MAP_FRAME,
            LASER_FRAME_CANDIDATES
        )

        if tf is None:
            return image

        lx = tf.transform.translation.x
        ly = tf.transform.translation.y
        q = tf.transform.rotation
        laser_yaw = self.get_yaw_from_quaternion(q)

        angle = self.scan_msg.angle_min

        step = 3

        for r in self.scan_msg.ranges[::step]:
            if math.isfinite(r):
                if self.scan_msg.range_min < r < self.scan_msg.range_max:
                    wx = lx + r * math.cos(laser_yaw + angle)
                    wy = ly + r * math.sin(laser_yaw + angle)

                    px, py = self.world_to_pixel(wx, wy, self.map_msg.info)

                    if 0 <= px < image.shape[1] and 0 <= py < image.shape[0]:
                        image[py, px] = (255, 0, 255)

            angle += self.scan_msg.angle_increment * step

        return image

    def draw(self):
        if self.map_msg is None:
            return

        image = self.make_base_map_image()

        image = self.overlay_costmap(
            image,
            self.global_costmap_msg,
            color=(0, 180, 255),
            alpha=0.35
        )

        image = self.overlay_costmap(
            image,
            self.local_costmap_msg,
            color=(0, 255, 255),
            alpha=0.45
        )

        image = self.draw_plan(image)
        image = self.draw_scan(image)
        image = self.draw_robot(image)

        image = cv2.resize(
            image,
            None,
            fx=SCALE,
            fy=SCALE,
            interpolation=cv2.INTER_NEAREST
        )

        cv2.imshow(WINDOW_NAME, image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = RvizLikeViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()