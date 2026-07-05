#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)


PRINT_INTERVAL = 0.1  # 초, 0.1이면 10Hz 출력


class AmclPoseViewer(Node):

    def __init__(self):
        super().__init__('amcl_pose_viewer')

        self.latest_pose = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot3/amcl_pose',
            self.pose_callback,
            qos
        )

        self.create_timer(PRINT_INTERVAL, self.print_pose)

        self.get_logger().info('Listening to /robot3/amcl_pose ...')

    def pose_callback(self, msg):
        self.latest_pose = msg.pose.pose

    def print_pose(self):
        if self.latest_pose is None:
            return

        x = self.latest_pose.position.x
        y = self.latest_pose.position.y

        qx = self.latest_pose.orientation.x
        qy = self.latest_pose.orientation.y
        qz = self.latest_pose.orientation.z
        qw = self.latest_pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        yaw = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw)

        self.get_logger().info(
            f'현재 위치: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f} deg'
        )


def main(args=None):
    rclpy.init(args=args)

    node = AmclPoseViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()