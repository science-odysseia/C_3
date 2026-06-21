#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator
)


class DetectionStopNode(Node):

    def __init__(self, navigator):
        super().__init__('detection_stop_node')

        self.navigator = navigator
        self.detected = False
        self.cancel_requested = False

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/robot3/cmd_vel',
            10
        )

        self.create_subscription(
            Bool,
            '/robot3/is_detected',
            self.detected_callback,
            10
        )

    def stop_robot(self):
        msg = Twist()

        for _ in range(20):
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)

    def detected_callback(self, msg):
        # 로그가 너무 많이 뜨는 게 싫으면 이 줄은 나중에 지워도 됨
        self.get_logger().info(f'/robot3/is_detected: {msg.data}')

        if msg.data and not self.cancel_requested:
            self.cancel_requested = True
            self.detected = True

            self.get_logger().info(
                'Vehicle detected. Cancelling navigation and stopping robot.'
            )

            try:
                self.navigator.cancelTask()
            except Exception as e:
                self.get_logger().info(f'cancelTask failed: {e}')

            self.stop_robot()


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator(namespace='robot3')

    stop_node = DetectionStopNode(navigator)

    stop_executor = SingleThreadedExecutor()
    stop_executor.add_node(stop_node)

    stop_thread = threading.Thread(
        target=stop_executor.spin,
        daemon=True
    )
    stop_thread.start()

    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    initial_pose = navigator.getPoseStamped(
        [0.0, 0.0],
        TurtleBot4Directions.NORTH
    )
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    goal_pose = []
    goal_pose.append(
        navigator.getPoseStamped(
            [-1.724, 1.565],
            TurtleBot4Directions.SOUTH
        )
    )
    goal_pose.append(
        navigator.getPoseStamped(
            [-4.476, 1.565],
            TurtleBot4Directions.EAST
        )
    )
    goal_pose.append(
        navigator.getPoseStamped(
            [-4.476, -0.118],
            TurtleBot4Directions.NORTH
        )
    )
    goal_pose.append(
        navigator.getPoseStamped(
            [-3.169, -0.118],
            TurtleBot4Directions.NORTH
        )
    )

    navigator.undock()

    navigator.startFollowWaypoints(goal_pose)

    navigator.info('Following waypoints. Waiting for vehicle detection...')

    while rclpy.ok() and not stop_node.detected:
        time.sleep(0.1)

    navigator.info('Navigation stop node finished. Shutting down this node.')

    stop_node.stop_robot()

    stop_executor.shutdown()
    stop_thread.join(timeout=1.0)

    stop_node.destroy_node()
    navigator.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()