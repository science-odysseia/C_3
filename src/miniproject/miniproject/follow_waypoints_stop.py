#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator
)


STOP_PUBLISH_HZ = 20.0
FORCE_STOP_SECONDS = 3.0


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

    def publish_zero_cmd(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def force_stop_robot(self, duration_sec=FORCE_STOP_SECONDS):
        period = 1.0 / STOP_PUBLISH_HZ
        repeat_count = int(duration_sec * STOP_PUBLISH_HZ)

        msg = Twist()

        for _ in range(repeat_count):
            self.cmd_vel_pub.publish(msg)
            time.sleep(period)

    def detected_callback(self, msg):
        if msg.data and not self.cancel_requested:
            self.cancel_requested = True
            self.detected = True

            self.get_logger().info(
                'Vehicle detected. Cancelling navigation.'
            )

            try:
                self.navigator.cancelTask()
            except Exception as e:
                self.get_logger().info(f'cancelTask failed: {e}')

            self.publish_zero_cmd()


def wait_for_amcl_service():
    wait_node = Node('wait_for_amcl_service_node')

    client = wait_node.create_client(
        GetState,
        '/robot3/amcl/get_state'
    )

    wait_node.get_logger().info(
        'Waiting for /robot3/amcl/get_state service...'
    )

    while rclpy.ok():
        if client.wait_for_service(timeout_sec=1.0):
            wait_node.get_logger().info(
                '/robot3/amcl/get_state service is available.'
            )
            break

        wait_node.get_logger().info(
            '/robot3/amcl/get_state service not available, waiting...'
        )

    wait_node.destroy_node()


def main():
    rclpy.init()

    wait_for_amcl_service()

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

    navigator.info('Setting initial pose...')
    navigator.setInitialPose(initial_pose)

    navigator.info('Initial pose published. Continuing...')
    time.sleep(1.0)

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

    navigator.info('Undocking...')
    navigator.undock()

    navigator.info('Waiting after undock...')
    time.sleep(0.5)

    navigator.info('Starting waypoint navigation...')
    navigator.startFollowWaypoints(goal_pose)

    navigator.info('Following waypoints. Waiting for vehicle detection...')

    while rclpy.ok() and not stop_node.detected:
        time.sleep(0.05)

    navigator.info('Vehicle detected. Forcing robot stop...')

    try:
        navigator.cancelTask()
    except Exception as e:
        navigator.info(f'cancelTask failed: {e}')

    stop_node.force_stop_robot(FORCE_STOP_SECONDS)

    navigator.info('Navigation stop node finished. Shutting down this node.')

    stop_executor.shutdown()
    stop_thread.join(timeout=1.0)

    stop_node.destroy_node()
    navigator.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()