#!/usr/bin/env python3

import time

import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator
)


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    detected = False

    cmd_vel_pub = navigator.create_publisher(
        Twist,
        'cmd_vel',
        10
    )

    def stop_robot():
        msg = Twist()
        for _ in range(5):
            cmd_vel_pub.publish(msg)
            time.sleep(0.05)

    def detected_callback(msg):
        nonlocal detected

        if msg.data and not detected:
            detected = True
            navigator.info('Object detected. Cancelling navigation and stopping robot.')

            try:
                navigator.cancelTask()
            except Exception as e:
                navigator.info(f'cancelTask failed: {e}')

            stop_robot()

    navigator.create_subscription(
        Bool,
        'is_detected',
        detected_callback,
        10
    )

    navigator.info('Nav node started')

    initial_pose = navigator.getPoseStamped(
        [-0.1718821734565192, 0.1102970552966071],
        TurtleBot4Directions.NORTH
    )

    navigator.info('Setting initial pose')
    navigator.setInitialPose(initial_pose)

    navigator.info('Waiting for Nav2 active')
    navigator.waitUntilNav2Active()

    navigator.info('Undocking')
    navigator.undock()

    time.sleep(1.0)

    goal_pose = navigator.getPoseStamped(
        [-3.169, -0.118],
        TurtleBot4Directions.EAST
    )

    navigator.info('Starting navigation to goal')
    navigator.startToPose(goal_pose)

    while rclpy.ok() and not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)

        if detected:
            navigator.info('Detected flag true. Navigation loop ending.')
            break

        time.sleep(0.1)

    stop_robot()

    navigator.info('Navigation node finished')

    navigator.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()