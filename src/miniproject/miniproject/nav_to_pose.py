#!/usr/bin/env python3

import threading

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
        '/robot3/cmd_vel',
        10
    )

    def stop_robot():
        msg = Twist()
        cmd_vel_pub.publish(msg)

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
        '/robot3/is_detected',
        detected_callback,
        10
    )

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(navigator,),
        daemon=True
    )
    spin_thread.start()

    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    initial_pose = navigator.getPoseStamped(
        [-0.1718821734565192, 0.1102970552966071],
        TurtleBot4Directions.NORTH
    )
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    goal_pose = navigator.getPoseStamped(
        [-3.169, -0.118],
        TurtleBot4Directions.EAST
    )

    navigator.undock()

    if not detected:
        navigator.startToPose(goal_pose)

    stop_robot()

    navigator.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()