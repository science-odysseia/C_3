#!/usr/bin/env python3

import time
import rclpy

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator
)

from nav2_simple_commander.robot_navigator import TaskResult


def main():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/robot3'])

    navigator = TurtleBot4Navigator()

    navigator.info('Back home node started.')

    navigator.info('Waiting for NavigateToPose action server...')
    while not navigator.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
        navigator.info('NavigateToPose action server not available, waiting...')

    goal_pose = navigator.getPoseStamped(
        [-1.1248518228530884, 0.03161429986357689],
        TurtleBot4Directions.NORTH
    )

    navigator.info('Going back home...')
    navigator.startToPose(goal_pose)

    while not navigator.isTaskComplete():
        time.sleep(0.1)

    nav_result = navigator.getResult()
    navigator.info(f'Back home navigation result: {nav_result}')

    if nav_result == TaskResult.SUCCEEDED:
        navigator.info('Arrived home. Back home node will shutdown.')
    else:
        navigator.info('Back home failed. Back home node will shutdown.')

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()