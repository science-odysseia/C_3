#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Bool, String

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator
)


MODE_TOPIC = '/robot3/mode'
PATROL_MODE = 'patrol'


class PatrolControlNode(Node):

    def __init__(self):
        super().__init__('patrol_control_node')

        self.stop_requested = False
        self.current_mode = 'home'
        self.mode_cancel_requested = False

        self.detect_course_pub = self.create_publisher(
            Bool,
            '/robot3/detect_course',
            10
        )

        self.create_subscription(
            String,
            MODE_TOPIC,
            self.mode_callback,
            10
        )

    def mode_callback(self, msg):
        prev_mode = self.current_mode
        self.current_mode = msg.data

        if prev_mode != self.current_mode:
            self.get_logger().info(
                f'Mode changed: {prev_mode} -> {self.current_mode}'
            )

        if self.current_mode != PATROL_MODE:
            self.mode_cancel_requested = True
            self.publish_detect_course(False)

    def is_patrol_mode(self):
        return self.current_mode == PATROL_MODE

    def clear_mode_cancel_request(self):
        self.mode_cancel_requested = False

    def publish_detect_course(self, value):
        msg = Bool()
        msg.data = value
        self.detect_course_pub.publish(msg)


def spin_control_node(executor, control_node):
    while rclpy.ok() and not control_node.stop_requested:
        executor.spin_once(timeout_sec=0.01)


def cancel_task_only(navigator, control_node):
    try:
        navigator.cancelTask()
    except Exception as e:
        control_node.get_logger().info(
            f'cancelTask failed: {e}'
        )

    control_node.publish_detect_course(False)


def wait_until_patrol_mode(control_node):
    while rclpy.ok() and not control_node.stop_requested:
        if control_node.is_patrol_mode():
            control_node.clear_mode_cancel_request()
            return True

        control_node.publish_detect_course(False)
        time.sleep(0.1)

    return False


def go_to_pose_with_mode_control(navigator, control_node, pose, detect):
    control_node.publish_detect_course(detect)
    navigator.startToPose(pose)

    goal_active = True
    last_cancel_time = 0.0

    while rclpy.ok():

        if control_node.stop_requested:
            cancel_task_only(navigator, control_node)
            return False

        if not control_node.is_patrol_mode():
            now = time.time()

            if goal_active or now - last_cancel_time > 0.3:
                cancel_task_only(navigator, control_node)
                last_cancel_time = now
                goal_active = False

            return False

        if control_node.mode_cancel_requested:
            cancel_task_only(navigator, control_node)
            control_node.clear_mode_cancel_request()
            return False

        if navigator.isTaskComplete():
            return True

        time.sleep(0.01)

    return False


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator(namespace='robot3')
    control_node = PatrolControlNode()

    control_executor = SingleThreadedExecutor()
    control_executor.add_node(control_node)

    ros_spin_thread = threading.Thread(
        target=spin_control_node,
        args=(control_executor, control_node),
        daemon=True
    )
    ros_spin_thread.start()

    navigator.info(
        'Assuming local_turtle, rviz_turtle, nav_turtle are already running.'
    )

    point1 = navigator.getPoseStamped(
        [-2.8649349212646484, -0.10677920281887054],
        TurtleBot4Directions.SOUTH
    )

    point2 = navigator.getPoseStamped(
        [-2.5, 1.05],
        TurtleBot4Directions.WEST
    )

    point3 = navigator.getPoseStamped(
        [-4.667, 1.275],
        TurtleBot4Directions.WEST
    )

    point4 = navigator.getPoseStamped(
        [-2.557, 4.094],
        TurtleBot4Directions.EAST
    )

    point5 = navigator.getPoseStamped(
        [-4.740, 4.403],
        TurtleBot4Directions.NORTH
    )

    point24_mid = navigator.getPoseStamped(
        [-2.434, 3.028],
        TurtleBot4Directions.SOUTH
    )

    point35_mid = navigator.getPoseStamped(
        [-4.700, 2.830],
        TurtleBot4Directions.NORTH
    )

    point23_detect_start = navigator.getPoseStamped(
        [-3.061108810814647, 1.3032647707750753],
        TurtleBot4Directions.SOUTH
    )

    point23_detect_end = navigator.getPoseStamped(
        [-4.158250714190365, 1.338449649757827],
        TurtleBot4Directions.SOUTH
    )

    point_mid_detect_1 = navigator.getPoseStamped(
        [-4.118566993064851, 2.8581530340494647],
        TurtleBot4Directions.EAST
    )

    point_mid_detect_2 = navigator.getPoseStamped(
        [-2.9251650482249247, 2.832489167593096],
        TurtleBot4Directions.EAST
    )

    point45_detect_start = navigator.getPoseStamped(
        [-4.245204401273927, 4.358098960907224],
        TurtleBot4Directions.NORTH
    )

    point45_detect_end = navigator.getPoseStamped(
        [-3.0470807314333106, 4.30784850903123],
        TurtleBot4Directions.NORTH
    )

    first_route = [
        (point1, False),
        (point2, False),
    ]

    loop_route = [
        (point2, False),

        (point23_detect_start, True),
        (point23_detect_end, False),

        (point3, False),

        (point35_mid, False),

        (point_mid_detect_1, True),
        (point_mid_detect_2, False),

        (point24_mid, False),

        (point_mid_detect_2, True),
        (point_mid_detect_1, False),

        (point35_mid, False),

        (point5, False),

        (point45_detect_start, True),
        (point45_detect_end, False),

        (point4, False),
    ]

    control_node.publish_detect_course(False)

    while rclpy.ok() and not control_node.stop_requested:

        navigator.info('Waiting for patrol mode...')

        if not wait_until_patrol_mode(control_node):
            break

        navigator.info('Starting initial route: 1 -> 2')

        for pose, detect in first_route:
            success = go_to_pose_with_mode_control(
                navigator,
                control_node,
                pose,
                detect
            )

            if not success:
                break

        while rclpy.ok() and not control_node.stop_requested:
            if not control_node.is_patrol_mode():
                break

            navigator.info('Starting infinite patrol loop with mode control')

            for pose, detect in loop_route:
                success = go_to_pose_with_mode_control(
                    navigator,
                    control_node,
                    pose,
                    detect
                )

                if not success:
                    break

    navigator.info('Stopping patrol node and shutting down.')

    cancel_task_only(navigator, control_node)

    control_node.stop_requested = True

    control_executor.shutdown()
    ros_spin_thread.join(timeout=1.0)

    control_node.destroy_node()
    navigator.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()