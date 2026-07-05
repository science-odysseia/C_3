#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

from irobot_create_msgs.action import Dock
from irobot_create_msgs.msg import DockStatus

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator
)


MODE_TOPIC = '/robot3/mode'
AMCL_TOPIC = '/robot3/amcl_pose'
DOCK_STATUS_TOPIC = '/robot3/dock_status'
DOCK_ACTION = '/robot3/dock'

HOME_MODE = 'home'
HOME_STABLE_SECONDS = 1.2


class GoHomeControlNode(Node):

    def __init__(self):
        super().__init__('gohome_control_node')

        self.stop_requested = False

        self.current_mode = None
        self.home_start_time = None

        self.current_x = None
        self.current_y = None

        self.is_docked = None

        self.mode_cancel_requested = False

        self.create_subscription(
            String,
            MODE_TOPIC,
            self.mode_callback,
            10
        )

        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            AMCL_TOPIC,
            self.amcl_callback,
            amcl_qos
        )

        self.create_subscription(
            DockStatus,
            DOCK_STATUS_TOPIC,
            self.dock_status_callback,
            10
        )

        self.dock_client = ActionClient(
            self,
            Dock,
            DOCK_ACTION
        )

    def mode_callback(self, msg):
        prev_mode = self.current_mode
        self.current_mode = msg.data

        if prev_mode != self.current_mode:
            self.get_logger().info(
                f'Mode changed: {prev_mode} -> {self.current_mode}'
            )

        if self.current_mode == HOME_MODE:
            if prev_mode != HOME_MODE:
                self.home_start_time = time.time()
        else:
            self.home_start_time = None
            self.mode_cancel_requested = True

    def amcl_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def dock_status_callback(self, msg):
        self.is_docked = msg.is_docked

    def is_home_mode(self):
        return self.current_mode == HOME_MODE

    def is_home_stable(self):
        if self.current_mode != HOME_MODE:
            return False

        if self.home_start_time is None:
            return False

        return time.time() - self.home_start_time >= HOME_STABLE_SECONDS

    def is_undocked(self):
        return self.is_docked is False

    def reset_home_stable_timer(self):
        self.home_start_time = None

    def has_current_pose(self):
        return self.current_x is not None and self.current_y is not None

    def has_dock_status(self):
        return self.is_docked is not None

    def clear_cancel_request(self):
        self.mode_cancel_requested = False

    def get_current_xy(self):
        return self.current_x, self.current_y


def spin_control_node(executor, control_node):
    while rclpy.ok() and not control_node.stop_requested:
        executor.spin_once(timeout_sec=0.01)


def cancel_task_only(navigator, control_node):
    try:
        navigator.cancelTask()
    except Exception as e:
        control_node.get_logger().info(f'cancelTask failed: {e}')


def wait_until_amcl_ready(control_node):
    while rclpy.ok() and not control_node.stop_requested:
        if control_node.has_current_pose():
            return True

        control_node.get_logger().info('Waiting for AMCL pose...')
        time.sleep(0.5)

    return False


def wait_until_dock_status_ready(control_node):
    while rclpy.ok() and not control_node.stop_requested:
        if control_node.has_dock_status():
            return True

        control_node.get_logger().info('Waiting for dock status...')
        time.sleep(0.5)

    return False


def wait_until_home_stable_and_undocked(control_node):
    while rclpy.ok() and not control_node.stop_requested:

        if control_node.is_home_stable():

            if control_node.is_undocked():
                control_node.clear_cancel_request()
                return True

            control_node.get_logger().info(
                'Home mode received, but robot is docked. Waiting...'
            )

            control_node.reset_home_stable_timer()

        time.sleep(0.1)

    return False


def go_to_pose_with_home_control(navigator, control_node, pose, name):
    control_node.get_logger().info(f'Going to {name}')

    navigator.startToPose(pose)

    goal_active = True
    last_cancel_time = 0.0

    while rclpy.ok() and not control_node.stop_requested:

        if not control_node.is_home_mode():
            now = time.time()

            if goal_active or now - last_cancel_time > 0.3:
                control_node.get_logger().info(
                    'Home mode released. Cancel goal only.'
                )
                cancel_task_only(navigator, control_node)
                goal_active = False
                last_cancel_time = now

            return False

        if control_node.mode_cancel_requested:
            cancel_task_only(navigator, control_node)
            control_node.clear_cancel_request()
            return False

        if navigator.isTaskComplete():
            control_node.get_logger().info(f'Arrived at {name}')
            return True

        time.sleep(0.01)

    return False


def get_nearest_point_name(control_node, point_xy_dict):
    current_x, current_y = control_node.get_current_xy()

    nearest_name = None
    nearest_dist = None

    for name, xy in point_xy_dict.items():
        px, py = xy
        dist = math.hypot(current_x - px, current_y - py)

        if nearest_dist is None or dist < nearest_dist:
            nearest_dist = dist
            nearest_name = name

    control_node.get_logger().info(
        f'Current position: ({current_x:.3f}, {current_y:.3f})'
    )
    control_node.get_logger().info(
        f'Nearest point: {nearest_name}, distance: {nearest_dist:.3f}'
    )

    return nearest_name


def dock_without_mode_cancel(control_node):
    control_node.get_logger().info('Waiting for dock action server...')

    if not control_node.dock_client.wait_for_server(timeout_sec=5.0):
        control_node.get_logger().info('Dock action server not available.')
        return False

    goal_msg = Dock.Goal()

    send_future = control_node.dock_client.send_goal_async(goal_msg)

    while rclpy.ok() and not send_future.done():
        time.sleep(0.02)

    goal_handle = send_future.result()

    if not goal_handle.accepted:
        control_node.get_logger().info('Dock goal rejected.')
        return False

    control_node.get_logger().info('Dock goal accepted.')

    result_future = goal_handle.get_result_async()

    while rclpy.ok() and not result_future.done():
        time.sleep(0.02)

    control_node.get_logger().info('Dock finished.')
    return True


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator(namespace='robot3')
    control_node = GoHomeControlNode()

    control_executor = SingleThreadedExecutor()
    control_executor.add_node(control_node)

    ros_spin_thread = threading.Thread(
        target=spin_control_node,
        args=(control_executor, control_node),
        daemon=True
    )
    ros_spin_thread.start()

    navigator.info(
        'Gohome node started. It only works when mode is home and robot is undocked.'
    )

    point_xy = {
        '1': [-2.8649349212646484, -0.10677920281887054],
        '2': [-2.5, 1.05],
        '3': [-4.667, 1.275],
        '4': [-2.557, 4.094],
        '5': [-4.740, 4.403],
        '24_mid': [-2.434, 3.028],
        '35_mid': [-4.700, 2.830],
        '0': [-0.1, -0.1],
    }

    point_pose = {
        '1': navigator.getPoseStamped(
            point_xy['1'],
            TurtleBot4Directions.NORTH
        ),
        '2': navigator.getPoseStamped(
            point_xy['2'],
            TurtleBot4Directions.EAST
        ),
        '3': navigator.getPoseStamped(
            point_xy['3'],
            TurtleBot4Directions.NORTH
        ),
        '4': navigator.getPoseStamped(
            point_xy['4'],
            TurtleBot4Directions.EAST
        ),
        '5': navigator.getPoseStamped(
            point_xy['5'],
            TurtleBot4Directions.EAST
        ),
        '24_mid': navigator.getPoseStamped(
            point_xy['24_mid'],
            TurtleBot4Directions.EAST
        ),
        '35_mid': navigator.getPoseStamped(
            point_xy['35_mid'],
            TurtleBot4Directions.EAST
        ),
        '0': navigator.getPoseStamped(
            point_xy['0'],
            TurtleBot4Directions.NORTH
        ),
    }

    route_table = {
        '1': ['1', '0'],
        '2': ['2', '1', '0'],
        '3': ['3', '2', '1', '0'],
        '4': ['4', '2', '1', '0'],
        '5': ['5', '3', '2', '1', '0'],
        '24_mid': ['24_mid', '2', '1', '0'],
        '35_mid': ['35_mid', '3', '2', '1', '0'],
    }

    if not wait_until_amcl_ready(control_node):
        control_node.stop_requested = True

    if not wait_until_dock_status_ready(control_node):
        control_node.stop_requested = True

    while rclpy.ok() and not control_node.stop_requested:

        navigator.info(
            'Waiting until mode is "home" for 1.2s and robot is undocked...'
        )

        if not wait_until_home_stable_and_undocked(control_node):
            break

        nearest_name = get_nearest_point_name(
            control_node,
            {
                '1': point_xy['1'],
                '2': point_xy['2'],
                '3': point_xy['3'],
                '4': point_xy['4'],
                '5': point_xy['5'],
                '24_mid': point_xy['24_mid'],
                '35_mid': point_xy['35_mid'],
            }
        )

        route = route_table[nearest_name]

        navigator.info(f'Gohome route: {" -> ".join(route)}')

        route_success = True

        for name in route:
            success = go_to_pose_with_home_control(
                navigator,
                control_node,
                point_pose[name],
                name
            )

            if not success:
                route_success = False
                break

        if not route_success:
            navigator.info('Gohome interrupted. Waiting again.')
            continue

        if not control_node.is_home_mode():
            continue

        dock_success = dock_without_mode_cancel(control_node)

        if dock_success:
            navigator.info('Gohome complete. Docked.')

            control_node.reset_home_stable_timer()
            control_node.clear_cancel_request()

        else:
            navigator.info('Dock failed. Waiting again.')

        time.sleep(0.5)

    navigator.info('Shutting down gohome node.')

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