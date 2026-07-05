#!/usr/bin/env python3

import math
import time
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator
)

from nav2_simple_commander.robot_navigator import TaskResult


AMCL_TOPIC = '/robot3/amcl_pose'

THRESHOLD_TOPIC = '/robot3/battery_threshold'
BATTERY_TOPIC = '/robot3/battery_state'
GOHOME_SECTOR_TOPIC = '/robot3/gohome_sector'
CMD_VEL_TOPIC = '/robot3/cmd_vel'

STOP_PUBLISH_HZ = 50.0
STOP_PERIOD = 1.0 / STOP_PUBLISH_HZ


class BatterySafetyNode(Node):

    def __init__(self):
        super().__init__('battery_safety_node')

        self.threshold = None
        self.battery_percentage = None

        self.paused = False
        self.pause_requested = False
        self.stop_requested = False

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            CMD_VEL_TOPIC,
            10
        )

        self.gohome_sector_pub = self.create_publisher(
            Bool,
            GOHOME_SECTOR_TOPIC,
            10
        )

        self.create_subscription(
            Float32,
            THRESHOLD_TOPIC,
            self.threshold_callback,
            10
        )

        self.create_subscription(
            BatteryState,
            BATTERY_TOPIC,
            self.battery_callback,
            10
        )

        self.timer = self.create_timer(
            STOP_PERIOD,
            self.timer_callback
        )

    def threshold_callback(self, msg):
        self.threshold = float(msg.data)
        self.update_pause_state()

    def battery_callback(self, msg):
        self.battery_percentage = msg.percentage * 100.0
        self.update_pause_state()

    def update_pause_state(self):
        if self.threshold is None or self.battery_percentage is None:
            return

        prev_paused = self.paused

        self.paused = self.battery_percentage <= self.threshold

        if self.paused and not prev_paused:
            self.pause_requested = True
            self.get_logger().warn(
                f'Battery pause ON: '
                f'{self.battery_percentage:.1f}% <= {self.threshold:.1f}%'
            )

        elif not self.paused and prev_paused:
            self.get_logger().info(
                f'Battery pause OFF: '
                f'{self.battery_percentage:.1f}% > {self.threshold:.1f}%'
            )

    def timer_callback(self):
        self.publish_gohome_sector(self.paused)

        if self.paused:
            self.publish_zero_cmd()

    def publish_zero_cmd(self):
        self.cmd_vel_pub.publish(Twist())

    def publish_gohome_sector(self, value):
        msg = Bool()
        msg.data = value
        self.gohome_sector_pub.publish(msg)

    def force_stop_robot(self, duration_sec=0.2):
        count = int(duration_sec * STOP_PUBLISH_HZ)

        for _ in range(count):
            self.publish_zero_cmd()
            self.publish_gohome_sector(True)
            time.sleep(STOP_PERIOD)

    def is_paused(self):
        return self.paused


def spin_safety_node(executor, safety_node):
    while rclpy.ok() and not safety_node.stop_requested:
        executor.spin_once(timeout_sec=0.01)


def battery_cancel_monitor(navigator, safety_node, nav_lock):
    while rclpy.ok() and not safety_node.stop_requested:

        if safety_node.pause_requested:
            safety_node.pause_requested = False

            navigator.info('Battery monitor: cancel current goal immediately.')

            try:
                with nav_lock:
                    navigator.cancelTask()
            except Exception as e:
                navigator.info(f'cancelTask failed: {e}')

            safety_node.force_stop_robot(1.0)

        if safety_node.is_paused():
            safety_node.publish_zero_cmd()
            safety_node.publish_gohome_sector(True)

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


def distance_xy(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def go_to_pose_with_battery_pause(navigator, safety_node, nav_lock, pose, name):
    while rclpy.ok():

        while rclpy.ok() and safety_node.is_paused():
            safety_node.force_stop_robot(0.2)
            time.sleep(0.05)

        safety_node.publish_gohome_sector(False)

        navigator.info(f'Going to {name}...')

        with nav_lock:
            navigator.startToPose(pose)

        was_paused = False

        while rclpy.ok():

            if safety_node.is_paused():
                was_paused = True
                safety_node.force_stop_robot(0.2)
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
                return result == TaskResult.SUCCEEDED

            time.sleep(0.02)

    return False


def run_pose_sequence(navigator, safety_node, nav_lock, route_names, point_data):
    for name in route_names:
        pose = navigator.getPoseStamped(
            point_data[name]['xy'],
            point_data[name]['direction']
        )

        success = go_to_pose_with_battery_pause(
            navigator,
            safety_node,
            nav_lock,
            pose,
            name
        )

        if not success:
            return False

    return True


def shutdown_all(navigator, safety_node, safety_executor, safety_thread, monitor_thread):
    safety_node.stop_requested = True

    try:
        navigator.cancelTask()
    except Exception:
        pass

    safety_node.force_stop_robot(1.0)
    safety_node.publish_gohome_sector(False)

    safety_executor.shutdown()
    safety_thread.join(timeout=1.0)
    monitor_thread.join(timeout=1.0)

    safety_node.destroy_node()
    navigator.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


def main():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/robot3'])

    navigator = TurtleBot4Navigator()
    safety_node = BatterySafetyNode()
    nav_lock = threading.Lock()

    safety_executor = SingleThreadedExecutor()
    safety_executor.add_node(safety_node)

    safety_thread = threading.Thread(
        target=spin_safety_node,
        args=(safety_executor, safety_node),
        daemon=True
    )
    safety_thread.start()

    monitor_thread = threading.Thread(
        target=battery_cancel_monitor,
        args=(navigator, safety_node, nav_lock),
        daemon=True
    )
    monitor_thread.start()

    navigator.info('Nearest point + immediate battery cancel node started.')

    # =========================
    # 도킹 상태면 먼저 undock
    # =========================

    # =========================
    # 도킹 상태 확인
    # =========================

    if navigator.getDockedStatus():

        navigator.info('Robot is docked.')

        # 배터리가 기준치보다 낮으면 계속 도킹 유지
        while rclpy.ok() and safety_node.is_paused():

            navigator.info(
                'Battery is below threshold. Waiting while staying docked...'
            )

            time.sleep(0.5)

        navigator.info('Battery threshold cleared. Undocking...')

        navigator.undock()

        navigator.info('Undock finished. Continue mission.')

    else:

        navigator.info('Robot is already undocked. Continue mission.')

    navigator.info(f'Reading current pose from {AMCL_TOPIC}...')

    current_x, current_y = get_current_xy_once()

    if current_x is None or current_y is None:
        navigator.info('Failed to read current position. Node will shutdown.')
        shutdown_all(
            navigator,
            safety_node,
            safety_executor,
            safety_thread,
            monitor_thread
        )
        return

    navigator.info(
        f'Current Position: x={current_x:.3f}, y={current_y:.3f}'
    )

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
        'direction': TurtleBot4Directions.EAST
    }

    navigator.info(
        f'Target point: x={target_xy[0]:.3f}, y={target_xy[1]:.3f}'
    )

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

        dist = distance_xy(
            current_x,
            current_y,
            px,
            py
        )

        navigator.info(f'{name}: distance = {dist:.3f} m')

        if nearest_distance is None or dist < nearest_distance:
            nearest_name = name
            nearest_distance = dist

    navigator.info(
        f'Nearest point: {nearest_name}, distance={nearest_distance:.3f} m'
    )

    navigator.info('Waiting for NavigateToPose action server...')
    while not navigator.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
        navigator.info('NavigateToPose action server not available, waiting...')

    route_names = [nearest_name]

    if nearest_name in ['point2', 'point4']:
        route_names += [
            'point24_mid',
            'target'
        ]

    elif nearest_name in ['point3', 'point5']:
        route_names += [
            'point35_mid',
            'target'
        ]

    elif nearest_name == 'point1':
        route_names += [
            'point2',
            'point24_mid',
            'target'
        ]

    elif nearest_name in ['point24_mid', 'point35_mid']:
        route_names += [
            'target'
        ]

    navigator.info(f'Final route: {" -> ".join(route_names)}')

    success = run_pose_sequence(
        navigator,
        safety_node,
        nav_lock,
        route_names,
        point_data
    )

    if success:
        navigator.info('Final target route completed. Node will shutdown.')
    else:
        navigator.info('Final target route failed. Node will shutdown.')

    shutdown_all(
        navigator,
        safety_node,
        safety_executor,
        safety_thread,
        monitor_thread
    )


if __name__ == '__main__':
    main()