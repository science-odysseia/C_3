#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool
from irobot_create_msgs.action import Undock


UNDERBAT_TOPIC = '/robot3/underbat'
UNDOCK_ACTION = '/robot8/undock'


class UnderbatUndockNode(Node):

    def __init__(self):
        super().__init__('underbat_undock_node')

        self.undock_sent = False

        self.client = ActionClient(
            self,
            Undock,
            UNDOCK_ACTION
        )

        self.create_subscription(
            Bool,
            UNDERBAT_TOPIC,
            self.underbat_callback,
            10
        )

        self.get_logger().info(
            f'Waiting for {UNDERBAT_TOPIC} == True...'
        )

    def underbat_callback(self, msg):

        if self.undock_sent:
            return

        if not msg.data:
            return

        self.undock_sent = True

        self.get_logger().info(
            f'{UNDERBAT_TOPIC} is True -> send undock goal'
        )

        self.send_undock_goal()

    def send_undock_goal(self):

        self.get_logger().info('Waiting for Undock action server...')

        if not self.client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                f'{UNDOCK_ACTION} action server not found'
            )
            self.undock_sent = False
            return

        self.get_logger().info('Undock action server connected.')

        goal_msg = Undock.Goal()

        self.send_goal_future = self.client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected.')
            self.undock_sent = False
            return

        self.get_logger().info('Undock goal accepted.')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        result = future.result()

        self.get_logger().info(
            f'Undock finished. Status: {result.status}'
        )

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = UnderbatUndockNode()

    rclpy.spin(node)


if __name__ == '__main__':
    main()