#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from irobot_create_msgs.action import Undock


class UndockNode(Node):

    def __init__(self):
        super().__init__('undock_node')

        self.client = ActionClient(
            self,
            Undock,
            '/robot8/undock'
        )

        self.get_logger().info('Waiting for Undock action server...')

        self.client.wait_for_server()

        self.get_logger().info('Undock action server connected.')

        goal_msg = Undock.Goal()

        self.send_goal_future = self.client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected.')
            rclpy.shutdown()
            return

        self.get_logger().info('Undock goal accepted.')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()

        self.get_logger().info(f'Undock finished. Status: {result.status}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = UndockNode()

    rclpy.spin(node)


if __name__ == '__main__':
    main()