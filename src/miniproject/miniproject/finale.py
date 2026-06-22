#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool
from irobot_create_msgs.action import Dock
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration


class FinaleNode(Node):

    def __init__(self):
        super().__init__('finale_node')

        self.audio_pub = self.create_publisher(
            AudioNoteVector,
            '/robot3/cmd_audio',
            10
        )

        self.finish_pub = self.create_publisher(
            Bool,
            '/robot3/mission_finished',
            10
        )

        self.dock_client = ActionClient(
            self,
            Dock,
            '/robot3/dock'
        )

        self.dock_sent = False
        self.beep_done = False
        self.finish_publish_count = 0

        self.timer = self.create_timer(
            0.5,
            self.check_and_run
        )

    def check_and_run(self):
        if self.beep_done:
            return

        if not self.dock_sent:
            if not self.dock_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info('Waiting for dock action server...')
                return

            self.send_dock_goal()
            self.dock_sent = True

        self.publish_mission_finished()

        sub_count = self.audio_pub.get_subscription_count()

        if sub_count == 0:
            self.get_logger().info('Waiting for cmd_audio subscriber...')
            return

        self.publish_beep()

    def send_dock_goal(self):
        goal_msg = Dock.Goal()
        self.dock_client.send_goal_async(goal_msg)

        self.get_logger().info('Dock goal sent.')

    def publish_mission_finished(self):
        msg = Bool()
        msg.data = True

        self.finish_pub.publish(msg)
        self.finish_publish_count += 1

        self.get_logger().info(
            f'Published mission_finished=True ({self.finish_publish_count})'
        )

    def publish_beep(self):
        msg = AudioNoteVector()
        msg.append = False

        notes_data = [
            (392, 0, 150000000),    # 솔
            (523, 0, 150000000),    # 도
            (659, 0, 150000000),    # 미
            (784, 0, 150000000),    # 솔

            (523, 0, 150000000),    # 도
            (659, 0, 150000000),    # 미
            (784, 0, 450000000),    # 솔

            (659, 0, 450000000),    # 미

            (415, 0, 150000000),    # 솔#
            (523, 0, 150000000),    # 도
            (659, 0, 150000000),    # 미
            (831, 0, 150000000),    # 솔#

            (523, 0, 150000000),    # 도
            (659, 0, 150000000),    # 미
            (831, 0, 450000000),    # 솔#

            (659, 0, 450000000),    # 미

            (440, 0, 150000000),
            (523, 0, 150000000),
            (659, 0, 150000000),

            (880, 0, 150000000),
            (523, 0, 150000000),
            (659, 0, 150000000),

            (988, 0, 300000000),    # 시

            (988, 0, 150000000),    # 시
            (988, 0, 150000000),    # 시
            (988, 0, 150000000),    # 시

            (1047, 0, 900000000),   # 도
        ]

        for freq, sec, nanosec in notes_data:
            note = AudioNote()
            note.frequency = freq
            note.max_runtime = Duration(
                sec=sec,
                nanosec=nanosec
            )
            msg.notes.append(note)

        self.audio_pub.publish(msg)

        self.get_logger().info('Published finale melody once.')

        self.beep_done = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    node = FinaleNode()

    while rclpy.ok() and not node.beep_done:
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()