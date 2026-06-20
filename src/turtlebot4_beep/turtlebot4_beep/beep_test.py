#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration


class BeepNode(Node):
    def __init__(self):
        super().__init__('beep_node')

        self.pub = self.create_publisher(
            AudioNoteVector,
            '/robot3/cmd_audio',
            10
        )

        self.done = False

        # 0.5초마다 구독자 연결 확인
        self.timer = self.create_timer(
            0.5,
            self.check_and_publish
        )

    def check_and_publish(self):
        if self.done:
            return

        sub_count = self.pub.get_subscription_count()

        if sub_count == 0:
            self.get_logger().info('Waiting for cmd_audio subscriber...')
            return

        self.publish_beep()

    def publish_beep(self):
        msg = AudioNoteVector()
        msg.append = False

        notes_data = [
            (880, 0, 400000000),
            (440, 0, 400000000),
            (880, 0, 400000000),
            (440, 0, 400000000),
            (880, 0, 200000000),
            (440, 0, 200000000),
            (880, 0, 200000000),
            (440, 0, 200000000),
        ]

        for freq, sec, nanosec in notes_data:
            note = AudioNote()
            note.frequency = freq
            note.max_runtime = Duration(
                sec=sec,
                nanosec=nanosec
            )
            msg.notes.append(note)

        self.pub.publish(msg)
        self.get_logger().info('Published melody once')

        self.done = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    node = BeepNode()

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()