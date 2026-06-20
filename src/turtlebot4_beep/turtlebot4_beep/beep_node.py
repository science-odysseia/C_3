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
        self.published = False
        self.shutdown_timer = None

        self.timer = self.create_timer(
            0.5,
            self.check_and_publish
        )

    def check_and_publish(self):
        if self.done or self.published:
            return

        if self.pub.get_subscription_count() == 0:
            self.get_logger().info(
                'Waiting for cmd_audio subscriber...'
            )
            return

        self.publish_beep()

    def publish_beep(self):
        if self.published:
            return

        msg = AudioNoteVector()
        msg.append = False

        notes_data = [
            (659, 0, 400000000),
            (494, 0, 200000000),
            (523, 0, 200000000),
            (587, 0, 400000000),
            (523, 0, 200000000),
            (494, 0, 200000000),

            (440, 0, 400000000),
            (440, 0, 200000000),
            (523, 0, 200000000),
            (659, 0, 400000000),
            (587, 0, 200000000),
            (523, 0, 200000000),

            (494, 0, 400000000),
            (494, 0, 200000000),
            (523, 0, 200000000),
            (587, 0, 400000000),
            (659, 0, 400000000),

            (523, 0, 400000000),
            (440, 0, 400000000),
            (440, 0, 800000000),

            (587, 0, 200000000),
            (587, 0, 400000000),
            (698, 0, 200000000),
            (880, 0, 400000000),
            (784, 0, 200000000),
            (698, 0, 200000000),

            (659, 0, 400000000),
            (659, 0, 200000000),
            (523, 0, 200000000),
            (659, 0, 400000000),
            (587, 0, 200000000),
            (523, 0, 200000000),

            (494, 0, 400000000),

            (494, 0, 200000000),
            (523, 0, 200000000),
            (587, 0, 400000000),
            (659, 0, 400000000),

            (523, 0, 400000000),
            (440, 0, 400000000),
            (440, 0, 400000000),
        ]

        for freq, sec, nanosec in notes_data:
            note = AudioNote()
            note.frequency = int(freq)
            note.max_runtime = Duration(
                sec=sec,
                nanosec=nanosec
            )
            msg.notes.append(note)

        self.pub.publish(msg)
        self.published = True

        self.get_logger().info('Published melody once')

        self.timer.cancel()

        self.shutdown_timer = self.create_timer(
            2.0,
            self.finish
        )

    def finish(self):
        self.done = True

        if self.shutdown_timer:
            self.shutdown_timer.cancel()

        self.get_logger().info('Beep node finished')


def main(args=None):
    rclpy.init(args=args)

    node = BeepNode()

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()