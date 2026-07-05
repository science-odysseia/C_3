#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration


WARNING_TOPIC = '/robot3/warning'
AUDIO_TOPIC = '/robot3/cmd_audio'

# 이 시간 안에 True가 다시 안 들어오면 자동 OFF 처리
WARNING_TIMEOUT = 1.0


class BeepNode(Node):

    def __init__(self):
        super().__init__('beep_node')

        self.pub = self.create_publisher(
            AudioNoteVector,
            AUDIO_TOPIC,
            10
        )

        self.create_subscription(
            Bool,
            WARNING_TOPIC,
            self.warning_callback,
            10
        )

        self.warning = False
        self.last_true_time = None

        # 0.5초마다 상태 확인
        self.timer = self.create_timer(
            0.5,
            self.timer_callback
        )

    def warning_callback(self, msg):
        now = self.get_clock().now()

        if msg.data is True:
            self.last_true_time = now

            if not self.warning:
                self.warning = True
                self.get_logger().info('Warning ON')
        else:
            if self.warning:
                self.warning = False
                self.get_logger().info('Warning OFF')
                self.stop_beep()

            self.last_true_time = None

    def timer_callback(self):
        if self.pub.get_subscription_count() == 0:
            return

        # True를 받은 적이 없으면 작동 안 함
        if self.last_true_time is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_true_time).nanoseconds / 1e9

        # 최근 True가 일정 시간 안에 안 들어오면 자동 OFF
        if elapsed > WARNING_TIMEOUT:
            if self.warning:
                self.warning = False
                self.get_logger().info('Warning timeout -> OFF')
                self.stop_beep()
            return

        # 최근에 True가 들어온 경우에만 삐뽀삐뽀
        if self.warning:
            self.publish_beep()

    def publish_beep(self):
        msg = AudioNoteVector()
        msg.append = False

        notes_data = [
            (784, 0, 100000000),  # 삐
            (523, 0, 100000000),  # 뽀
            (784, 0, 100000000),  # 삐
            (523, 0, 100000000),  # 뽀
            (784, 0, 100000000),  # 삐
            (523, 0, 100000000),  # 뽀
            (784, 0, 100000000),  # 삐
            (523, 0, 100000000),  # 뽀
            (784, 0, 100000000),  # 삐
            (523, 0, 100000000),  # 뽀
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

    def stop_beep(self):
        msg = AudioNoteVector()
        msg.append = False
        self.pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = BeepNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()