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

        # 시작 후 1초 뒤 1회 실행
        self.timer = self.create_timer(
            1.0,
            self.publish_beep
        )

    def publish_beep(self):
        if self.done:
            return

        msg = AudioNoteVector()
        msg.append = False

        notes_data = [
            # 1절
            (659,0,400000000), # 미
            (494,0,200000000), # 시
            (523,0,200000000), # 도
            (587,0,400000000), # 레
            (523,0,200000000), # 도
            (494,0,200000000), # 시

            (440,0,400000000), # 라
            (440,0,200000000), # 라
            (523,0,200000000), # 도
            (659,0,400000000), # 미
            (587,0,200000000), # 레
            (523,0,200000000), # 도

            (494,0,400000000), # 시
            (494,0,200000000), # 시
            (523,0,200000000), # 도
            (587,0,400000000), # 레
            (659,0,400000000), # 미

            (523,0,400000000), # 도
            (440,0,400000000), # 라
            (440,0,800000000), # 라

            # 2절
            (0, 0, 200000000), 
            (587,0,400000000), # 레
            (698,0,200000000), # 파
            (880,0,400000000), # 라
            (784,0,200000000), # 솔
            (698,0,200000000), # 파

            (659,0,800000000), # 미
            (523,0,200000000), # 도
            (659,0,400000000), # 미
            (587,0,200000000), # 레
            (523,0,200000000), # 도

            (494,0,400000000), # 시

            # 3절
            (494,0,200000000), # 시
            (523,0,200000000), # 도
            (587,0,400000000), # 레
            (659,0,400000000), # 미

            (523,0,400000000), # 도
            (440,0,400000000), # 라
            (440,0,400000000), # 라
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
        self.get_logger().info('Published melody')

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