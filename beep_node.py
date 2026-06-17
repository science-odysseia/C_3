import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration


class BeepNode(Node):
    def __init__(self):
        super().__init__('beep_node')

        self.pub = self.create_publisher(
            AudioNoteVector,
            '/robot3/cmd_audio',
            10
        )

        self.timer = self.create_timer(2.0, self.play_sound)

    def play_sound(self):
        msg = AudioNoteVector()
        msg.append = False

        pattern = [
        # 학교종이
            (784, 180), (784, 180),
            (659, 180), (659, 180),

        # 땡땡
            (784, 180), (784, 180),
            (659, 180), (659, 180),

        # 땡땡땡 (마무리)
            (784, 180), (784, 180),
            (659, 180), (659, 180),

        # 마지막 길게
            (523, 300), (523, 300), (523, 500),
    ]

        msg.notes = []

        for freq, ms in pattern:
            note = AudioNote()
            note.frequency = int(freq)  # 반드시 int
            note.max_runtime = Duration(
                sec=0,
                nanosec=int(ms * 1_000_000)
        )
            msg.notes.append(note)

        self.pub.publish(msg)
        self.get_logger().info("🔔 학교종이 땡땡땡 출력")


def main():
    rclpy.init()
    node = BeepNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
