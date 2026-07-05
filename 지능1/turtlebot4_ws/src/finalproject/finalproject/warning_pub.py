#!/usr/bin/env python3

import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


# =========================
# 설정
# =========================

TOPIC_NAME = '/robot3/warning'
PUBLISH_HZ = 10.0


class WarningUiNode(Node):
    """
    경고음 ON/OFF UI 노드.

    버튼을 누를 때마다 상태가 토글된다.

    ON  -> True 발행
    OFF -> False 발행

    현재 상태는 계속 발행한다.
    """

    def __init__(self):
        super().__init__('warning_ui_node')

        # 처음 상태는 OFF
        self.warning_on = False

        # Publisher
        self.pub = self.create_publisher(
            Bool,
            TOPIC_NAME,
            10
        )

        # 주기적 발행
        self.timer = self.create_timer(
            1.0 / PUBLISH_HZ,
            self.publish_state
        )

    def publish_state(self):
        msg = Bool()
        msg.data = self.warning_on
        self.pub.publish(msg)

    def toggle_warning(self):
        self.warning_on = not self.warning_on

        if self.warning_on:
            self.button.config(
                text='WARNING : ON',
                bg='red',
                fg='white'
            )
        else:
            self.button.config(
                text='WARNING : OFF',
                bg='lightgray',
                fg='black'
            )

        self.get_logger().info(
            f'Warning state : {self.warning_on}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = WarningUiNode()

    root = tk.Tk()
    root.title('Warning Control')
    root.geometry('300x150')

    node.button = tk.Button(
        root,
        text='WARNING : OFF',
        bg='lightgray',
        fg='black',
        font=('Arial', 16, 'bold'),
        width=16,
        height=2,
        command=node.toggle_warning
    )

    node.button.pack(expand=True)

    def spin():
        rclpy.spin_once(node, timeout_sec=0.01)
        root.after(10, spin)

    root.after(10, spin)

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()