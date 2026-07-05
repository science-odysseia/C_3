#!/usr/bin/env python3

import tkinter as tk
import urllib.request
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# =========================
# Topic / 기본 설정
# =========================

MODE_TOPIC = '/robot3/mode'
RESERVE_TIME_TOPIC = '/robot3/reserve_time'

PUBLISH_HZ = 10.0
DEFAULT_MODE = 'home'


def get_internet_time_text():
    """
    인터넷에서 현재 시간을 받아온다.
    실패하면 로컬 시간을 사용한다.
    """

    try:
        url = 'https://worldtimeapi.org/api/timezone/Asia/Seoul'

        with urllib.request.urlopen(url, timeout=2.0) as response:
            data = json.loads(response.read().decode())

        datetime_text = data['datetime']
        dt = datetime.fromisoformat(datetime_text)

        return dt.strftime('%Y-%m-%d %H:%M:%S KST')

    except Exception:
        return datetime.now().astimezone().strftime('%Y-%m-%d %H:%M:%S %Z')


class ModeUiNode(Node):
    """
    로봇 모드를 발행하는 ROS2 노드.

    역할:
    1. 현재 선택된 모드를 저장한다.
    2. /robot3/mode 토픽을 주기적으로 발행한다.
    3. /robot3/reserve_time 토픽을 구독한다.
    4. 새로운 예약 시간이 들어오면 mode를 patrol로 변경한다.
    """

    def __init__(self):
        super().__init__('mode_ui_node')

        self.current_mode = DEFAULT_MODE

        # 이전에 받은 예약 시간
        self.prev = None

        self.mode_pub = self.create_publisher(
            String,
            MODE_TOPIC,
            10
        )

        self.reserve_time_sub = self.create_subscription(
            String,
            RESERVE_TIME_TOPIC,
            self.reserve_time_callback,
            10
        )

        self.timer = self.create_timer(
            1.0 / PUBLISH_HZ,
            self.publish_mode
        )

        self.get_logger().info(f'Publish: {MODE_TOPIC}')
        self.get_logger().info(f'Subscribe: {RESERVE_TIME_TOPIC}')
        self.get_logger().info(f'Default mode: {DEFAULT_MODE}')

    def publish_mode(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def set_mode(self, mode):
        self.current_mode = mode
        self.get_logger().info(f'Mode changed by UI: {self.current_mode}')

    def reserve_time_callback(self, msg):
        received_time = msg.data.strip()

        if self.prev is None:
            self.current_mode = 'patrol'
            self.prev = received_time

            self.get_logger().info(
                f'First reserve time received: {received_time}'
            )
            self.get_logger().info('Mode changed to patrol')
            return

        if received_time != self.prev:
            self.current_mode = 'patrol'
            self.prev = received_time

            self.get_logger().info(
                f'New reserve time received: {received_time}'
            )
            self.get_logger().info('Mode changed to patrol')
            return

        self.get_logger().info(
            f'Same reserve time ignored: {received_time}'
        )


class ModeUI:
    """
    Tkinter 기반 로봇 모드 선택 UI.
    """

    def __init__(self, ros_node):
        self.ros_node = ros_node

        self.root = tk.Tk()
        self.root.title("Robot Mode UI")
        self.root.geometry("360x360")

        title = tk.Label(
            self.root,
            text="Select Robot Mode",
            font=("Arial", 16)
        )
        title.pack(pady=10)

        self.time_label = tk.Label(
            self.root,
            text="Internet Time: -",
            font=("Arial", 11)
        )
        self.time_label.pack(pady=5)

        self.mode_label = tk.Label(
            self.root,
            text=f"Current Mode: {self.ros_node.current_mode}",
            font=("Arial", 12)
        )
        self.mode_label.pack(pady=5)

        tk.Button(
            self.root,
            text="patrol",
            font=("Arial", 14),
            width=20,
            command=lambda: self.set_mode_from_ui("patrol")
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="to station",
            font=("Arial", 14),
            width=20,
            command=lambda: self.set_mode_from_ui("to station")
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="to sector",
            font=("Arial", 14),
            width=20,
            command=lambda: self.set_mode_from_ui("to sector")
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="home",
            font=("Arial", 14),
            width=20,
            command=lambda: self.set_mode_from_ui("home")
        ).pack(pady=5)

        self.update_time_label()

    def set_mode_from_ui(self, mode):
        self.ros_node.set_mode(mode)
        self.update_mode_label()

    def update_mode_label(self):
        self.mode_label.config(
            text=f"Current Mode: {self.ros_node.current_mode}"
        )

    def update_time_label(self):
        time_text = get_internet_time_text()

        self.time_label.config(
            text=f"Internet Time: {time_text}"
        )

        self.root.after(1000, self.update_time_label)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.01)

            self.update_mode_label()
            self.root.update()


def main(args=None):
    rclpy.init(args=args)

    node = ModeUiNode()
    ui = ModeUI(node)

    try:
        ui.run()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()