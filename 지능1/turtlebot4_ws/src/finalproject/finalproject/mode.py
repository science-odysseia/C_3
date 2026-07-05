#!/usr/bin/env python3

import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


# =========================
# Topic / 기본 설정
# =========================

MODE_TOPIC = '/robot3/mode'

SECTOR_HOME_REQ_TOPIC = '/robot3/sector_home_req'
PATROL_HOME_REQ_TOPIC = '/robot3/patrol_home_req'

PUBLISH_HZ = 10.0
DEFAULT_MODE = 'home'


class ModeUiNode(Node):

    def __init__(self):
        super().__init__('mode_ui_node')

        self.current_mode = DEFAULT_MODE

        # gohome 요청 상태 저장
        self.sector_home_req = False
        self.patrol_home_req = False

        self.mode_pub = self.create_publisher(
            String,
            MODE_TOPIC,
            10
        )

        self.create_subscription(
            Bool,
            SECTOR_HOME_REQ_TOPIC,
            self.sector_home_req_callback,
            10
        )

        self.create_subscription(
            Bool,
            PATROL_HOME_REQ_TOPIC,
            self.patrol_home_req_callback,
            10
        )

        self.timer = self.create_timer(
            1.0 / PUBLISH_HZ,
            self.publish_mode
        )

    def sector_home_req_callback(self, msg):
        self.sector_home_req = msg.data

    def patrol_home_req_callback(self, msg):
        self.patrol_home_req = msg.data

    def publish_mode(self):
        msg = String()

        # 둘 중 하나라도 True면 무조건 gohome
        if self.sector_home_req or self.patrol_home_req:
            msg.data = "gohome"
        else:
            msg.data = self.current_mode

        self.mode_pub.publish(msg)

    def set_mode(self, mode):
        self.current_mode = mode


class ModeUI:

    def __init__(self, ros_node):
        self.ros_node = ros_node

        self.root = tk.Tk()
        self.root.title("Robot Mode UI")
        self.root.geometry("320x300")

        title = tk.Label(
            self.root,
            text="Select Robot Mode",
            font=("Arial", 16)
        )
        title.pack(pady=15)

        tk.Button(
            self.root,
            text="patrol",
            font=("Arial", 14),
            width=20,
            command=lambda: self.ros_node.set_mode("patrol")
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="to station",
            font=("Arial", 14),
            width=20,
            command=lambda: self.ros_node.set_mode("to_station")
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="to sector",
            font=("Arial", 14),
            width=20,
            command=lambda: self.ros_node.set_mode("to_sector")
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="home",
            font=("Arial", 14),
            width=20,
            command=lambda: self.ros_node.set_mode("home")
        ).pack(pady=5)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.01)
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