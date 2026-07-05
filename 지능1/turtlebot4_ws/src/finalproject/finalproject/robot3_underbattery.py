#!/usr/bin/env python3

import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState


BATTERY_TOPIC = '/robot3/battery_state'
UNDERBAT_TOPIC = '/robot3/underbat'

DEFAULT_THRESHOLD = 130.0
MAX_THRESHOLD = 150.0


class BatteryUnderbatNode(Node):

    def __init__(self, get_threshold_callback):
        super().__init__('battery_underbat_node')

        self.get_threshold = get_threshold_callback

        self.underbat_pub = self.create_publisher(
            Bool,
            UNDERBAT_TOPIC,
            10
        )

        self.create_subscription(
            BatteryState,
            BATTERY_TOPIC,
            self.battery_callback,
            10
        )

    def battery_callback(self, msg):
        percentage = msg.percentage * 100.0
        threshold = float(self.get_threshold())

        underbat = percentage < threshold

        underbat_msg = Bool()
        underbat_msg.data = underbat

        self.underbat_pub.publish(underbat_msg)

        self.get_logger().info(
            f'Battery: {percentage:.1f}% / Threshold: {threshold:.1f}% / Underbat: {underbat}'
        )


class BatteryThresholdUI:

    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Battery Threshold Publisher')
        self.root.geometry('500x150')

        self.threshold_var = tk.DoubleVar(
            value=DEFAULT_THRESHOLD
        )

        self.label = tk.Label(
            self.root,
            text=f'Battery Threshold: {DEFAULT_THRESHOLD:.0f}%',
            font=('Arial', 12)
        )
        self.label.pack(pady=10)

        self.slider = tk.Scale(
            self.root,
            from_=0,
            to=MAX_THRESHOLD,
            orient=tk.HORIZONTAL,
            length=400,
            resolution=1,
            variable=self.threshold_var,
            command=self.update_label
        )
        self.slider.pack()

    def update_label(self, value):
        self.label.config(
            text=f'Battery Threshold: {float(value):.0f}%'
        )

    def get_threshold(self):
        return self.threshold_var.get()

    def run(self):
        self.root.mainloop()


def ros_spin(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)

    ui = BatteryThresholdUI()

    node = BatteryUnderbatNode(
        ui.get_threshold
    )

    ros_thread = threading.Thread(
        target=ros_spin,
        args=(node,),
        daemon=True
    )
    ros_thread.start()

    try:
        ui.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()