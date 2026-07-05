#!/usr/bin/env python3

import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


THRESHOLD_TOPIC = '/robot3/battery_threshold'

DEFAULT_THRESHOLD = 130.0
MAX_THRESHOLD = 150.0
PUBLISH_HZ = 10.0


class BatteryThresholdPublisherNode(Node):

    def __init__(self, get_threshold_callback):
        super().__init__('battery_threshold_publisher_node')

        self.get_threshold = get_threshold_callback

        self.threshold_pub = self.create_publisher(
            Float32,
            THRESHOLD_TOPIC,
            10
        )

        self.timer = self.create_timer(
            1.0 / PUBLISH_HZ,
            self.publish_threshold
        )

    def publish_threshold(self):
        threshold = float(self.get_threshold())

        msg = Float32()
        msg.data = threshold

        self.threshold_pub.publish(msg)


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

    node = BatteryThresholdPublisherNode(
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