#!/usr/bin/env python3

import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.action import Undock
from rclpy.action import ActionClient


DEFAULT_THRESHOLD = 130.0
MAX_THRESHOLD = 150.0


class BatteryUndockNode(Node):

    def __init__(self, get_threshold_callback):
        super().__init__('battery_undock_node')

        self.get_threshold = get_threshold_callback
        self.undock_sent = False

        self.undock_client = ActionClient(
            self,
            Undock,
            '/robot3/undock'
        )

        self.create_subscription(
            BatteryState,
            '/robot3/battery_state',
            self.battery_callback,
            10
        )

    def battery_callback(self, msg):
        percentage = msg.percentage * 100.0
        threshold = self.get_threshold()

        self.get_logger().info(
            f'Battery: {percentage:.1f}% / Threshold: {threshold:.1f}%'
        )

        if self.undock_sent:
            return

        if percentage >= threshold:
            self.undock_sent = True

            self.get_logger().info(
                f'Battery >= {threshold:.1f}% -> send undock goal'
            )

            self.send_undock_goal()

    def send_undock_goal(self):

        if not self.undock_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                '/robot3/undock action server not found'
            )
            self.undock_sent = False
            return

        goal_msg = Undock.Goal()

        future = self.undock_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected')
            self.undock_sent = False
            return

        self.get_logger().info('Undock goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info('Undock finished')


class BatteryThresholdUI:

    def __init__(self):

        self.root = tk.Tk()
        self.root.title('Battery Undock Threshold')
        self.root.geometry('500x150')

        self.threshold_var = tk.DoubleVar(
            value=DEFAULT_THRESHOLD
        )

        self.label = tk.Label(
            self.root,
            text=f'Undock Battery Threshold: {DEFAULT_THRESHOLD:.0f}%',
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
            text=f'Undock Battery Threshold: {float(value):.0f}%'
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

    node = BatteryUndockNode(
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