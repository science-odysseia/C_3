#!/usr/bin/env python3

import sys

import rclpy
import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


rclpy.init(args=sys.argv)

node = rclpy.create_node(
    "async_periodic_rotation_node",
    namespace=ROBOT_ID
)

DR_init.__dsr__node = node

from DSR_ROBOT2 import *


def main():
    try:
        print("비동기 제자리 회전 시작")

        amove_periodic(
            amp=[0, 0, 0, 0, 0, 15],
            period=2.0,
            repeat=5,
            ref=DR_BASE
        )

        print("비동기 명령 전송 완료")

        while rclpy.ok():
            wait(0.1)

    except KeyboardInterrupt:
        print("강제 종료")

        try:
            drl_script_stop()
        except Exception:
            pass

    finally:
        print("ROS 2 노드 종료")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()