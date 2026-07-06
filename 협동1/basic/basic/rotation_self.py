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
    "periodic_rotation_node",
    namespace=ROBOT_ID
)

DR_init.__dsr__node = node

from DSR_ROBOT2 import *


def main():
    try:
        print("제자리 회전 시작")

        move_periodic(
            amp=[0, 0, 0, 0, 0, 15],
            period=2.0,
            repeat=3,
            ref=DR_BASE
        )

        print("제자리 회전 완료")

    except KeyboardInterrupt:
        print("강제 종료")

    finally:
        print("ROS 2 노드 종료")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()