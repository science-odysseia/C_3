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
    "force_test_node",
    namespace=ROBOT_ID
)

DR_init.__dsr__node = node

from DSR_ROBOT2 import *


def main():
    try:
        print("힘제어 ON")

        fd = [0, 0, -30, 0, 0, 0]
        fc_dir = [0, 0, 1, 0, 0, 0]

        task_compliance_ctrl(
            [20000, 20000, 20000, 200, 200, 200],
            0
        )

        wait(0.2)

        set_desired_force(
            fd,
            dir=fc_dir,
            mod=DR_FC_MOD_REL
        )

        wait(1.0)

        print("힘제어 OFF")

        release_force()
        release_compliance_ctrl()

    except KeyboardInterrupt:
        print("강제 종료")

        try:
            release_force()
            release_compliance_ctrl()
        except Exception:
            pass

    finally:
        print("ROS 2 노드 종료")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()