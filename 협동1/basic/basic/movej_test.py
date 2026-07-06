#!/usr/bin/env python3

import sys
import rclpy
import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        "movej_test_node",
        namespace=ROBOT_ID
    )

    DR_init.__dsr__node = node

    import DSR_ROBOT2 as dsr

    try:
        print("MoveJ 시작")

        dsr.movej(
            dsr.posj(0, 0, 90, 0, 90, 0),
            vel=200,
            acc=200
        )

        print("MoveJ 완료")

    except Exception as e:
        print(f"오류: {e}")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()