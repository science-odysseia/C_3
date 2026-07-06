#!/usr/bin/env python3

import sys

import rclpy
import DR_init

from dsr_msgs2.srv import MoveStop


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


rclpy.init(args=sys.argv)

node = rclpy.create_node(
    "async_periodic_rotation_stop_node",
    namespace=ROBOT_ID
)

DR_init.__dsr__node = node

from DSR_ROBOT2 import *


def motion_stop():
    print("MoveStop 서비스 호출")

    client = node.create_client(
        MoveStop,
        "motion/move_stop"
    )

    if not client.wait_for_service(timeout_sec=2.0):
        print("motion/move_stop 서비스 없음")
        return False

    req = MoveStop.Request()
    req.stop_mode = DR_SSTOP

    future = client.call_async(req)

    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=2.0
    )

    if not future.done():
        print("MoveStop 응답 없음")
        return False

    result = future.result()

    if result is None:
        print("MoveStop 실패")
        return False

    print("MoveStop 완료")
    return True


def main():
    try:
        print("비동기 제자리 회전 시작")

        amove_periodic(
            amp=[0, 0, 0, 0, 0, 15],
            period=2.0,
            repeat=5,
            ref=DR_BASE
        )

        print("2초 대기")

        wait(2.0)

        print("회전 강제 정지")

        motion_stop()

        wait(0.5)

    except KeyboardInterrupt:
        print("강제 종료")

        try:
            motion_stop()
        except Exception:
            pass

    finally:
        print("ROS 2 노드 종료")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()