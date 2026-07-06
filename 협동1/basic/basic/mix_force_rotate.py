#!/usr/bin/env python3

import sys
import time

import rclpy
import DR_init

from dsr_msgs2.srv import MoveStop
from dsr_msgs2.srv import GetToolForce


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

FORCE_LIMIT_FZ = 10.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


rclpy.init(args=sys.argv)

node = rclpy.create_node(
    "force_rotation_test_node",
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


def get_fz_once(force_client):
    req = GetToolForce.Request()
    req.ref = 0

    future = force_client.call_async(req)

    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=0.3
    )

    if not future.done():
        return None

    result = future.result()

    if result is None or not result.success:
        return None

    return result.tool_force[2]


def wait_until_force_detected():
    force_client = node.create_client(
        GetToolForce,
        "aux_control/get_tool_force"
    )

    if not force_client.wait_for_service(timeout_sec=2.0):
        print("aux_control/get_tool_force 서비스 없음")
        return False

    print("힘 감시 시작")

    while rclpy.ok():
        fz = get_fz_once(force_client)

        if fz is None:
            wait(0.05)
            continue

        print(f"Current Fz = {fz:.2f} N")

        if fz >= FORCE_LIMIT_FZ:
            print(f"힘 감지됨: Fz={fz:.2f} N")
            return True

        wait(0.05)

    return False


def start_force_control():
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


def stop_force_control():
    print("힘제어 OFF")

    release_force()
    release_compliance_ctrl()


def main():
    try:
        start_force_control()

        print("비동기 제자리 회전 시작")

        amove_periodic(
            amp=[0, 0, 0, 0, 0, 15],
            period=2.0,
            repeat=5,
            ref=DR_BASE
        )

        detected = wait_until_force_detected()

        if detected:
            print("회전 정지")
            motion_stop()
        else:
            print("힘 감지 실패")
            motion_stop()

        print("추가 2초 동안 힘제어만 유지")

        wait(2.0)

        stop_force_control()

        print("테스트 완료")

    except KeyboardInterrupt:
        print("강제 종료")

        try:
            motion_stop()
        except Exception:
            pass

        try:
            stop_force_control()
        except Exception:
            pass

    finally:
        print("ROS 2 노드 종료")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()