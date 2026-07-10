#!/usr/bin/env python3
import sys
import time
import threading

import rclpy
import DR_init

from copy import deepcopy
from dsr_msgs2.srv import GetToolForce


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY = 200
ACC = 200

DOWN_DISTANCE = 50.0
FORCE_LIMIT_FZ = 10.0

gripper_status = 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


rclpy.init(args=sys.argv)

move_node = rclpy.create_node(
    "gear_move_node",
    namespace=ROBOT_ID
)

force_node = rclpy.create_node(
    "gear_force_node",
    namespace=ROBOT_ID
)

DR_init.__dsr__node = move_node

from DSR_ROBOT2 import (
            movej, movel, set_digital_output,
            wait, OFF, ON, task_compliance_ctrl,
            set_desired_force, release_force, release_compliance_ctrl,
            DR_FC_MOD_REL, set_ref_coord, move_periodic, DR_BASE
        )
from DR_common2 import posx, posj

class ForceDetector:
    def __init__(self, node):
        self.node = node
        self.detected = False
        self.running = True

        self.force_client = node.create_client(
            GetToolForce,
            "aux_control/get_tool_force"
        )

    def start(self):
        self.thread = threading.Thread(
            target=self.force_loop,
            daemon=True
        )
        self.thread.start()

    def stop(self):
        self.running = False

    def force_loop(self):
        self.node.get_logger().info("Force monitor started")
        self.force_client.wait_for_service()

        while rclpy.ok() and self.running:
            req = GetToolForce.Request()
            req.ref = 0

            future = self.force_client.call_async(req)

            rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=0.3
            )

            if not future.done():
                time.sleep(0.05)
                continue

            result = future.result()

            if result is None or not result.success:
                time.sleep(0.05)
                continue

            fz = result.tool_force[2]

            self.node.get_logger().info(
                f"Current Fz = {fz:.2f} N"
            )

            if abs(fz) > FORCE_LIMIT_FZ:
                self.node.get_logger().warn(
                    f"Force detected: Fz={fz:.2f} N"
                )
                self.detected = True
                self.running = False
                break

            time.sleep(0.05)


def open_gripper():
    global gripper_status

    print("그리퍼 열기")

    set_digital_output(1, OFF)
    set_digital_output(2, OFF)
    set_digital_output(3, OFF)
    set_digital_output(1, ON)

    wait(0.5)

    gripper_status = 0


def close_gripper():
    global gripper_status

    print("블록 집기")

    set_digital_output(1, OFF)
    set_digital_output(2, OFF)
    set_digital_output(3, OFF)
    set_digital_output(2, ON)

    wait(0.5)

    gripper_status = 1

def full_close_gripper():
    global gripper_status

    print("그리퍼 닫기")

    set_digital_output(1, OFF)
    set_digital_output(2, OFF)
    set_digital_output(3, OFF)
    set_digital_output(3, ON)

    wait(0.5)

    gripper_status = 1


def start_force_control():
    fd = [0, 0, 15, 0, 0, 0]
    fc_dir = [0, 0, 1, 0, 0, 0]

    task_compliance_ctrl(
        [3000, 3000, 3000, 200, 200, 200],
        0
    )

    wait(0.2)

    set_desired_force(
        fd,
        dir=fc_dir,
        mod=DR_FC_MOD_REL
    )

    wait(0.2)

def stop_force_control():
    release_force()
    release_compliance_ctrl()


def wait_force_detect_once():
    force_detector = ForceDetector(force_node)

    force_detector.start()

    print("힘 감시 시작")

    while rclpy.ok() and not force_detector.detected:
        wait(0.05)

    force_detector.stop()

    return force_detector.detected


def force_place_final(place_pose_up):
    print("마지막 기어 힘제어 삽입 위치 이동")

    movel(place_pose_up, VELOCITY, ACC)

    try:
        # ===============================
        # 1차 힘제어
        # 힘 감지 → 힘제어 즉시 해제 → 회전 1회
        # ===============================

        print("1차 힘제어 시작")
        start_force_control()

        print("1차 아래 방향 힘제어 중...")

        detected_1 = wait_force_detect_once()

        if detected_1:
            print("1차 힘 감지됨. 힘제어 해제 후 회전 1회 수행")

            stop_force_control()

            wait(0.05)

            move_periodic(
                amp=[0, 0, 0, 0, 0, 15],
                period=2.0,
                repeat=1,
                ref=DR_BASE
            )

            print("1차 회전 완료")

        else:
            print("1차 힘 감지 실패")
            stop_force_control()
            return False

        print("1차 삽입 후 1초 대기")
        wait(1.0)

        # ===============================
        # 2차 힘제어
        # 다시 힘제어 ON → 힘 감지 → 힘제어 해제
        # ===============================

        print("2차 힘제어 시작")
        start_force_control()

        print("2차 아래 방향 힘제어 중...")

        detected_2 = wait_force_detect_once()

        if detected_2:
            print("2차 힘 감지됨. 힘제어 해제")

            stop_force_control()

        else:
            print("2차 힘 감지 실패")
            stop_force_control()
            return False

        wait(0.2)

        print("마지막 기어 내려놓기")
        open_gripper()

        print("상승")
        movel(place_pose_up)

        return True

    except KeyboardInterrupt:
        print("마지막 힘제어 중 강제 종료")

        try:
            stop_force_control()
        except Exception:
            pass

        return False

    except Exception as e:
        print(f"마지막 힘제어 중 예외 발생: {e}")

        try:
            stop_force_control()
        except Exception:
            pass

        return False


# def pick_and_force_place_final(pick_pose_up, place_pose_up):
#     print("마지막 기어 힘제어 Place 시작")

#     # pick_success = pick_at(pick_pose_up)

#     if pick_success == False:
#         print("마지막 기어 Pick failed.")
#         return False

#     place_success = force_place_final(place_pose_up)

#     if place_success == False:
#         print("마지막 기어 Force Place failed.")
#         return False

#     print("마지막 기어 힘제어 Place 완료")
#     return True


def main():
    homej = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    homex = posx([367.31, 8.71, 194.87, 91.63, -180.00, 91.26])
    print("로봇 상태 데이터를 수신하기 위해 대기합니다...")

    rclpy.spin_once(move_node, timeout_sec=0.1)

    try:
        movej(homej, vel=VELOCITY, acc=ACC)
        # 5. [X축 푸시] 컴플라이언스를 켜고 X축으로 먼저 밀어버림
        task_compliance_ctrl([3000, 3000, 5000, 200, 200, 200], 0); wait(0.1)
        # X축 정방향 힘(8N)과 블록이 들리지 않게 지시 가압(Z축 5N) 동시 부여
        set_desired_force([8.0, 0.0, 0.0, 0, 0, 0], dir=[1, 0, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
        wait(1.2)  # X축 밀기 시간
        release_force(); wait(0.1)
        
        # 6. [Y축 푸시] 이어서 Y축으로 밀어버림
        # Y축 정방향 힘(8N)과 지시 가압(Z축 5N)
        set_desired_force([0.0, 8.0, 0.0, 0, 0, 0], dir=[0, 1, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
        wait(1.2)  # Y축 밀기 시간
        release_force(); wait(0.1)
        
        # 7. [최종 Z 가압] 구석에 들어갔으니 위에서 정면으로 꾹 누르기
        release_compliance_ctrl() # 밀기 모드 해제
        
        # 정수 목표 위치의 완전 상부로 이동하여 프레스 준비
        pose_final_press = deepcopy(homex)
        pose_final_press[2] -= 10.0  # 목표 10mm 위로 이동
        movel(pose_final_press, vel=150, acc=ACC)
        
        # 컴플라이언스 재가동 후 25N 가압
        task_compliance_ctrl([4000, 4000, 20000, 200, 200, 200], 0); wait(0.1)
        set_desired_force([0, 0, 25.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        move_periodic(amp=[0, 0, 0, 0, 0, 2], period=1.0, repeat=1, ref=DR_BASE); wait(0.5)
        
        # 루틴 종료 및 안전 탈출
        release_force(); release_compliance_ctrl(); wait(0.2)

    except KeyboardInterrupt:
        print("강제 종료 시그널 감지!")

        try:
            stop_force_control()
        except Exception:
            pass

    finally:
        print("ROS 2 노드를 안전하게 종료합니다.")
        movej(homej, vel=VELOCITY, acc=ACC)
        try:
            move_node.destroy_node()
            force_node.destroy_node()
        except Exception:
            pass

        rclpy.shutdown()


if __name__ == "__main__":
    main()