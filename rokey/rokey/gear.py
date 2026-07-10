#!/usr/bin/env python3
import sys
import time
import threading
import copy

import rclpy
import DR_init

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

move_node = rclpy.create_node("gear_move_node",namespace=ROBOT_ID)
force_node = rclpy.create_node("gear_force_node",namespace=ROBOT_ID)

DR_init.__dsr__node = move_node

from DSR_ROBOT2 import *

start_pose = posx(348.56, -79.14, 92.64, 121.22, -179.42, 128.61)

# ===============================
# 1번 기어
# ===============================
pick_pose_up_1 = posx(296.91, -51.48, 95.61, 120.29, -179.20, 128.12)
place_pose_up_1 = posx(539.98, 44.18, 94.73, 110.77, -179.05, 118.34)

# ===============================
# 2번 기어
# ===============================
pick_pose_up_2 = posx(353.08, -139.05, 96.88, 120.36, -179.20, 128.09)
place_pose_up_2 = posx(547.15, -58.71, 94.33, 111.58, -178.88, 119.12)

# ===============================
# 3번 기어
# ===============================
pick_pose_up_3 = posx(399.49, -47.40, 92.65, 129.46, -179.30, 136.87)
place_pose_up_3 = posx(633.83, -4.17, 94.30, 111.65, -178.89, 119.09)

# ===============================
# 마지막 기어
# ===============================
pick_pose_up_final = posx(348.56, -79.14, 92.64, 121.22, -179.42, 128.61)
carry_pose_up_final = posx(572.75, -5.75, 94.49, 97.77, -178.84, 105.41)

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

    set_digital_output(2, OFF)
    set_digital_output(1, ON)
    wait(1.0)

    gripper_status = 0

def close_gripper():
    global gripper_status

    print("그리퍼 닫기")

    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    wait(1.0)

    gripper_status = 1

def gripper_init():
    print("초기 위치 이동")

    movel(start_pose, VELOCITY, ACC)

    open_gripper()

def move_down(pose_up):
    pose_down = copy.deepcopy(pose_up)
    pose_down[2] -= DOWN_DISTANCE

    movel(pose_down, VELOCITY, ACC)

    return pose_down

def move_up(pose_up):
    movel(pose_up, VELOCITY, ACC)

def pick_at(pose_up):
    print("pick 위치 이동")

    movel(pose_up, VELOCITY, ACC)
    move_down(pose_up)

    close_gripper()
    move_up(pose_up)

    return True

def place_at(pose_up):
    print("place 위치 이동")

    movel(pose_up, VELOCITY, ACC)
    move_down(pose_up)

    open_gripper()
    move_up(pose_up)

    return True

def move_one_gear(pick_pose_up, place_pose_up, gear_name):
    print(f"{gear_name} 이동 시작")

    pick_success = pick_at(pick_pose_up)

    if pick_success == False:
        print(f"{gear_name} Pick failed.")
        return False

    place_success = place_at(place_pose_up)

    if place_success == False:
        print(f"{gear_name} Place failed.")
        return False

    print(f"{gear_name} 이동 완료")
    return True





def start_force_control():
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
        move_up(place_pose_up)
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

def pick_and_force_place_final(pick_pose_up, place_pose_up):
    print("마지막 기어 힘제어 Place 시작")

    pick_success = pick_at(pick_pose_up)

    if pick_success == False:
        print("마지막 기어 Pick failed.")
        return False

    place_success = force_place_final(place_pose_up)

    if place_success == False:
        print("마지막 기어 Force Place failed.")
        return False

    print("마지막 기어 힘제어 Place 완료")
    return True

def start():
    print("시작")
    gripper_init()

    success_1 = move_one_gear(
        pick_pose_up_1,
        place_pose_up_1,
        "1번 기어"
    )

    if success_1 == False:
        return

    success_2 = move_one_gear(
        pick_pose_up_2,
        place_pose_up_2,
        "2번 기어"
    )

    if success_2 == False:
        return

    success_3 = move_one_gear(
        pick_pose_up_3,
        place_pose_up_3,
        "3번 기어"
    )

    if success_3 == False:
        return

    success_final = pick_and_force_place_final(
        pick_pose_up_final,
        carry_pose_up_final
    )

    if success_final == False:
        return

    print("전체 동작 완료")

def main():
    print("로봇 상태 데이터를 수신하기 위해 대기합니다...")

    for _ in range(20):
        rclpy.spin_once(move_node, timeout_sec=0.1)

    try:
        start()
    except KeyboardInterrupt:
        print("강제 종료 시그널 감지!")
        try:
            stop_force_control()
        except Exception:
            pass

    finally:
        print("ROS 2 노드를 안전하게 종료합니다.")
        try:
            move_node.destroy_node()
            force_node.destroy_node()
        except Exception:
            pass

        rclpy.shutdown()

if __name__ == "__main__":
    main()