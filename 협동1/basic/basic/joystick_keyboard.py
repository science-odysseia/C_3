#!/usr/bin/env python3

import sys
import time

import rclpy
import DR_init

import keyboard


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

LINEAR_SPEED = 200.0
ACC = 500.0
LOOP_DT = 0.01
SPEED_TIME = 0.03

HOME_VEL = 30.0
HOME_ACC = 30.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init(args=sys.argv)

node = rclpy.create_node(
    "keyboard_module_joystick_node",
    namespace=ROBOT_ID
)

DR_init.__dsr__node = node

import DSR_ROBOT2 as dsr


def open_gripper():
    print("그리퍼 열기")
    dsr.set_digital_output(2, dsr.OFF)
    dsr.set_digital_output(1, dsr.ON)


def close_gripper():
    print("그리퍼 닫기")
    dsr.set_digital_output(1, dsr.OFF)
    dsr.set_digital_output(2, dsr.ON)


def print_current_position():
    pos, sol = dsr.get_current_posx()

    print("--------------------------------")
    print("현재 TCP 위치")
    print(
        f"X:{pos[0]:.2f}, "
        f"Y:{pos[1]:.2f}, "
        f"Z:{pos[2]:.2f}, "
        f"RX:{pos[3]:.2f}, "
        f"RY:{pos[4]:.2f}, "
        f"RZ:{pos[5]:.2f}"
    )
    print(f"Solution: {sol}")


def stop_jog():
    dsr.speedl(
        [0, 0, 0, 0, 0, 0],
        [ACC, ACC],
        t=SPEED_TIME
    )


def send_speed(vx, vy, vz):
    dsr.speedl(
        [vx, vy, vz, 0, 0, 0],
        [ACC, ACC],
        t=SPEED_TIME
    )


def move_home_joint():
    print("Home Joint 이동 시작")

    try:
        ret = dsr.movej(
            dsr.posj(0, 0, 90, 0, 90, 0),
            vel=HOME_VEL,
            acc=HOME_ACC
        )
        print(f"Home Joint 이동 완료, return: {ret}")

    except Exception as e:
        print(f"Home Joint 이동 오류: {e}")


def main():
    print("================================")
    print("keyboard 모듈 조이스틱 모드")
    print("i/k/j/l/w/s : 누르고 있는 동안 조그 이동")
    print("1 : 그리퍼 열기")
    print("2 : 그리퍼 닫기")
    print("3 : 현재 TCP 위치 출력")
    print("r : Home Joint")
    print("q : 종료")
    print("================================")

    was_moving = False
    prev_cmd = [0.0, 0.0, 0.0]

    key_1_pressed = False
    key_2_pressed = False
    key_3_pressed = False
    key_r_pressed = False

    try:
        while rclpy.ok():
            if keyboard.is_pressed("q"):
                print("종료")
                break

            vx = 0.0
            vy = 0.0
            vz = 0.0

            if keyboard.is_pressed("i"):
                vy += LINEAR_SPEED
            if keyboard.is_pressed("k"):
                vy -= LINEAR_SPEED
            if keyboard.is_pressed("j"):
                vx -= LINEAR_SPEED
            if keyboard.is_pressed("l"):
                vx += LINEAR_SPEED
            if keyboard.is_pressed("w"):
                vz += LINEAR_SPEED
            if keyboard.is_pressed("s"):
                vz -= LINEAR_SPEED

            moving = (vx != 0.0 or vy != 0.0 or vz != 0.0)

            if keyboard.is_pressed("1"):
                if not key_1_pressed:
                    stop_jog()
                    was_moving = False
                    open_gripper()
                key_1_pressed = True
            else:
                key_1_pressed = False

            if keyboard.is_pressed("2"):
                if not key_2_pressed:
                    stop_jog()
                    was_moving = False
                    close_gripper()
                key_2_pressed = True
            else:
                key_2_pressed = False

            if keyboard.is_pressed("3"):
                if not key_3_pressed:
                    stop_jog()
                    was_moving = False
                    print_current_position()
                key_3_pressed = True
            else:
                key_3_pressed = False

            if keyboard.is_pressed("r"):
                if not key_r_pressed:
                    if moving or was_moving:
                        stop_jog()
                        was_moving = False

                    move_home_joint()

                key_r_pressed = True
            else:
                key_r_pressed = False

            if moving:
                cmd = [vx, vy, vz]

                if cmd != prev_cmd or not was_moving:
                    send_speed(vx, vy, vz)
                    prev_cmd = cmd

                was_moving = True

            else:
                if was_moving:
                    stop_jog()
                    prev_cmd = [0.0, 0.0, 0.0]
                    was_moving = False

            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("강제 종료")

    finally:
        try:
            stop_jog()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()