#!/usr/bin/env python3

import sys
import time
import termios
import tty
import select

import rclpy
import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

LINEAR_SPEED = 200.0
ACC = 500.0
KEY_TIMEOUT = 0.03
SPEED_TIME = 0.05

HOME_VEL = 30.0
HOME_ACC = 30.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init(args=sys.argv)

node = rclpy.create_node(
    "keyboard_speed_move_node",
    namespace=ROBOT_ID
)

DR_init.__dsr__node = node

import DSR_ROBOT2 as dsr


def get_key():
    if select.select([sys.stdin], [], [], 0.02)[0]:
        return sys.stdin.read(1)
    return None


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
    try:
        dsr.speedl(
            [0, 0, 0, 0, 0, 0],
            [ACC, ACC],
            t=SPEED_TIME
        )
    except Exception:
        pass


def move_speed(key):
    vx = 0.0
    vy = 0.0
    vz = 0.0

    if key == "i":
        vy = LINEAR_SPEED
    elif key == "k":
        vy = -LINEAR_SPEED
    elif key == "j":
        vx = -LINEAR_SPEED
    elif key == "l":
        vx = LINEAR_SPEED
    elif key == "w":
        vz = LINEAR_SPEED
    elif key == "s":
        vz = -LINEAR_SPEED
    else:
        return False

    dsr.speedl(
        [vx, vy, vz, 0, 0, 0],
        [ACC, ACC],
        t=SPEED_TIME
    )

    return True


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
    old_settings = termios.tcgetattr(sys.stdin)

    print("================================")
    print("키보드 조그 모드")
    print("i/k/j/l/w/s : 조그 이동")
    print("1 : 그리퍼 열기")
    print("2 : 그리퍼 닫기")
    print("3 : 현재 TCP 위치 출력")
    print("r : Home Joint")
    print("q : 종료")
    print("================================")

    last_key_time = 0.0
    moving = False

    try:
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():
            key = get_key()
            now = time.time()

            if key == "q":
                print("종료")
                break

            if key == "r":
                if moving:
                    print("이동 중에는 Home Joint 이동 불가. 정지 후 다시 누르세요.")
                    continue

                move_home_joint()
                continue

            if key == "1":
                stop_jog()
                moving = False
                open_gripper()
                continue

            if key == "2":
                stop_jog()
                moving = False
                close_gripper()
                continue

            if key == "3":
                stop_jog()
                moving = False
                print_current_position()
                continue

            if key in ["i", "k", "j", "l", "w", "s"]:
                try:
                    move_speed(key)
                    last_key_time = now
                    moving = True
                except Exception as e:
                    print(f"speedl 오류: {e}")
                    stop_jog()
                    moving = False

            if moving and now - last_key_time > KEY_TIMEOUT:
                stop_jog()
                moving = False

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("강제 종료")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        stop_jog()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()