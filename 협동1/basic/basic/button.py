#!/usr/bin/env python3

import sys
import copy
import termios
import tty
import select
import threading

import rclpy
import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY = 30
ACC = 30
STEP = 10.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init(args=sys.argv)

node = rclpy.create_node(
    "keyboard_xyz_move_node",
    namespace=ROBOT_ID
)

DR_init.__dsr__node = node

from DSR_ROBOT2 import *


moving_lock = threading.Lock()


def get_key():
    if select.select([sys.stdin], [], [], 0.1)[0]:
        return sys.stdin.read(1)
    return None


def move_xyz(dx=0.0, dy=0.0, dz=0.0):
    if not moving_lock.acquire(blocking=False):
        print("이전 이동 중이라 입력 무시")
        return

    try:
        cur = get_current_posx()[0]

        target = copy.deepcopy(cur)
        target[0] += dx
        target[1] += dy
        target[2] += dz

        print("--------------------------------")
        print(f"현재 X:{cur[0]:.2f}, Y:{cur[1]:.2f}, Z:{cur[2]:.2f}")
        print(f"목표 X:{target[0]:.2f}, Y:{target[1]:.2f}, Z:{target[2]:.2f}")

        movel(
            target,
            vel=VELOCITY,
            acc=ACC
        )

    except Exception as e:
        print(f"이동 중 오류: {e}")

    finally:
        moving_lock.release()


def main():
    print("================================")
    print("디지털 입력 TCP 이동 노드 시작")
    print("DI13 : +X 10 mm")
    print("DI14 : -X 10 mm")
    print("DI15 : +Y 10 mm")
    print("DI16 : -Y 10 mm")
    print("Ctrl+C : 종료")
    print("================================")

    try:
        while rclpy.ok():

            if get_digital_input(13):
                move_xyz(dx=STEP)
                while get_digital_input(13):
                    wait(0.01)

            elif get_digital_input(14):
                move_xyz(dx=-STEP)
                while get_digital_input(14):
                    wait(0.01)

            elif get_digital_input(15):
                move_xyz(dy=STEP)
                while get_digital_input(15):
                    wait(0.01)

            elif get_digital_input(16):
                move_xyz(dy=-STEP)
                while get_digital_input(16):
                    wait(0.01)

            wait(0.01)

    except KeyboardInterrupt:
        print("강제 종료")

    finally:
        try:
            drl_script_stop(DR_SSTOP)
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()