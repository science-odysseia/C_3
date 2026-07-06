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
    old_settings = termios.tcgetattr(sys.stdin)

    print("================================")
    print("키보드 TCP 이동 노드 시작")
    print("i : +Y 10 mm")
    print("k : -Y 10 mm")
    print("j : -X 10 mm")
    print("l : +X 10 mm")
    print("r : +Z 10 mm")
    print("f : -Z 10 mm")
    print("q : 종료")
    print("================================")

    try:
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():
            key = get_key()

            if key is None:
                continue

            if key == "q":
                print("종료 키 입력")
                break

            elif key == "i":
                move_xyz(dy=STEP)

            elif key == "k":
                move_xyz(dy=-STEP)

            elif key == "j":
                move_xyz(dx=-STEP)

            elif key == "l":
                move_xyz(dx=STEP)

            elif key == "r":
                move_xyz(dz=STEP)

            elif key == "f":
                move_xyz(dz=-STEP)

    except KeyboardInterrupt:
        print("강제 종료")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        try:
            drl_script_stop(DR_SSTOP)
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()