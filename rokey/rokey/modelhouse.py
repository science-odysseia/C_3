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

def move_down(pose_up):
    pose_down = copy.deepcopy(pose_up)
    pose_down[2] -= DOWN_DISTANCE

    movel(pose_down, VELOCITY, ACC)

    return pose_down

def move_up(pose_up):
    movel(pose_up, VELOCITY, ACC)

def place_at(pose_up):
    print("place 위치 이동")

    movel(pose_up, VELOCITY, ACC)
    move_down(pose_up)

    open_gripper()
    move_up(pose_up)

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


