#!/usr/bin/env python3
"""Doosan collision recovery controller used by the web UI.

This is the ROS/robot portion of the original Tkinter program.  The widget
state is exposed as JSON by web_server_node.py instead of opening a second
desktop window.
"""
import threading
import time

from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool

from DSR_ROBOT2 import get_current_posj
from dsr_msgs2.action import MovejH2r
from dsr_msgs2.srv import (
    GetRobotState,
    SetRobotControl,
    SetRobotMode,
    SetSafetyMode,
)


ROBOT_ID = "dsr01"
H2R_JOG_VEL = 5.0
H2R_JOG_ACC = 10.0
H2R_JOINT_LIMITS = [
    (-360.0, 360.0), (-95.0, 95.0), (-135.0, 135.0),
    (-360.0, 360.0), (-135.0, 135.0), (-360.0, 360.0),
]

STATE_STANDBY = 1
STATE_RECOVERY = 8
STATE_NAMES = {
    0: "INITIALIZING", 1: "STANDBY", 2: "MOVING", 3: "SAFE_OFF",
    4: "TEACHING", 5: "SAFE_STOP", 6: "EMERGENCY_STOP", 7: "HOMMING",
    8: "RECOVERY", 9: "SAFE_STOP2", 10: "SAFE_OFF2",
}

CONTROL_RESET_RECOVERY = 7
SAFETY_MODE_RECOVERY = 2
SAFETY_MODE_EVENT_ENTER = 0
SAFETY_MODE_EVENT_MOVE = 1
SAFETY_MODE_EVENT_STOP = 2
ROBOT_MODE_AUTONOMOUS = 1

SERVICE_TIMEOUT_SEC = 15.0
RECOVERY_WAIT_TIMEOUT_SEC = 90.0
STANDBY_WAIT_TIMEOUT_SEC = 15.0
ACTION_RESPONSE_TIMEOUT_SEC = 3.0
ACTION_CANCEL_TIMEOUT_SEC = 3.0


class CollisionRecoveryNode(Node):
    """Collision subscriber and hold-to-jog controller for the web overlay."""

    def __init__(self):
        super().__init__("collision_recovery_jog_ui", namespace=ROBOT_ID)

        self.cli_state = self.create_client(GetRobotState, "system/get_robot_state")
        self.cli_robot_control = self.create_client(
            SetRobotControl, "system/set_robot_control")
        self.cli_safety_mode = self.create_client(
            SetSafetyMode, "system/set_safety_mode")
        self.cli_robot_mode = self.create_client(SetRobotMode, "system/set_robot_mode")
        self.h2r_action_client = ActionClient(self, MovejH2r, "motion/movej_h2r")

        self.shutdown_event = threading.Event()
        self.collision_triggered = threading.Event()
        self.recovery_wait_started = threading.Event()
        self.recovery_ready = threading.Event()
        self.jog_hold_event = threading.Event()
        self.jog_running_event = threading.Event()

        self.service_lock = threading.Lock()
        self.current_goal_lock = threading.Lock()
        self.current_goal = None
        self.jog_heartbeat_lock = threading.Lock()
        self.jog_heartbeat_at = 0.0
        self.active_jog = None
        self.ui_lock = threading.Lock()
        self.ui = {
            "active": False,
            "ready": False,
            "closing": False,
            "robot_state": None,
            "state_name": "WAITING",
            "status": "/collision 토픽 대기 중",
            "joints": [0.0] * 6,
        }

        self.collision_subscription = self.create_subscription(
            Bool, "/collision", self._collision_callback, 10)
        self.get_logger().info("/collision 토픽 대기 중 - 웹 Recovery UI 숨김 상태")

    def snapshot(self):
        with self.ui_lock:
            data = dict(self.ui)
            data["joints"] = list(self.ui["joints"])
            return data

    def _update_ui(self, **values):
        with self.ui_lock:
            self.ui.update(values)

    def _wait_future(self, future, timeout_sec, label):
        start = time.monotonic()
        while not self.shutdown_event.is_set() and not future.done():
            if time.monotonic() - start >= timeout_sec:
                self.get_logger().error(f"[{label}] 응답 시간 초과")
                return None
            time.sleep(0.01)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f"[{label}] Future 오류: {exc}")
            return None

    def _call_service(self, client, request, timeout_sec, label):
        with self.service_lock:
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(f"[{label}] 서비스 없음: {client.srv_name}")
                return None
            try:
                return self._wait_future(
                    client.call_async(request), timeout_sec, label)
            except Exception as exc:
                self.get_logger().error(f"[{label}] 요청 오류: {exc}")
                return None

    def _get_robot_state(self):
        response = self._call_service(
            self.cli_state, GetRobotState.Request(), 2.0, "GET STATE")
        if response is None or not response.success:
            return None
        return int(response.robot_state)

    def _set_robot_control(self, value, label):
        request = SetRobotControl.Request()
        request.robot_control = int(value)
        response = self._call_service(
            self.cli_robot_control, request, SERVICE_TIMEOUT_SEC, label)
        return response is not None and bool(response.success)

    def _set_safety_event(self, event, event_name):
        request = SetSafetyMode.Request()
        request.safety_mode = SAFETY_MODE_RECOVERY
        request.safety_event = int(event)
        self.get_logger().warn(
            f"[RECOVERY {event_name}] safety_mode={SAFETY_MODE_RECOVERY}, event={event}")
        response = self._call_service(
            self.cli_safety_mode, request, SERVICE_TIMEOUT_SEC,
            f"RECOVERY {event_name}")
        return response is not None and bool(response.success)

    def _set_robot_mode_one(self):
        request = SetRobotMode.Request()
        request.robot_mode = ROBOT_MODE_AUTONOMOUS
        response = self._call_service(
            self.cli_robot_mode, request, SERVICE_TIMEOUT_SEC, "ROBOT MODE 1")
        return response is not None and bool(response.success)

    def _set_state_ui(self, state):
        self._update_ui(
            robot_state=state,
            state_name=STATE_NAMES.get(state, "UNKNOWN"),
        )

    def _wait_for_state(self, target_state, timeout_sec):
        start = time.monotonic()
        previous = object()
        while (not self.shutdown_event.is_set()
               and time.monotonic() - start < timeout_sec):
            state = self._get_robot_state()
            if state != previous and state is not None:
                self.get_logger().info(
                    f"robot_state={state} ({STATE_NAMES.get(state, 'UNKNOWN')})")
                self._set_state_ui(state)
                previous = state
            if state == target_state:
                return True
            time.sleep(0.1)
        return False

    def _read_current_posj(self):
        raw = get_current_posj()
        values = raw
        if isinstance(raw, tuple) and raw:
            try:
                if len(raw[0]) >= 6:
                    values = raw[0]
            except TypeError:
                pass
        result = [float(values[index]) for index in range(6)]
        self._update_ui(joints=result)
        return result

    def _feedback_callback(self, feedback_msg):
        try:
            values = [float(value) for value in feedback_msg.feedback.pos]
            self._update_ui(joints=values)
        except Exception as exc:
            self.get_logger().warn(f"H2R feedback 오류: {exc}")

    def _send_h2r_goal(self, target_values):
        if not self.h2r_action_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("MovejH2r Action Server가 없습니다.")
        goal = MovejH2r.Goal()
        goal.target_pos = [float(value) for value in target_values]
        goal.target_vel = [H2R_JOG_VEL] * 6
        goal.target_acc = [H2R_JOG_ACC] * 6
        future = self.h2r_action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        handle = self._wait_future(
            future, ACTION_RESPONSE_TIMEOUT_SEC, "MOVEJ H2R GOAL")
        if handle is None or not handle.accepted:
            raise RuntimeError("MovejH2r Goal이 거부되었거나 응답이 없습니다.")
        with self.current_goal_lock:
            self.current_goal = handle
        return handle

    def _cancel_current_goal(self):
        with self.current_goal_lock:
            handle = self.current_goal
            self.current_goal = None
        if handle is not None:
            future = handle.cancel_goal_async()
            self._wait_future(future, ACTION_CANCEL_TIMEOUT_SEC, "MOVEJ H2R CANCEL")

    def _jog_worker(self, joint_index, direction):
        joint_name = f"J{joint_index + 1}"
        sign = "+" if direction > 0 else "−"
        self._update_ui(status=f"{joint_name} {sign} 연속 이동 중")
        try:
            state = self._get_robot_state()
            if state != STATE_RECOVERY:
                raise RuntimeError(f"RECOVERY(8) 상태가 아닙니다. 현재={state}")
            current = self._read_current_posj()
            joint_min, joint_max = H2R_JOINT_LIMITS[joint_index]
            target = list(current)
            target[joint_index] = joint_max if direction > 0 else joint_min
            if not self._set_safety_event(SAFETY_MODE_EVENT_MOVE, "MOVE"):
                raise RuntimeError("RECOVERY MOVE Event 실패")
            handle = self._send_h2r_goal(target)
            result_future = handle.get_result_async()
            while not self.shutdown_event.is_set() and self.jog_hold_event.is_set():
                with self.jog_heartbeat_lock:
                    heartbeat_age = time.monotonic() - self.jog_heartbeat_at
                if heartbeat_age > 0.6:
                    self._update_ui(status="조그 입력 연결 끊김 - 안전 정지")
                    self.jog_hold_event.clear()
                    break
                if result_future.done():
                    wrapped = result_future.result()
                    with self.current_goal_lock:
                        if self.current_goal is handle:
                            self.current_goal = None
                    if wrapped is None or not wrapped.result.success:
                        raise RuntimeError("MovejH2r 이동 실패")
                    self._update_ui(status=f"{joint_name} 관절 제한 도달")
                    break
                time.sleep(0.005)
            if not self.jog_hold_event.is_set() or self.shutdown_event.is_set():
                self._cancel_current_goal()
        except Exception as exc:
            self.get_logger().error(f"H2R Jog 오류: {exc}")
            self._update_ui(status=f"H2R Jog 오류: {exc}")
        finally:
            self.jog_hold_event.clear()
            try:
                self._cancel_current_goal()
                self._set_safety_event(SAFETY_MODE_EVENT_STOP, "STOP")
            except Exception as exc:
                self.get_logger().warn(f"RECOVERY STOP 오류: {exc}")
            self.jog_running_event.clear()
            with self.jog_heartbeat_lock:
                self.active_jog = None
            try:
                self._read_current_posj()
            except Exception:
                pass
            if not self.shutdown_event.is_set() and self.recovery_ready.is_set():
                self._update_ui(status="RECOVERY READY - 버튼을 누르면 이동")

    def start_jog(self, joint_index, direction):
        joint_index, direction = int(joint_index), int(direction)
        if joint_index not in range(6) or direction not in (-1, 1):
            raise ValueError("joint는 0~5, direction은 -1 또는 1이어야 합니다.")
        if not self.recovery_ready.is_set():
            self._update_ui(status="Recovery 진입이 아직 완료되지 않았습니다.")
            return False
        with self.jog_heartbeat_lock:
            if self.jog_running_event.is_set():
                if self.active_jog != (joint_index, direction):
                    return False
                # 같은 버튼을 누르는 동안 웹 클라이언트가 보내는 heartbeat.
                self.jog_heartbeat_at = time.monotonic()
                return True
            self.active_jog = (joint_index, direction)
            self.jog_heartbeat_at = time.monotonic()
            self.jog_running_event.set()
        self.jog_hold_event.set()
        threading.Thread(
            target=self._jog_worker, args=(joint_index, direction), daemon=True
        ).start()
        return True

    def stop_jog(self):
        self.jog_hold_event.clear()
        return True

    def _recovery_wait_worker(self):
        self._update_ui(
            active=True, ready=False, closing=False,
            status="충돌 감지 - Recovery 진입 대기 중...",
        )
        if not self._wait_for_state(STATE_RECOVERY, RECOVERY_WAIT_TIMEOUT_SEC):
            state = self._get_robot_state()
            self._update_ui(
                status=(f"Recovery 진입 확인 실패: {state} "
                        f"({STATE_NAMES.get(state, 'UNKNOWN')})"))
            return
        self._set_safety_event(SAFETY_MODE_EVENT_ENTER, "ENTER")
        self.recovery_ready.set()
        try:
            self._read_current_posj()
        except Exception as exc:
            self.get_logger().warn(f"초기 관절 위치 조회 실패: {exc}")
        self._set_state_ui(STATE_RECOVERY)
        self._update_ui(
            ready=True, status="RECOVERY READY - 버튼을 누르면 이동")
        self.get_logger().warn("RECOVERY(8) 확인 - 웹 H2R 조그 활성화")

    def _collision_callback(self, message):
        if not message.data or self.collision_triggered.is_set():
            return
        self.collision_triggered.set()
        self.get_logger().error("/collision=true 수신")
        if not self.recovery_wait_started.is_set():
            self.recovery_wait_started.set()
            threading.Thread(target=self._recovery_wait_worker, daemon=True).start()

    def close_recovery(self):
        snapshot = self.snapshot()
        if snapshot["closing"]:
            return False
        self._update_ui(
            closing=True, ready=False, status="조그 정지 및 Mode 1 복귀 처리 중...")
        self.jog_hold_event.clear()

        def worker():
            try:
                self._cancel_current_goal()
                state = self._get_robot_state()
                if state == STATE_RECOVERY:
                    try:
                        self._set_safety_event(SAFETY_MODE_EVENT_STOP, "STOP")
                    except Exception:
                        pass
                    time.sleep(0.2)
                    self._set_robot_control(CONTROL_RESET_RECOVERY, "RESET RECOVERY")
                    self._wait_for_state(STATE_STANDBY, STANDBY_WAIT_TIMEOUT_SEC)
                if not self._set_robot_mode_one():
                    raise RuntimeError("Mode 1(AUTONOMOUS) 복귀 요청 실패")
                self._update_ui(
                    active=False, closing=False, robot_state=STATE_STANDBY,
                    state_name=STATE_NAMES[STATE_STANDBY],
                    status="/collision 토픽 대기 중", ready=False)
                self.recovery_ready.clear()
                self.recovery_wait_started.clear()
                self.collision_triggered.clear()
            except Exception as exc:
                self.get_logger().error(f"복귀 처리 오류: {exc}")
                self._update_ui(
                    closing=False, ready=self.recovery_ready.is_set(),
                    status=f"복귀 처리 오류: {exc}")

        threading.Thread(target=worker, daemon=True).start()
        return True

    def shutdown_controller(self):
        self.shutdown_event.set()
        self.jog_hold_event.clear()
        try:
            self._cancel_current_goal()
        except Exception:
            pass
