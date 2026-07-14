import rclpy
import DR_init
import json
import math
import threading
import time

from copy import deepcopy
from std_msgs.msg import String, Bool
from rclpy.executors import SingleThreadedExecutor

# =========================================================================
# [1. 로봇 및 환경 설정 환경 상수]
# =========================================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 600, 600
HOME_VEL, HOME_ACC = 60, 60   # 🔧 홈 이동(movej) 전용 속도/가속도 [deg/s]

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

gripper_status = 0
USER_COORD = 102

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)
    #현재 배치할 블록 발신할 퍼블리셔 생성
    pub_block = node.create_publisher(String, '/current_block_status', 10)
    DR_init.__dsr__node = node

    # =========================================================================
    # [2. API 임포트]
    # =========================================================================
    try:
        from DSR_ROBOT2 import (
            movej as _dsr_movej,
            movel as _dsr_movel,
            movejx as _dsr_movejx,
            set_digital_output as _dsr_set_digital_output,
            task_compliance_ctrl as _dsr_task_compliance_ctrl,
            set_desired_force as _dsr_set_desired_force,
            release_force as _dsr_release_force,
            release_compliance_ctrl as _dsr_release_compliance_ctrl,
            move_periodic as _dsr_move_periodic,
            get_current_posx as _dsr_get_current_posx,
            OFF, ON, DR_FC_MOD_REL, set_ref_coord
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    # =========================================================================
    # [2-1. 독립 충돌 감시 및 Software Recovery 시스템]
    # =========================================================================
    from dsr_msgs2.srv import (
        GetRobotState,
        GetRobotMode,
        MoveStop,
        ReleaseForce,
        ReleaseComplianceCtrl,
        SetRobotControl,
        SetSafetyMode,
    )
    from dsr_msgs2.msg import LogAlarm

    STATE_STANDBY = 1
    STATE_SAFE_OFF = 3
    STATE_SAFE_STOP = 5
    STATE_RECOVERY = 8
    STATE_SAFE_STOP2 = 9
    STATE_SAFE_OFF2 = 10

    STATE_NAMES = {
        0: "INITIALIZING",
        1: "STANDBY",
        2: "MOVING",
        3: "SAFE_OFF",
        4: "TEACHING",
        5: "SAFE_STOP",
        6: "EMERGENCY_STOP",
        7: "HOMMING",
        8: "RECOVERY",
        9: "SAFE_STOP2",
        10: "SAFE_OFF2",
    }

    COLLISION_STATES = {
        STATE_SAFE_OFF,
        STATE_SAFE_STOP,
        STATE_SAFE_STOP2,
        STATE_SAFE_OFF2,
    }

    CONTROL_RESET_SAFE_STOP = 2
    CONTROL_RESET_SAFE_OFF = 3
    CONTROL_RECOVERY_SAFE_STOP = 4
    CONTROL_RECOVERY_SAFE_OFF = 5

    SAFETY_MODE_RECOVERY = 2
    SAFETY_MODE_EVENT_ENTER = 0

    collision_detected = threading.Event()
    collision_alarm_detected = threading.Event()
    interrupt_detected = threading.Event()
    interrupt_finished = threading.Event()
    interrupt_request_lock = threading.Lock()
    interrupt_request_futures = []
    recovery_started = threading.Event()
    recovery_entered = threading.Event()
    recovery_finished = threading.Event()
    safety_monitor_shutdown = threading.Event()

    class CollisionAbort(RuntimeError):
        """충돌 감지 후 기존 조립 흐름을 즉시 중단하기 위한 예외."""

    class InterruptAbort(RuntimeError):
        """외부 /interrupt=true로 기존 조립 흐름을 중단하는 예외."""

    safety_node = rclpy.create_node(
        "assembly_collision_recovery_monitor",
        namespace=ROBOT_ID,
    )
    safety_executor = SingleThreadedExecutor()
    safety_executor.add_node(safety_node)

    # 충돌 상태를 다른 ROS 노드에 전달하는 토픽
    collision_publisher = safety_node.create_publisher(
        Bool,
        "/collision",
        10,
    )

    def publish_collision_state(detected):
        message = Bool()
        message.data = bool(detected)
        collision_publisher.publish(message)

    # 프로그램 동작 중 /collision 상태를 20Hz로 계속 발행한다.
    collision_publish_timer = safety_node.create_timer(
        0.05,
        lambda: publish_collision_state(
            collision_detected.is_set()
        ),
    )

    # 인위적 중단이 실제로 끝난 뒤 웹 수동 조작을 허용하는 신호.
    # 실제 충돌 감지 및 /collision 처리와는 완전히 분리한다.
    interrupt_ready_publisher = safety_node.create_publisher(
        Bool,
        "/interrupt_ready",
        10,
    )

    def publish_interrupt_ready(ready):
        message = Bool()
        message.data = bool(ready)
        interrupt_ready_publisher.publish(message)

    publish_interrupt_ready(False)

    def robot_error_callback(message):
        try:
            alarm_index = int(message.index)
        except Exception:
            return

        # 두산 컨트롤러 collision violation 알람
        if alarm_index == 7060:
            collision_alarm_detected.set()
            collision_detected.set()
            publish_collision_state(True)

    collision_error_subscription = safety_node.create_subscription(
        LogAlarm,
        "error",
        robot_error_callback,
        10,
    )

    cli_robot_state = safety_node.create_client(
        GetRobotState,
        "system/get_robot_state",
    )
    cli_robot_mode = safety_node.create_client(
        GetRobotMode,
        "system/get_robot_mode",
    )
    cli_move_stop = safety_node.create_client(
        MoveStop,
        "motion/move_stop",
    )
    cli_release_force = safety_node.create_client(
        ReleaseForce,
        "force/release_force",
    )
    cli_release_compliance = safety_node.create_client(
        ReleaseComplianceCtrl,
        "force/release_compliance_ctrl",
    )
    cli_robot_control = safety_node.create_client(
        SetRobotControl,
        "system/set_robot_control",
    )
    cli_safety_mode = safety_node.create_client(
        SetSafetyMode,
        "system/set_safety_mode",
    )

    def interrupt_callback(message):
        if not message.data:
            publish_interrupt_ready(False)
            return

        if interrupt_detected.is_set():
            return

        publish_interrupt_ready(False)
        interrupt_detected.set()

        safety_node.get_logger().error(
            "========================================"
        )
        safety_node.get_logger().error(
            "/interrupt=true 수신 - 모든 공정 명령 즉시 차단"
        )
        safety_node.get_logger().error(
            "Recovery에는 진입하지 않고 현재 작업을 종료합니다."
        )
        safety_node.get_logger().error(
            "========================================"
        )

        # 콜백 안에서는 기다리지 않고 정지 요청을 즉시 전송한다.
        try:
            stop_request = MoveStop.Request()
            # 인위적 중단만 Category 1 Soft Stop으로 처리한다.
            # 충돌 감지 경로의 DR_QSTOP은 변경하지 않는다.
            stop_request.stop_mode = 2
            future = cli_move_stop.call_async(stop_request)
            with interrupt_request_lock:
                interrupt_request_futures.append(
                    ("MoveStop(SSTO)", future, True)
                )
        except Exception as exc:
            safety_node.get_logger().error(
                f"[INTERRUPT] MoveStop 전송 오류: {exc}"
            )

        try:
            force_request = ReleaseForce.Request()
            force_request.time = 0.0
            future = cli_release_force.call_async(force_request)
            with interrupt_request_lock:
                interrupt_request_futures.append(
                    ("ReleaseForce", future, False)
                )
        except Exception as exc:
            safety_node.get_logger().error(
                f"[INTERRUPT] ReleaseForce 전송 오류: {exc}"
            )

        try:
            future = cli_release_compliance.call_async(
                ReleaseComplianceCtrl.Request()
            )
            with interrupt_request_lock:
                interrupt_request_futures.append(
                    ("ReleaseCompliance", future, False)
                )
        except Exception as exc:
            safety_node.get_logger().error(
                f"[INTERRUPT] ReleaseCompliance 전송 오류: {exc}"
            )

    interrupt_subscription = safety_node.create_subscription(
        Bool,
        "/interrupt",
        interrupt_callback,
        10,
    )

    def safety_call(client, request, timeout_sec, label):
        if not client.wait_for_service(timeout_sec=1.0):
            safety_node.get_logger().error(
                f"[{label}] 서비스 없음: {client.srv_name}"
            )
            return None

        future = client.call_async(request)
        start = time.monotonic()

        while rclpy.ok() and not future.done():
            if time.monotonic() - start >= timeout_sec:
                try:
                    client.remove_pending_request(future)
                except Exception:
                    pass
                safety_node.get_logger().error(
                    f"[{label}] 응답 시간 초과"
                )
                return None

            safety_executor.spin_once(timeout_sec=0.02)

        if not future.done():
            return None

        try:
            return future.result()
        except Exception as exc:
            safety_node.get_logger().error(
                f"[{label}] 서비스 예외: {exc}"
            )
            return None

    def read_robot_state():
        response = safety_call(
            cli_robot_state,
            GetRobotState.Request(),
            1.0,
            "GET STATE",
        )
        if response is None or not response.success:
            return None
        return int(response.robot_state)

    def read_robot_mode():
        response = safety_call(
            cli_robot_mode,
            GetRobotMode.Request(),
            1.0,
            "GET ROBOT MODE",
        )
        if response is None or not response.success:
            return None
        return int(response.robot_mode)

    def wait_interrupt_request(label, future, timeout_sec=5.0):
        """인위적 중단용 비동기 서비스가 실제 완료될 때까지 기다린다."""
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - start >= timeout_sec:
                safety_node.get_logger().error(
                    f"[INTERRUPT] {label} 응답 시간 초과"
                )
                return False
            safety_executor.spin_once(timeout_sec=0.02)

        try:
            response = future.result()
        except Exception as exc:
            safety_node.get_logger().error(
                f"[INTERRUPT] {label} 서비스 예외: {exc}"
            )
            return False

        success = bool(getattr(response, "success", True))
        safety_node.get_logger().info(
            f"[INTERRUPT] {label} 완료: success={success}"
        )
        return success

    def wait_for_interrupt_standby(timeout_sec=15.0):
        """Soft Stop 이후 로봇이 STANDBY(1)가 될 때까지 기다린다."""
        start = time.monotonic()
        previous = None
        while (
            rclpy.ok()
            and not safety_monitor_shutdown.is_set()
            and time.monotonic() - start < timeout_sec
        ):
            state = read_robot_state()
            if state != previous:
                safety_node.get_logger().info(
                    f"[INTERRUPT WAIT] robot_state={state} "
                    f"({STATE_NAMES.get(state, 'UNKNOWN')})"
                )
                previous = state
            if state == STATE_STANDBY:
                return True
            time.sleep(0.1)
        return False

    def wait_for_ui_mode_one_and_clear_collision():
        """UI가 Recovery를 종료하고 Mode 1로 복귀할 때까지 대기."""
        safety_node.get_logger().warn(
            "UI의 STANDBY(1) + Robot Mode 1 복귀를 기다립니다."
        )

        previous = None

        while rclpy.ok():
            state = read_robot_state()
            robot_mode = read_robot_mode()
            current = (state, robot_mode)

            if current != previous:
                safety_node.get_logger().info(
                    f"[COLLISION RESET WAIT] "
                    f"state={state} "
                    f"({STATE_NAMES.get(state, 'UNKNOWN')}), "
                    f"mode={robot_mode}"
                )
                previous = current

            if (
                state == STATE_STANDBY
                and robot_mode == 1
            ):
                # collision_detected는 기존 공정 재개와 종료 홈 이동을
                # 막는 내부 래치이므로 유지하고, 외부 토픽만 False로 내린다.
                collision_alarm_detected.clear()

                # 구독 노드가 확실히 수신할 수 있도록 몇 차례 발행한다.
                for _ in range(3):
                    publish_collision_state(False)
                    time.sleep(0.05)

                safety_node.get_logger().warn(
                    "========================================"
                )
                safety_node.get_logger().warn(
                    "STANDBY(1) + Robot Mode 1 복귀 확인"
                )
                safety_node.get_logger().warn(
                    "/collision=false 발행 완료"
                )
                safety_node.get_logger().warn(
                    "========================================"
                )
                return

            time.sleep(0.2)

    def request_emergency_motion_stop():
        request = MoveStop.Request()
        request.stop_mode = 1  # DR_QSTOP
        response = safety_call(
            cli_move_stop,
            request,
            2.0,
            "MOVE STOP",
        )
        safety_node.get_logger().warn(
            f"[MOVE STOP] result={response}"
        )

    def request_robot_control(control_value, label):
        request = SetRobotControl.Request()
        request.robot_control = int(control_value)
        response = safety_call(
            cli_robot_control,
            request,
            15.0,
            label,
        )
        safety_node.get_logger().warn(
            f"[{label}] control={control_value}, result={response}"
        )
        return response is not None

    def request_recovery_enter_event():
        request = SetSafetyMode.Request()
        request.safety_mode = SAFETY_MODE_RECOVERY
        request.safety_event = SAFETY_MODE_EVENT_ENTER
        response = safety_call(
            cli_safety_mode,
            request,
            15.0,
            "RECOVERY ENTER",
        )
        safety_node.get_logger().warn(
            f"[RECOVERY ENTER] result={response}"
        )
        return response is not None

    def wait_for_states(target_states, timeout_sec):
        start = time.monotonic()
        previous = None

        while (
            rclpy.ok()
            and not safety_monitor_shutdown.is_set()
            and time.monotonic() - start < timeout_sec
        ):
            state = read_robot_state()

            if state is not None and state != previous:
                safety_node.get_logger().warn(
                    f"[RECOVERY] robot_state={state} "
                    f"({STATE_NAMES.get(state, 'UNKNOWN')})"
                )
                previous = state

            if state in target_states:
                return state

            time.sleep(0.1)

        return read_robot_state()

    def wait_for_readable_state(timeout_sec):
        """모션 서비스가 반환되는 동안 막혀 있던 상태 서비스 복구 대기."""
        start = time.monotonic()

        while (
            rclpy.ok()
            and not safety_monitor_shutdown.is_set()
            and time.monotonic() - start < timeout_sec
        ):
            state = read_robot_state()
            if state is not None:
                return state
            time.sleep(0.1)

        return None

    def enter_recovery_from_state2(state):
        if state == STATE_SAFE_STOP2:
            request_robot_control(
                CONTROL_RECOVERY_SAFE_STOP,
                "RECOVERY SAFE_STOP2",
            )
        elif state == STATE_SAFE_OFF2:
            request_robot_control(
                CONTROL_RECOVERY_SAFE_OFF,
                "RECOVERY SAFE_OFF2",
            )
        else:
            return False

        state = wait_for_states(
            {STATE_RECOVERY},
            15.0,
        )
        return state == STATE_RECOVERY

    def enter_software_recovery():
        # 충돌 직전 실행 중이던 동기 모션 서비스가 종료될 때까지
        # get_robot_state가 잠시 응답하지 않을 수 있으므로 기다린다.
        state = wait_for_readable_state(15.0)

        safety_node.get_logger().warn(
            "========================================"
        )
        safety_node.get_logger().warn(
            "충돌 후 Software Recovery 자동 진입 시작"
        )
        safety_node.get_logger().warn(
            f"시작 상태={state} "
            f"({STATE_NAMES.get(state, 'UNKNOWN')})"
        )
        safety_node.get_logger().warn(
            "========================================"
        )

        if state == STATE_RECOVERY:
            request_recovery_enter_event()
            return True

        # SAFE_STOP(5)은 짧은 시간 뒤 SAFE_OFF(3) 등으로 바뀔 수 있다.
        if state == STATE_SAFE_STOP:
            state = wait_for_states(
                {
                    STATE_SAFE_OFF,
                    STATE_SAFE_STOP2,
                    STATE_SAFE_OFF2,
                    STATE_RECOVERY,
                },
                3.0,
            )

        if state == STATE_RECOVERY:
            request_recovery_enter_event()
            return True

        # 힘 제어/컴플라이언스 작업 중 충돌은 SAFE_STOP(5)에 계속
        # 머무를 수 있다. 펜던트 Recovery 화면 진입 후 SAFE_STOP을
        # 리셋하고, 그 결과 상태에 맞춰 기존 Recovery 경로를 계속한다.
        if state == STATE_SAFE_STOP:
            safety_node.get_logger().warn(
                "SAFE_STOP(5) 유지 - Recovery ENTER 후 "
                "RESET_SAFE_STOP(2) 요청"
            )

            request_recovery_enter_event()
            request_robot_control(
                CONTROL_RESET_SAFE_STOP,
                "RECOVERY RESET SAFE_STOP",
            )

            state = wait_for_states(
                {
                    STATE_RECOVERY,
                    STATE_SAFE_OFF,
                    STATE_SAFE_STOP2,
                    STATE_SAFE_OFF2,
                },
                15.0,
            )

            if state == STATE_RECOVERY:
                request_recovery_enter_event()
                return True

            # 일부 펌웨어에서는 SAFE_STOP(5) 표시를 유지한 채
            # Recovery Safe Stop 명령을 처리하므로 호환성 시도한다.
            if state == STATE_SAFE_STOP:
                request_robot_control(
                    CONTROL_RECOVERY_SAFE_STOP,
                    "RECOVERY SAFE_STOP FALLBACK",
                )
                state = wait_for_states(
                    {
                        STATE_RECOVERY,
                        STATE_SAFE_OFF,
                        STATE_SAFE_STOP2,
                        STATE_SAFE_OFF2,
                    },
                    15.0,
                )

                if state == STATE_RECOVERY:
                    request_recovery_enter_event()
                    return True

        if state in (STATE_SAFE_STOP2, STATE_SAFE_OFF2):
            if not enter_recovery_from_state2(state):
                return False
            request_recovery_enter_event()
            return read_robot_state() == STATE_RECOVERY

        if state == STATE_SAFE_OFF:
            # 펜던트 Recovery 화면 진입에 해당한다.
            request_recovery_enter_event()

            transitioned = wait_for_states(
                {
                    STATE_RECOVERY,
                    STATE_SAFE_STOP2,
                    STATE_SAFE_OFF2,
                },
                2.0,
            )

            if transitioned == STATE_RECOVERY:
                return True

            if transitioned in (STATE_SAFE_STOP2, STATE_SAFE_OFF2):
                if not enter_recovery_from_state2(transitioned):
                    return False
                request_recovery_enter_event()
                return read_robot_state() == STATE_RECOVERY

            # 펜던트 Recovery 화면의 Servo On에 해당한다.
            request_robot_control(
                CONTROL_RESET_SAFE_OFF,
                "RECOVERY SERVO ON",
            )

            transitioned = wait_for_states(
                {
                    STATE_RECOVERY,
                    STATE_STANDBY,
                    STATE_SAFE_STOP2,
                    STATE_SAFE_OFF2,
                },
                15.0,
            )

            if transitioned == STATE_RECOVERY:
                return True

            if transitioned in (STATE_SAFE_STOP2, STATE_SAFE_OFF2):
                if not enter_recovery_from_state2(transitioned):
                    return False
                request_recovery_enter_event()
                return read_robot_state() == STATE_RECOVERY

            # 현재 컨트롤러에서 검증된 SAFE_OFF 호환성 경로.
            if transitioned == STATE_SAFE_OFF:
                request_robot_control(
                    CONTROL_RECOVERY_SAFE_OFF,
                    "RECOVERY SAFE_OFF FALLBACK",
                )
                transitioned = wait_for_states(
                    {STATE_RECOVERY},
                    15.0,
                )
                if transitioned == STATE_RECOVERY:
                    request_recovery_enter_event()
                    return True

        final_state = read_robot_state()
        safety_node.get_logger().error(
            f"Recovery 자동 진입 실패: {final_state} "
            f"({STATE_NAMES.get(final_state, 'UNKNOWN')})"
        )
        return False

    def collision_monitor_worker():
        try:
            safety_node.get_logger().info(
                "독립 충돌 감시 스레드 시작"
            )

            while (
                rclpy.ok()
                and not safety_monitor_shutdown.is_set()
            ):
                state = read_robot_state()

                if (
                    state in COLLISION_STATES
                    or collision_alarm_detected.is_set()
                ):
                    if recovery_started.is_set():
                        return

                    recovery_started.set()
                    collision_detected.set()
                    publish_collision_state(True)

                    safety_node.get_logger().error(
                        "========================================"
                    )
                    safety_node.get_logger().error(
                        "충돌/안전정지 감지 - 모든 공정 명령 차단"
                    )
                    safety_node.get_logger().error(
                        f"robot_state={state} "
                        f"({STATE_NAMES.get(state, 'UNKNOWN')})"
                    )
                    safety_node.get_logger().error(
                        "========================================"
                    )

                    # 각 요청은 실패하더라도 다음 정지 절차를 계속 수행한다.
                    try:
                        request_emergency_motion_stop()
                    except Exception as exc:
                        safety_node.get_logger().error(
                            f"MoveStop 요청 오류: {exc}"
                        )

                    # 충돌 안전정지 상태에서 ReleaseForce/
                    # ReleaseCompliance 서비스를 다시 호출하면 해당 서비스가
                    # 컨트롤러 내부에서 블로킹되어 GetRobotState까지 막힐 수 있다.
                    # 실제 힘/컴플라이언스는 안전 컨트롤러가 이미 정지했으며,
                    # collision_detected 래퍼가 이후 모든 힘 명령을 차단한다.
                    safety_node.get_logger().warn(
                        "안전정지 후 추가 Force Release 서비스 호출 생략"
                    )

                    if enter_software_recovery():
                        recovery_entered.set()
                        print("recov entered", flush=True)
                        safety_node.get_logger().warn(
                            "RECOVERY(8) 진입 완료"
                        )

                    recovery_finished.set()
                    return

                if interrupt_detected.is_set():
                    with interrupt_request_lock:
                        requests = list(interrupt_request_futures)

                    required_requests = [
                        (label, future)
                        for label, future, required in requests
                        if required
                    ]
                    required_ok = bool(required_requests)
                    for label, future in required_requests:
                        if not wait_interrupt_request(label, future):
                            required_ok = False

                    # Force/Compliance 해제는 보조 요청이다. 이 응답 때문에
                    # Soft Stop 상태 확인 자체가 지연되지는 않게 한다.
                    for label, future, required in requests:
                        if not required and future.done():
                            wait_interrupt_request(label, future)

                    standby = wait_for_interrupt_standby()
                    ready = required_ok and standby
                    publish_interrupt_ready(ready)

                    if ready:
                        safety_node.get_logger().warn(
                            "[INTERRUPT] Soft Stop 완료 + STANDBY(1) 확인 - "
                            "/interrupt_ready=true"
                        )
                    else:
                        safety_node.get_logger().error(
                            "[INTERRUPT] 정지 완료를 확인하지 못해 "
                            "웹 수동 조작을 잠금 상태로 유지합니다."
                        )
                    interrupt_finished.set()
                    return

                time.sleep(0.05)

        except Exception as exc:
            safety_node.get_logger().error(
                f"충돌 감시 스레드 오류: {exc}"
            )
            recovery_finished.set()
            interrupt_finished.set()

    safety_thread = threading.Thread(
        target=collision_monitor_worker,
        daemon=True,
    )
    safety_thread.start()

    # =========================================================================
    # [2-2. 모든 물리 동작을 충돌/외부중단 플래그로 차단하는 래퍼]
    # =========================================================================
    def guard_collision():
        if collision_detected.is_set():
            raise CollisionAbort(
                "충돌 감지로 기존 조립 공정을 중단합니다."
            )
        if interrupt_detected.is_set():
            raise InterruptAbort(
                "/interrupt=true 수신으로 기존 조립 공정을 중단합니다."
            )

    def checked_dsr_call(function, *args, **kwargs):
        guard_collision()
        result = function(*args, **kwargs)
        guard_collision()
        return result

    def movej(*args, **kwargs):
        return checked_dsr_call(_dsr_movej, *args, **kwargs)

    def movel(*args, **kwargs):
        return checked_dsr_call(_dsr_movel, *args, **kwargs)

    def movejx(*args, **kwargs):
        return checked_dsr_call(_dsr_movejx, *args, **kwargs)

    def set_digital_output(*args, **kwargs):
        return checked_dsr_call(
            _dsr_set_digital_output,
            *args,
            **kwargs,
        )

    def task_compliance_ctrl(*args, **kwargs):
        return checked_dsr_call(
            _dsr_task_compliance_ctrl,
            *args,
            **kwargs,
        )

    def set_desired_force(*args, **kwargs):
        return checked_dsr_call(
            _dsr_set_desired_force,
            *args,
            **kwargs,
        )

    def release_force(*args, **kwargs):
        return checked_dsr_call(
            _dsr_release_force,
            *args,
            **kwargs,
        )

    def release_compliance_ctrl(*args, **kwargs):
        return checked_dsr_call(
            _dsr_release_compliance_ctrl,
            *args,
            **kwargs,
        )

    def move_periodic(*args, **kwargs):
        return checked_dsr_call(
            _dsr_move_periodic,
            *args,
            **kwargs,
        )

    def get_current_posx(*args, **kwargs):
        return checked_dsr_call(
            _dsr_get_current_posx,
            *args,
            **kwargs,
        )

    def wait(seconds):
        """기존 DSR wait 대신 충돌에 즉시 반응하는 분할 대기."""
        guard_collision()
        deadline = time.monotonic() + max(0.0, float(seconds))
        while time.monotonic() < deadline:
            guard_collision()
            time.sleep(min(0.02, deadline - time.monotonic()))
        guard_collision()

    # WEB 수신용 글로벌 버퍼
    received_blocks = []   

    def centers_callback(msg):
        try:
            blocks_raw_list = json.loads(msg.data)
            # 버퍼 비우고 새로 수신된 리스트 추가
            received_blocks.clear()
            received_blocks.append(blocks_raw_list)
            node.get_logger().info(f"✨ [토픽 수신] 새 블록 데이터 접수 완료.")
        except Exception as e:
            node.get_logger().error(f"데이터 파싱 실패: {e}")

    # 전역 네임스페이스 경로로 구독자 생성
    node.create_subscription(String, '/centers', centers_callback, 10)

    # =========================================================================
    # [3. 그리퍼 서브 루틴]
    # =========================================================================
    def open_gripper():
        global gripper_status
        set_digital_output(1, OFF); set_digital_output(2, OFF); set_digital_output(3, OFF)
        set_digital_output(1, ON); wait(1.0); gripper_status = 0

    def close_gripper():
        global gripper_status
        set_digital_output(1, OFF); set_digital_output(2, OFF); set_digital_output(3, OFF)
        set_digital_output(2, ON); wait(1.0); gripper_status = 1

    def full_close_gripper():
        global gripper_status
        set_digital_output(1, OFF); set_digital_output(2, OFF); set_digital_output(3, OFF)
        set_digital_output(3, ON); wait(1.0); gripper_status = 1

    # =========================================================================
    # [4. 공정 초기화 및 레고 규격 정의]
    # =========================================================================
    homej = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    PICK_POSE_X, PICK_POSE_Y, PICK_POSE_Z = -212.030, -24.820, 10.000
    PICK_FIX_A, PICK_FIX_B, PICK_FIX_C = 10.77, 0.56, 77.71
    FIXED_A, FIXED_B, FIXED_C = 90.00, 0.0, -90.00

    LEGO_W_X, LEGO_W_Y, LEGO_W_Z = 16.00, 16.00, 18.5
    NOMINAL_SPAN = 144.0  

    CAL = [
        [[-143.63, -143.75, 0.03], [-143.82, 0.20, -0.01], [-143.68, 143.79, 0.12]],
        [[0.06, -142.55, -0.29], [0.23, -0.17, 0.30], [-0.17, 143.48, 0.14]],
        [[143.81, -144.18, 0.28], [143.27, -0.42, 0.23], [143.66, 143.37, 0.33]],
    ]

    def _lagrange_basis(t):
        return [0.5 * t * (t - 1.0), (1.0 - t) * (1.0 + t), 0.5 * t * (t + 1.0)]

    # 격자 좌표(grid)를 실제 로봇 좌표(posx)로 변환하는 라그랑주 보간법 사용
    def grid_to_pos(grid_x, grid_y, grid_z):
        nominal_x = grid_x * LEGO_W_X
        nominal_y = grid_y * LEGO_W_Y
        u, v = nominal_x / NOMINAL_SPAN, nominal_y / NOMINAL_SPAN
        Lu, Lv = _lagrange_basis(u), _lagrange_basis(v)
        px, py, pz = 0.0, 0.0, 0.0
        for i in range(3):
            for j in range(3):
                w = Lu[i] * Lv[j]
                px += CAL[i][j][0] * w; py += CAL[i][j][1] * w; pz += CAL[i][j][2] * w
        pz -= grid_z * LEGO_W_Z
        return px, py, pz


    ROT_TABLE = {
         144: (146.28, 0.69, -12.44), 128: (128.60, 1.25, -12.14), 112: (112.55, 1.60, -11.10),
          96: (97.19, 1.90, -11.26),  80: (81.36, 2.52, -12.45),  64: (64.92, 1.54, -12.58),
          48: (50.50, 0.98, -12.57),  32: (33.45, 1.58, -11.92),  16: (17.56, 1.91, -11.83),
           0: (1.41, 1.90, -9.73),   -16: (-13.93, -0.03, -8.67), -32: (-30.77, 0.79, -7.48),
         -48: (-47.21, 1.62, -6.98),  -64: (-61.45, 2.29, -7.91), -80: (-78.57, 2.96, -8.39),
         -96: (-93.51, 4.12, -5.94), -112: (-111.44, -0.04, -6.61), -128: (-125.56, 0.66, -7.23),
        -144: (-142.82, 1.34, -8.14),
    }
    ROT_KEYS = sorted(ROT_TABLE.keys())

    # 로봇의 그리퍼가 특정 구역에서 가야 할 위치를, 미리 측정해 둔 ROT_TABLE을 참조해 실시간으로 미세 조정해 주는 보정 함수
    def apply_rotated_correction(px, py, grid_x, grid_y, grid_z):
        nominal_x = grid_x * LEGO_W_X
        if nominal_x <= ROT_KEYS[0]: tx, ty, tz = ROT_TABLE[ROT_KEYS[0]]
        elif nominal_x >= ROT_KEYS[-1]: tx, ty, tz = ROT_TABLE[ROT_KEYS[-1]]
        else:
            for k in range(len(ROT_KEYS) - 1):
                k0, k1 = ROT_KEYS[k], ROT_KEYS[k + 1]
                if k0 <= nominal_x <= k1:
                    t = (nominal_x - k0) / (k1 - k0)
                    p0, p1 = ROT_TABLE[k0], ROT_TABLE[k1]
                    tx = p0[0] + (p1[0] - p0[0]) * t
                    ty = p0[1] + (p1[1] - p0[1]) * t
                    tz = p0[2] + (p1[2] - p0[2]) * t
                    break
        py_line0 = grid_to_pos(grid_x, 0, 0)[1]
        cy = ty + (py - py_line0)
        return tx, cy, tz - grid_z * LEGO_W_Z

    # =========================================================================
    # [4-2. 코너 틸트 삽입용 가상 축 회전 함수]
    # =========================================================================
    TILT_L = 10.0      # 핑거팁(TCP) 아래 가상 축까지 거리 [mm]
    TILT_DEG = 20.0    # 기울일 각도 [deg]
    SIDE_PUSH_BACK = 16.0   # 안착 후 옆 푸시 시 블록 중심에서 물러날 거리 [mm]

    # 🔧 [코너 전용 튜닝 변수]
    DROP_OFFSET = 4.0       # STEP 1 가착 시 벽에서 떨어질 X/Y 거리 [mm]
    NUDGE_X = 3.0           # 틸트 후 X벽 방향 단순 이동 거리 [mm]
    NUDGE_Y = 1.0           # 틸트 후 Y벽 방향 단순 이동 거리 [mm]
    NUDGE_Z = 6.0           # Z 가압 전 아래 방향 단순 하강 거리 [mm]
    Z_PRESS_TIME = 5.0      # STEP 3 Z축 힘 제어 가압 시간 [s]
    PRE_PUSH_FORCE = 5.0    # (예비) X/Y 사전 밀착 힘 [N]
    PRE_PUSH_TIME = 2.0     # (예비) X/Y 사전 밀착 시간 [s]
    WALL_PUSH_VEL = 20.0    # (예비) 벽 밀착 movel 속도 [mm/s]
    WALL_PUSH_OVER = 4.0    # (예비) 벽 밀착 movel 초과 목표 [mm]

    def tilt_about_pivot_dir(theta_deg, dir_x, dir_y, L=TILT_L):
        """
        USER_COORD 기준. TCP 아래 L[mm] 지점의 가상 축을 중심으로,
        user 좌표 XY 평면의 (dir_x, dir_y) 방향으로 툴을 기울인다.
        (공간 고정축 회전: 툴 Z가 (dir_x, dir_y) 쪽으로 눕는다)
        """
        r = math.radians(theta_deg)
        mag = math.hypot(dir_x, dir_y)
        if mag < 1e-6:
            return
        tx_, ty_ = dir_x / mag, dir_y / mag
        # 회전축 = 기울임 방향에 수직인 수평축 (z_hat x t)
        ax, ay, az = -ty_, tx_, 0.0

        # 로드리게스 공식: 임의축(단위벡터) 회전행렬
        c, s, Cc = math.cos(r), math.sin(r), 1.0 - math.cos(r)
        R_d = [
            [c + ax*ax*Cc,     ax*ay*Cc - az*s,  ax*az*Cc + ay*s],
            [ay*ax*Cc + az*s,  c + ay*ay*Cc,     ay*az*Cc - ax*s],
            [az*ax*Cc - ay*s,  az*ay*Cc + ax*s,  c + az*az*Cc   ],
        ]

        def Rz(a):
            c2, s2 = math.cos(a), math.sin(a)
            return [[c2, -s2, 0], [s2, c2, 0], [0, 0, 1]]

        def Ry(a):
            c2, s2 = math.cos(a), math.sin(a)
            return [[c2, 0, s2], [0, 1, 0], [-s2, 0, c2]]

        def matmul(A, B):
            return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
                    for i in range(3)]

        def matvec(A, v):
            return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]

        # --- 1. 현재 포즈 읽기 (USER_COORD 기준, ZYZ 오일러) ---
        cur, _sol = get_current_posx(ref=USER_COORD)
        node.get_logger().info(
            f"[틸트 입력] 현재 포즈: pos=({cur[0]:.1f}, {cur[1]:.1f}, {cur[2]:.1f}), "
            f"A={cur[3]:.2f}, B={cur[4]:.2f}, C={cur[5]:.2f}"
        )
        px, py, pz = cur[0], cur[1], cur[2]
        A_, B_, C_ = map(math.radians, (cur[3], cur[4], cur[5]))
        R_cur = matmul(matmul(Rz(A_), Ry(B_)), Rz(C_))

        # --- 2. 가상 축 위치 = TCP 위치 + R_cur * [0,0,L] ---
        pivot = [px + R_cur[0][2] * L,
                 py + R_cur[1][2] * L,
                 pz + R_cur[2][2] * L]

        # --- 3. 새 자세 = 공간 고정축 회전이므로 R_d를 '왼쪽'에 곱함 ---
        R_new = matmul(R_d, R_cur)

        # --- 4. 새 TCP 위치 = 축 위치 - R_new * [0,0,L] ---
        offs = matvec(R_new, [0, 0, L])
        p_new = [pivot[0] - offs[0], pivot[1] - offs[1], pivot[2] - offs[2]]

        # --- 5. R_new -> ZYZ 오일러 (deg) 역변환 ---
        sB = math.hypot(R_new[0][2], R_new[1][2])
        if sB < 1e-8:
            nB = 0.0 if R_new[2][2] > 0 else 180.0
            nA = 0.0
            nC = math.degrees(math.atan2(R_new[1][0], R_new[0][0]))
            if R_new[2][2] < 0:
                nC = -nC
        else:
            nB = math.degrees(math.atan2(sB, R_new[2][2]))
            nA = math.degrees(math.atan2(R_new[1][2], R_new[0][2]))
            nC = math.degrees(math.atan2(R_new[2][1], -R_new[2][0]))

        target = posx([p_new[0], p_new[1], p_new[2], nA, nB, nC])
        node.get_logger().info(
            f"[틸트] 가상 축=({pivot[0]:.1f}, {pivot[1]:.1f}, {pivot[2]:.1f}), "
            f"방향=({tx_:.2f}, {ty_:.2f}), {theta_deg:.1f}도 회전"
        )
        node.get_logger().info(
            f"[틸트 목표] pos=({p_new[0]:.1f}, {p_new[1]:.1f}, {p_new[2]:.1f}), "
            f"A={nA:.2f}, B={nB:.2f}, C={nC:.2f}"
        )
        movel(target, vel=[100, 30], acc=[200, 60], ref=USER_COORD)

        # --- 6. 도달 검증: movel이 조용히 거부되면(특이점 경로 등) movejx로 폴백 ---
        chk, _ = get_current_posx(ref=USER_COORD)
        if abs(chk[4] - nB) > 5.0:
            node.get_logger().warn(
                f"[틸트 경고] movel 미도달 (목표 B={nB:.1f} vs 현재 B={chk[4]:.1f}) "
                f"→ 관절 공간 이동(movejx)으로 폴백"
            )
            wait(0.2)
            try:
                movejx(target, vel=30, acc=30, ref=USER_COORD, sol=_sol)
            except Exception as e:
                node.get_logger().error(f"[틸트 폴백 실패] movejx 오류: {e}")
            chk2, _ = get_current_posx(ref=USER_COORD)
            node.get_logger().info(f"[틸트 폴백 결과] B={chk2[4]:.1f} (목표 {nB:.1f})")

    # =========================================================================
    # [4-2b. RG2 그리퍼 Modbus 직접 제어 (부분 열기용)]
    # =========================================================================
    COMPUTE_BOX_IP = "192.168.1.1"   # OnRobot 컴퓨트 박스 IP
    COMPUTE_BOX_PORT = 502
    RG2_DEVICE_ID = 65               # 컴퓨트 박스 unit ID (고정값)

    # 🔧 [튜닝 변수] 코너 릴리즈 시 부분 열기 파라미터
    RELEASE_WIDTH = 44.0   # 릴리즈 시 그리퍼 간격 [mm] (0 ~ 110)
    RELEASE_FORCE = 40.0   # 릴리즈 시 힘 [N] (3 ~ 40)

    rg2_client = None
    rg2_connected = False
    try:
        from pymodbus.client import ModbusTcpClient
        rg2_client = ModbusTcpClient(COMPUTE_BOX_IP, port=COMPUTE_BOX_PORT)
        rg2_connected = rg2_client.connect()
        if rg2_connected:
            node.get_logger().info(f"✅ RG2 컴퓨트 박스 연결 성공 ({COMPUTE_BOX_IP}:{COMPUTE_BOX_PORT})")
        else:
            node.get_logger().warn(f"⚠️ RG2 컴퓨트 박스 연결 실패 — 부분 열기 시 DO 방식으로 대체됩니다.")
    except ImportError:
        node.get_logger().warn("⚠️ pymodbus 미설치 — 부분 열기 시 DO 방식으로 대체됩니다. (pip install pymodbus)")
    except Exception as e:
        node.get_logger().warn(f"⚠️ RG2 Modbus 초기화 실패: {e}")

    def _rg2_write_registers(address, values):
        """pymodbus 버전별 unit ID 파라미터 이름 호환 처리"""
        last_err = None
        for kw in ('device_id', 'slave', 'unit'):
            try:
                return rg2_client.write_registers(address, values, **{kw: RG2_DEVICE_ID})
            except TypeError as e:
                last_err = e
                continue
        raise last_err

    def rg2_move(width_mm, force_n):
        """RG2를 지정 간격/힘으로 동작. 성공 시 True."""
        guard_collision()
        nonlocal rg2_connected
        if rg2_client is None:
            return False
        if not rg2_connected:
            rg2_connected = rg2_client.connect()
            if not rg2_connected:
                node.get_logger().error("RG2 Modbus 재연결 실패")
                return False
        try:
            width_raw = int(width_mm * 10)   # 0.1mm 단위
            force_raw = int(force_n * 10)    # 0.1N 단위
            # [force, width, control=1(grip)] 을 주소 0부터 한 번에 전송
            guard_collision()
            result = _rg2_write_registers(0, [force_raw, width_raw, 1])
            guard_collision()
            if result.isError():
                node.get_logger().error(f"RG2 Modbus 응답 오류: {result}")
                return False
            node.get_logger().info(f"[RG2] width={width_mm}mm, force={force_n}N 적용")
            return True
        except Exception as e:
            node.get_logger().error(f"RG2 명령 실패: {e}")
            return False

    # =========================================================================
    # [4-3. 배치 이력 관리 및 도우미 함수]
    # =========================================================================
    # 메인 루프 외부에서 선언하여 전체 공정 동안 적치 이력이 초기화되지 않고 누적
    placed_blocks = []  

    # 🔄 [리셋 토픽] 조립판을 손으로 치웠을 때 노드 재시작 없이 이력 초기화
    #    사용법: ros2 topic pub --once /reset_blocks std_msgs/msg/String "data: ''"
    def reset_callback(msg):
        placed_blocks.clear()
        node.get_logger().warn("🔄 [이력 리셋] placed_blocks가 초기화되었습니다. 조립판이 비어있는지 확인하세요!")

    node.create_subscription(String, '/reset_blocks', reset_callback, 10)

    def is_occupied(gx, gy, gz):
        return (gx, gy, gz) in placed_blocks

    def get_place_orientation(grid_x, grid_y, grid_z):
        x_neighbor = is_occupied(grid_x + 2, grid_y, grid_z) or is_occupied(grid_x - 2, grid_y, grid_z)
        y_neighbor = is_occupied(grid_x, grid_y + 2, grid_z) or is_occupied(grid_x, grid_y - 2, grid_z)
        if not x_neighbor: return FIXED_A, FIXED_B, FIXED_C, False
        elif not y_neighbor: return FIXED_A, FIXED_B, FIXED_C + 90.0, True
        # 🌟 코너(양쪽 이웃): 90도 회전 파지(Y축 평행)로 변경 — 손목 특이점 회피 목적
        #    is_rotated=True라 ROT_TABLE 위치 보정도 자동 적용됨
        else: return FIXED_A, FIXED_B, FIXED_C + 90.0, True

    # 주변 블록 배치 상태에 따른 적절한 회전각 및 슬라이드 방향 결정
    def get_slide_direction(grid_x, grid_y, grid_z):
        sx, sy = 0.0, 0.0
        # 4방향 이웃 확인 후 회전 여부 결정
        if is_occupied(grid_x + 2, grid_y, grid_z): sx += 1.0
        if is_occupied(grid_x - 2, grid_y, grid_z): sx -= 1.0
        if is_occupied(grid_x, grid_y + 2, grid_z): sy += 1.0
        if is_occupied(grid_x, grid_y - 2, grid_z): sy -= 1.0
        mag = (sx * sx + sy * sy) ** 0.5
        if mag > 0: sx, sy = sx / mag, sy / mag
        return sx, sy

    SLIDE_OFFSET, SLIDE_HOVER, SLIDE_FORCE, SLIDE_DOWN_FORCE, SLIDE_TIME = 4.0, 2.0, 5.0, 3.0, 2.5

    # 초기 구동 세팅
    open_gripper()
    movej(homej, vel=HOME_VEL, acc=HOME_ACC)
    set_ref_coord(USER_COORD)

    # 블록 집을 좌표
    posepick = posx([PICK_POSE_X, PICK_POSE_Y, PICK_POSE_Z, PICK_FIX_A, PICK_FIX_B, PICK_FIX_C])
    pose_pick_ready = deepcopy(posepick)
    pose_pick_ready[2] -= 110  

    # =========================================================================
    # [5. 지속 운영 메인 무한 루프]
    # =========================================================================
    try:
        while rclpy.ok():
            guard_collision()
            # 데이터 수신 대기
            node.get_logger().info("⏳ [시스템 대기] 웹 UI에서 '출력하기' 새 명령을 기다리는 중...")
            
            # 새 입력이 올 때까지 spin하며 대기
            while rclpy.ok() and not received_blocks:
                rclpy.spin_once(node, timeout_sec=0.1)
                guard_collision()

            # 버퍼에서 도면 데이터를 꺼냄
            incoming_blocks = received_blocks.pop(0)

            # 중심 앵커 변환 수행
            converted_blocks = [
                [block_id, [x - 0.5, y - 0.5, z]]
                for block_id, (x, y, z) in incoming_blocks
            ]

            # ⭐ [핵심 로직] 이미 조립판에 놓인 블록(placed_blocks)은 필터링하여 제외
            lego_queue = []
            for block in converted_blocks:
                gx, gy, gz = block[1]
                if (gx, gy, gz) in placed_blocks:
                    node.get_logger().info(f"⏭️ 스킵: 격자 ({gx}, {gy}, {gz})에는 이미 블록이 존재합니다.")
                    continue
                lego_queue.append(block)

            # 새로운 블록이 없으면 홈 위치 대기 상태로 루프 복귀
            if not lego_queue:
                node.get_logger().info("⚠️ 추가로 조립할 새로운 블록이 없습니다. 대기 상태로 전환합니다.")
                continue

            # Z축 우선, 원점에서 먼 구석부터 시작하여 벽면 활용 극대화
            lego_queue = sorted(lego_queue, key=lambda block: (block[1][2], block[1][0] ** 2 + block[1][1] ** 2))
            node.get_logger().info(f"🚀 신규 조립 프로세스 시작! 대상 큐: {lego_queue}")

            # 리스트에 남은 새 블록들만 순차 제어
            for current_block in lego_queue:
                guard_collision()
                # [발행] 현재 조립 중인 블록 정보 발행
                msg = String()
                msg.data = json.dumps(current_block)
                pub_block.publish(msg)
                node.get_logger().info(f"발행 완료: {msg.data}")
                DEFAULT_POSE_Z_OFFSET = -8.0
                block_type = current_block[0]
                grid_x, grid_y, grid_z = current_block[1]

                node.get_logger().info(f"🧱 조립 중: 타입={block_type}, 격자=({grid_x}, {grid_y}, {grid_z})")

                # 2. 로봇 좌표 생성
                calc_x, calc_y, calc_z = grid_to_pos(grid_x, grid_y, grid_z)
                place_a, place_b, place_c, is_rotated = get_place_orientation(grid_x, grid_y, grid_z)

                if is_rotated:
                    calc_x, calc_y, calc_z = apply_rotated_correction(calc_x, calc_y, grid_x, grid_y, grid_z)
                else:
                    calc_z += DEFAULT_POSE_Z_OFFSET

                posx1 = posx([calc_x, calc_y, calc_z, place_a, place_b, place_c])
                slide_x, slide_y = get_slide_direction(grid_x, grid_y, grid_z)
                
                # 🌟 X축 이웃도 있고, Y축 이웃도 동시에 있는 '닫힌 코너' 구조인가?
                has_x_neighbor = is_occupied(grid_x + 2, grid_y, grid_z) or is_occupied(grid_x - 2, grid_y, grid_z)
                has_y_neighbor = is_occupied(grid_x, grid_y + 2, grid_z) or is_occupied(grid_x, grid_y - 2, grid_z)
                
                # 3. 주변 상황 분석 (슬라이딩 조립 vs 코너 틸트 조립)
                use_corner = (has_x_neighbor and has_y_neighbor) # 둘 다 참이면 코너 모드!
                use_slide = (not use_corner) and (slide_x != 0.0 or slide_y != 0.0) # 코너가 아닐 때만 일반 슬라이드

                # 픽업 단계
                open_gripper()
                movel(pose_pick_ready, vel=VELOCITY, acc=ACC)
                movel(posepick, vel=VELOCITY, acc=ACC)
                close_gripper(); wait(0.2)
                movel(pose_pick_ready, vel=VELOCITY, acc=ACC)

                # 조립 접근
                pose_place_ready = deepcopy(posx1)
                pose_place_ready[2] -= 110
                movel(pose_place_ready, vel=VELOCITY, acc=ACC)

                # =============================================================
                # 📥 [놓기 메커니즘 3단 분기]
                # =============================================================
                
                # 1️⃣ [패턴 A] 한쪽 면만 막혀있을 때: 기존 1자형 미끄러뜨리기 조립
                if use_slide:
                    pose_slide_start = deepcopy(posx1)
                    pose_slide_start[0] -= slide_x * SLIDE_OFFSET
                    pose_slide_start[1] -= slide_y * SLIDE_OFFSET
                    pose_slide_start[2] -= SLIDE_HOVER

                    movel(pose_slide_start, vel=150, acc=ACC); wait(0.1)

                    task_compliance_ctrl([3000, 3000, 3000, 200, 200, 200], 0); wait(0.2)
                    fd_slide = [slide_x * SLIDE_FORCE, slide_y * SLIDE_FORCE, SLIDE_DOWN_FORCE, 0, 0, 0]
                    fc_slide = [1 if slide_x != 0 else 0, 1 if slide_y != 0 else 0, 1, 0, 0, 0]
                    set_desired_force(fd_slide, dir=fc_slide, mod=DR_FC_MOD_REL); wait(SLIDE_TIME)
                    release_force(); release_compliance_ctrl(); wait(0.2)
                    
                    task_compliance_ctrl([3000, 3000, 3000, 200, 200, 200], 0); wait(0.2)
                    set_desired_force([0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL); wait(3.0)
                    release_force(); release_compliance_ctrl(); wait(0.2)

                    open_gripper(); wait(0.5)

                # 2️⃣ [패턴 B] (틸트 방식) 양쪽 다 막힌 닫힌 코너: 기울여 넣기
                elif use_corner:
                    node.get_logger().info("[코너 틸트 삽입] 닫힌 코너 감지! 기울여 넣기 공정을 시작합니다.")

                    # 🌟 [방향 계산] 어느 쪽 벽이 막혔는지 파악 (벽이 있는 쪽 = 밀어붙일 방향)
                    cx = 1.0 if is_occupied(grid_x + 2, grid_y, grid_z) else -1.0
                    cy = 1.0 if is_occupied(grid_x, grid_y + 2, grid_z) else -1.0

                    # 🔍 [디버그] 이웃 판정 근거 출력 — 기우는 방향이 이상할 때 이 로그로 원인 추적
                    node.get_logger().info(
                        f"[코너 판정] 격자=({grid_x},{grid_y},{grid_z}) | "
                        f"+X이웃={is_occupied(grid_x + 2, grid_y, grid_z)}, "
                        f"-X이웃={is_occupied(grid_x - 2, grid_y, grid_z)}, "
                        f"+Y이웃={is_occupied(grid_x, grid_y + 2, grid_z)}, "
                        f"-Y이웃={is_occupied(grid_x, grid_y - 2, grid_z)} "
                        f"→ cx={cx}, cy={cy}"
                    )

                    # ---------------------------------------------------------
                    # STEP 1. 대각선 뒤쪽(양쪽 벽에서 각 DROP_OFFSET) 10mm 상공으로 접근
                    #         ⚠️ 수평(수직 그리퍼) 상태, 블록은 잡은 채!
                    # ---------------------------------------------------------
                    pose_drop = deepcopy(posx1)
                    pose_drop[0] -= cx * DROP_OFFSET
                    pose_drop[1] -= cy * DROP_OFFSET
                    pose_drop[2] -= 10
                    movel(pose_drop, vel=100, acc=ACC); wait(0.2)

                    # ---------------------------------------------------------
                    # STEP 2. 그리퍼 끝점(TCP) 기준 틸트 회전 (L=0, TILT_DEG도)
                    #         🌟 90도 회전 파지에 맞춰 Y벽 방향으로 틸트
                    #         ⚠️ 방향이 반대이거나 회전 중 과부하로 멈추면 -cy로 뒤집을 것!
                    # ---------------------------------------------------------
                    tilt_about_pivot_dir(TILT_DEG, 0.0, cy, L=0.0)
                    wait(0.2)
                    # 🔍 [디버그] 회전이 실제로 달성됐는지 자세 확인
                    cur_tilt, _ = get_current_posx(ref=USER_COORD)
                    node.get_logger().info(
                        f"[틸트 확인] 회전 후 자세 A={cur_tilt[3]:.1f}, B={cur_tilt[4]:.1f}, C={cur_tilt[5]:.1f} "
                        f"(수직 기준: A={posx1[3]:.1f}, B={posx1[4]:.1f}, C={posx1[5]:.1f})"
                    )

                    # ---------------------------------------------------------
                    # STEP 2-2. (신규) 기운 채 벽 방향으로 단순 이동 접근
                    #           힘 제어 없이 X는 NUDGE_X, Y는 NUDGE_Y만큼 위치 이동
                    #           (갭 DROP_OFFSET보다 작게 이동 → 비접촉 접근)
                    # ---------------------------------------------------------
                    cur_n, _ = get_current_posx(ref=USER_COORD)
                    pose_nudge_x = posx([cur_n[0] + cx * NUDGE_X, cur_n[1], cur_n[2],
                                         cur_n[3], cur_n[4], cur_n[5]])
                    movel(pose_nudge_x, vel=30, acc=100, ref=USER_COORD); wait(0.1)

                    pose_nudge_y = posx([cur_n[0] + cx * NUDGE_X, cur_n[1] + cy * NUDGE_Y, cur_n[2],
                                         cur_n[3], cur_n[4], cur_n[5]])
                    movel(pose_nudge_y, vel=30, acc=100, ref=USER_COORD); wait(0.1)

                    # ---------------------------------------------------------
                    # STEP 3. Z 방향으로 NUDGE_Z만큼 단순 하강 후,
                    #         Z축 8N 힘 제어 가압 (Z_PRESS_TIME초, 기운 채 모서리 걸기)
                    # ---------------------------------------------------------
                    cur_z, _ = get_current_posx(ref=USER_COORD)
                    pose_nudge_z = posx([cur_z[0], cur_z[1], cur_z[2] + NUDGE_Z,
                                         cur_z[3], cur_z[4], cur_z[5]])
                    movel(pose_nudge_z, vel=20, acc=100, ref=USER_COORD); wait(0.1)

                    task_compliance_ctrl([3000, 3000, 3000, 200, 200, 200], 0); wait(0.2)
                    set_desired_force([0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    wait(Z_PRESS_TIME)
                    release_force(); wait(0.1)

                    # ---------------------------------------------------------
                    # STEP 4. X벽 방향으로 힘 제어 밀착 (2초)
                    # ---------------------------------------------------------
                    set_desired_force([cx * 5, 0, 0, 0, 0, 0], dir=[1, 0, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
                    wait(2.0)
                    release_force(); wait(0.1)

                    # ---------------------------------------------------------
                    # STEP 5. 이어서 Y벽 방향으로 힘 제어 밀착 (1초)
                    # ---------------------------------------------------------
                    set_desired_force([0, cy * 5, 0, 0, 0, 0], dir=[0, 1, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
                    wait(1.0)
                    release_force(); release_compliance_ctrl(); wait(0.2)

                    # ---------------------------------------------------------
                    # STEP 6. 부분 열기 릴리즈(RG2 Modbus: RELEASE_WIDTH/RELEASE_FORCE)
                    #         + 기울어진 채 수직 이탈
                    #         → 이후 공통 프레스에서 목표 위치 상공으로 이동하며
                    #           수직 자세로 자동 복귀됨
                    # ---------------------------------------------------------
                    if not rg2_move(RELEASE_WIDTH, RELEASE_FORCE):
                        node.get_logger().warn("RG2 부분 열기 실패 → DO 방식 완전 열기로 대체")
                        open_gripper()
                    wait(1.0)   # 그리퍼 동작 완료 대기 (Modbus 명령은 즉시 리턴됨)

                    cur_now, _ = get_current_posx(ref=USER_COORD)
                    pose_up = posx([cur_now[0], cur_now[1], cur_now[2] - 40,
                                    cur_now[3], cur_now[4], cur_now[5]])
                    movel(pose_up, vel=150, acc=ACC)

                    # ---------------------------------------------------------
                    # STEP 7. 수직 복귀: 상대 회전(-20도) 대신 '절대 자세'로 복귀
                    #         (힘 제어 중 자세가 틀어졌어도 항상 정확히 수직 보장)
                    #         이후 주먹 상태로 전환
                    # ---------------------------------------------------------
                    cur_now2, _ = get_current_posx(ref=USER_COORD)
                    pose_vertical = posx([cur_now2[0], cur_now2[1], cur_now2[2],
                                          posx1[3], posx1[4], posx1[5]])  # 위치는 현재, 자세만 수직으로
                    movel(pose_vertical, vel=[100, 30], acc=[200, 60], ref=USER_COORD)
                    wait(0.2)
                    full_close_gripper()

                    # ---------------------------------------------------------
                    # STEP 8. X축 옆 푸시: 벽 반대 방향 옆 칸(PUSH_BACK) 상공으로 이동 후
                    #         PUSH_HOVER 높이까지 하강 → X벽 방향 힘 제어 밀착
                    # ---------------------------------------------------------
                    PUSH_BACK = 32.0        # 🔧 목표에서 물러날 거리 [mm] (옆 칸)
                    PUSH_FORCE = 8.0        # 🔧 옆 푸시 힘 [N]
                    PUSH_TIME = 1.2         # 🔧 옆 푸시 시간 [s]
                    PUSH_HOVER = 10.0       # 🔧 푸시 시 하강 높이 (목표 Z 기준 상공) [mm]

                    pose_x_ready = deepcopy(posx1)
                    pose_x_ready[0] -= cx * PUSH_BACK      # 붙어있는 블록 반대 방향
                    pose_x_ready[2] -= 40                  # 안전 상공
                    movel(pose_x_ready, vel=150, acc=ACC)

                    pose_x_down = deepcopy(pose_x_ready)
                    pose_x_down[2] = posx1[2] - PUSH_HOVER
                    movel(pose_x_down, vel=100, acc=ACC); wait(0.1)

                    task_compliance_ctrl([3000, 3000, 5000, 200, 200, 200], 0); wait(0.1)
                    set_desired_force([cx * PUSH_FORCE, 0, 0, 0, 0, 0], dir=[1, 0, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
                    wait(PUSH_TIME)
                    release_force(); release_compliance_ctrl(); wait(0.1)

                    pose_x_escape = deepcopy(pose_x_down)
                    pose_x_escape[2] -= 25
                    movel(pose_x_escape, vel=150, acc=ACC)

                    # ---------------------------------------------------------
                    # STEP 9. Y축 옆 푸시: 동일 방식 (X는 목표값, Y만 PUSH_BACK 후퇴)
                    # ---------------------------------------------------------
                    pose_y_ready = deepcopy(posx1)
                    pose_y_ready[1] -= cy * PUSH_BACK
                    pose_y_ready[2] -= 40
                    movel(pose_y_ready, vel=150, acc=ACC)

                    pose_y_down = deepcopy(pose_y_ready)
                    pose_y_down[2] = posx1[2] - PUSH_HOVER
                    movel(pose_y_down, vel=100, acc=ACC); wait(0.1)

                    task_compliance_ctrl([3000, 3000, 5000, 200, 200, 200], 0); wait(0.1)
                    set_desired_force([0, cy * PUSH_FORCE, 0, 0, 0, 0], dir=[0, 1, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
                    wait(PUSH_TIME)
                    release_force(); release_compliance_ctrl(); wait(0.1)

                    pose_y_escape = deepcopy(pose_y_down)
                    pose_y_escape[2] -= 25
                    movel(pose_y_escape, vel=150, acc=ACC)
                    # → 이후 공통 프레스로 이어짐

                # 3️⃣ [패턴 C] 주변에 아무것도 없을 때: 원래 쓰시던 정직한 첫 수직 놓기 로직
                else:
                    node.get_logger().info("새판 짜기: 주변에 방해물이 없으므로 정방향으로 안전하게 가압 정렬합니다.")
                    pose_place_soft = deepcopy(posx1)
                    pose_place_soft[2] -= 2.0
                    movel(pose_place_soft, vel=150, acc=ACC); wait(0.1)
                    
                    # 드릴처럼 파고들면서 누르기
                    task_compliance_ctrl([4000, 4000, 20000, 200, 200, 200], 0); wait(0.2)
                    set_desired_force([0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    # 드라이버처럼 그리퍼 회전
                    move_periodic(amp=[0, 0, 0, 0, 0, 2], period=1.0, repeat=1, ref=USER_COORD); wait(0.2)

                    release_force(); release_compliance_ctrl(); wait(0.2)
                    open_gripper(); wait(0.3)

                # 결합 프레스 매칭 (힘 제어 가압) — 모든 패턴 공통 (코너 모드도 목표 위치 상공에서 최종 프레스)
                pose_press_ready = deepcopy(posx1)
                pose_press_ready[2] -= 30
                movel(pose_press_ready, vel=VELOCITY, acc=ACC)
                full_close_gripper(); wait(0.5)

                # 상하 좌우 번갈아가면서 눌러 모난데 없게 누르기
                task_compliance_ctrl([4000, 4000, 20000, 200, 200, 200], 0); wait(0.2)
                set_desired_force([0, 0, 25, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                move_periodic(amp=[0, 5, 0, 0, 0, 0], period=2.0, repeat=2, ref=USER_COORD)
                move_periodic(amp=[5, 0, 0, 0, 0, 0], period=2.0, repeat=1, ref=USER_COORD)
                wait(0.5)
                release_force(); release_compliance_ctrl(); wait(0.2)

                # 안전하게 올라오기
                movel(pose_place_ready, vel=VELOCITY, acc=ACC); wait(0.2)
                open_gripper(); wait(0.5)

                # 💡 현재 적치 이력을 시스템 메모리에 영구 등록
                placed_blocks.append((grid_x, grid_y, grid_z))

            # 한 사이클 완료 후 원위치 및 재대기
            node.get_logger().info("✅ 현재 도면 갱신 완료! 안전을 위해 홈 포지션으로 이동 후 다음 명령을 대기합니다.")
            movej(homej, vel=HOME_VEL, acc=HOME_ACC)

    except CollisionAbort as e:
        node.get_logger().error(
            f"공정 중단: {e}"
        )
        node.get_logger().warn(
            "Recovery 진입 완료를 기다립니다."
        )
        recovery_finished.wait(timeout=40.0)
        if recovery_entered.is_set():
            node.get_logger().warn(
                "Recovery 진입 완료. UI 복구 완료를 기다립니다."
            )
            wait_for_ui_mode_one_and_clear_collision()
        else:
            node.get_logger().error(
                "Recovery 진입을 확인하지 못했습니다."
            )
    except InterruptAbort as e:
        node.get_logger().error(
            f"외부 중단: {e}"
        )
        interrupt_finished.wait(timeout=5.0)
        node.get_logger().warn(
            "이동/힘 제어 중단 요청 완료. 조립 공정을 종료합니다."
        )
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 수동 종료되었습니다.")
    except Exception as e:
        # DSR 이동 함수가 충돌 때문에 먼저 예외를 반환하고, 감시 스레드가
        # 수십 ms 뒤 상태를 확인하는 경우를 고려해 잠깐 기다린다.
        time.sleep(0.2)
        if collision_detected.is_set():
            node.get_logger().error(
                f"충돌로 인한 Robot Error: {e}"
            )
            recovery_finished.wait(timeout=40.0)
            if recovery_entered.is_set():
                wait_for_ui_mode_one_and_clear_collision()
        elif interrupt_detected.is_set():
            node.get_logger().error(
                f"외부 중단 처리 중 Robot Error: {e}"
            )
            interrupt_finished.wait(timeout=5.0)
        else:
            node.get_logger().info(f"Robot Error: {e}")
            try:
                release_force()
                release_compliance_ctrl()
            except Exception:
                pass
    finally:
        # 충돌 후에는 Recovery 상태이므로 홈 이동을 절대 시도하지 않는다.
        if (
            not collision_detected.is_set()
            and not interrupt_detected.is_set()
        ):
            try:
                movej(homej, vel=HOME_VEL, acc=HOME_ACC)
            except Exception as e:
                node.get_logger().warn(
                    f"종료 홈 이동 실패: {e}"
                )
            try:
                set_ref_coord(0)
            except Exception:
                pass
        elif collision_detected.is_set():
            node.get_logger().warn(
                "충돌/Recovery 상태이므로 종료 홈 이동을 생략합니다."
            )
        else:
            node.get_logger().warn(
                "/interrupt 외부 중단 상태이므로 종료 홈 이동을 생략합니다."
            )

        safety_monitor_shutdown.set()
        try:
            safety_thread.join(timeout=2.0)
        except Exception:
            pass

        try:
            if rg2_client is not None: rg2_client.close()
        except Exception:
            pass

        try:
            safety_executor.remove_node(safety_node)
            safety_executor.shutdown()
        except Exception:
            pass
        try:
            safety_node.destroy_node()
        except Exception:
            pass

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()