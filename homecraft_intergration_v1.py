import rclpy
import DR_init
import json

from copy import deepcopy
from std_msgs.msg import String

# =========================================================================
# [1. 로봇 및 환경 설정 환경 상수]
# =========================================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 600, 600

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

gripper_status = 0
USER_COORD = 102


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # =========================================================================
    # [2. API 임포트]
    # =========================================================================
    try:
        from DSR_ROBOT2 import (
            movej, movel, set_digital_output,
            wait, OFF, ON, task_compliance_ctrl,
            set_desired_force, release_force, release_compliance_ctrl,
            DR_FC_MOD_REL, set_ref_coord, amove_periodic, DR_TOOL
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    # WEB 수신용 글로벌 버퍼
    received_blocks = []

    def centers_callback(msg):
        try:
            blocks_raw_list = json.loads(msg.data)
            received_blocks.clear()
            received_blocks.append(blocks_raw_list)
            node.get_logger().info("[토픽 수신] 새 블록 데이터 접수 완료.")
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
    FIXED_A, FIXED_B, FIXED_C = 90.80, 0.17, -92.19

    LEGO_W_X, LEGO_W_Y, LEGO_W_Z = 16.00, 16.00, 19.0
    NOMINAL_SPAN = 144.0

    # 기본 자세 place용 z 오프셋 (튜닝값)
    DEFAULT_POSE_Z_OFFSET = -6.0

    # ---------------------------------------------------------------
    # [힘 제어 동작 파라미터 모음]
    # ---------------------------------------------------------------
    # 슬라이드 인서션 (같은 층 이웃 밀착)
    SLIDE_OFFSET, SLIDE_HOVER = 4.0, 2.0
    SLIDE_FORCE, SLIDE_DOWN_FORCE, SLIDE_TIME = 5.0, 3.0, 2.5

    # 터치다운 place (2층 이상: 힘 제어로 내려가 닿으면 멈춤)
    TOUCH_FORCE = 8.0
    TOUCH_TIME = 3.0
    TOUCH_START_ABOVE = 15.0

    # 위글 프레스 (문 채로 약하게 누르며 소폭 비틀어 스터드 안착 유도)
    WIGGLE_AMP = 4.0        # rz 진폭 (±도)
    WIGGLE_PERIOD = 1.0
    WIGGLE_REPEAT = 2
    WIGGLE_FORCE = 15.0

    # 컴플라이언트 삽입 (포위 배치: 문 채로 위글 하강 삽입)
    INSERT_START_ABOVE = 12.0
    INSERT_FORCE = 10.0
    INSERT_WIGGLE_AMP = 3.0
    INSERT_WIGGLE_PERIOD = 0.8
    INSERT_TIME = 4.0

    # 지지점 순차 프레스 (최종 결합)
    PRESS_FORCE = 25.0
    PRESS_TIME = 4.0

    # =========================================================================
    # [4-1. 9점 티칭 기반 2차 보간 캘리브레이션 -- 기본 자세용]
    # =========================================================================
    CAL = [
        [[-143.63, -143.75, 0.03], [-143.82, 0.20, -0.01], [-143.68, 143.79, 0.12]],
        [[0.06, -142.55, -0.29], [0.23, -0.17, 0.30], [-0.17, 143.48, 0.14]],
        [[143.81, -144.18, 0.28], [143.27, -0.42, 0.23], [143.66, 143.37, 0.33]],
    ]

    def _lagrange_basis(t):
        return [0.5 * t * (t - 1.0), (1.0 - t) * (1.0 + t), 0.5 * t * (t + 1.0)]

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

    # =========================================================================
    # [4-2. 90도 회전 자세 place 전용 보정 -- 19점 룩업 테이블]
    # =========================================================================
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

    def apply_rotated_correction(px, py, grid_x, grid_y, grid_z):
        nominal_x = grid_x * LEGO_W_X
        if nominal_x <= ROT_KEYS[0]:
            tx, ty, tz = ROT_TABLE[ROT_KEYS[0]]
        elif nominal_x >= ROT_KEYS[-1]:
            tx, ty, tz = ROT_TABLE[ROT_KEYS[-1]]
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
    # [4-3. 배치 이력 관리 및 판단 함수]
    # =========================================================================
    # 메인 무한 루프 밖에서 선언 -> 전체 운영 동안 적치 이력 누적 유지
    placed_blocks = []

    def is_occupied(gx, gy, gz):
        return (gx, gy, gz) in placed_blocks

    def get_place_orientation(grid_x, grid_y, grid_z):
        """그리퍼 손가락과 같은 층 이웃의 충돌을 피하는 자세 선택"""
        x_neighbor = is_occupied(grid_x + 2, grid_y, grid_z) or is_occupied(grid_x - 2, grid_y, grid_z)
        y_neighbor = is_occupied(grid_x, grid_y + 2, grid_z) or is_occupied(grid_x, grid_y - 2, grid_z)
        if not x_neighbor:
            return FIXED_A, FIXED_B, FIXED_C, False
        elif not y_neighbor:
            return FIXED_A, FIXED_B, FIXED_C + 90.0, True
        else:
            return FIXED_A, FIXED_B, FIXED_C, False

    def get_slide_direction(grid_x, grid_y, grid_z):
        """같은 층 이웃 방향 단위벡터. 없거나 상쇄면 (0,0)"""
        sx, sy = 0.0, 0.0
        if is_occupied(grid_x + 2, grid_y, grid_z): sx += 1.0
        if is_occupied(grid_x - 2, grid_y, grid_z): sx -= 1.0
        if is_occupied(grid_x, grid_y + 2, grid_z): sy += 1.0
        if is_occupied(grid_x, grid_y - 2, grid_z): sy -= 1.0
        mag = (sx * sx + sy * sy) ** 0.5
        if mag > 0:
            sx, sy = sx / mag, sy / mag
        return sx, sy

    def is_enclosed(grid_x, grid_y, grid_z):
        """
        마주보는 양쪽에 이웃이 있는 '포위 배치' 판정
        (사각형 채우기의 마지막 블록 등 -> 슬라이드 불가 + 손가락 충돌 위험)
        """
        x_both = is_occupied(grid_x + 2, grid_y, grid_z) and \
                 is_occupied(grid_x - 2, grid_y, grid_z)
        y_both = is_occupied(grid_x, grid_y + 2, grid_z) and \
                 is_occupied(grid_x, grid_y - 2, grid_z)
        return x_both or y_both

    def get_support_offsets(grid_x, grid_y, grid_z):
        """
        아래층에서 이 블록을 받치는 지점들의 (dx, dy) 오프셋(mm) 리스트.
        브릿지/캔틸레버 배치에서 지지된 결합부 위만 눌러 시소 기울어짐 방지.
        """
        if grid_z == 0:
            return [(0.0, 0.0)]   # 1층은 판이 전체 지지

        supports = []
        if is_occupied(grid_x - 1, grid_y, grid_z - 1):
            supports.append((-8.0, 0.0))
        if is_occupied(grid_x + 1, grid_y, grid_z - 1):
            supports.append((+8.0, 0.0))
        if is_occupied(grid_x, grid_y - 1, grid_z - 1):
            supports.append((0.0, -8.0))
        if is_occupied(grid_x, grid_y + 1, grid_z - 1):
            supports.append((0.0, +8.0))
        if is_occupied(grid_x, grid_y, grid_z - 1):
            supports.append((0.0, 0.0))

        if not supports:
            node.get_logger().warn("경고: 아래층 지지 블록 없음! 중심 누르기로 진행")
            supports.append((0.0, 0.0))
        return supports

    # =========================================================================
    # [4-4. 힘 제어 동작 서브루틴]
    # =========================================================================
    def wiggle_place():
        """
        블록을 문 채로: 약한 힘으로 누르며 rz를 소폭 왕복 회전.
        스터드가 소켓을 스스로 찾아 들어가게 함 (위치/각도 오차 1~2mm/2~3도 흡수).
        원점에서 먼 좌표의 첫 블록 안정 배치용.
        """
        task_compliance_ctrl([3000, 3000, 3000, 200, 200, 100], 0)
        wait(0.2)
        set_desired_force([0, 0, WIGGLE_FORCE, 0, 0, 0],
                          dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        amove_periodic(amp=[0, 0, 0, 0, 0, WIGGLE_AMP],
                       period=WIGGLE_PERIOD,
                       repeat=WIGGLE_REPEAT,
                       ref=DR_TOOL)
        wait(WIGGLE_PERIOD * WIGGLE_REPEAT + 0.5)
        release_force()
        release_compliance_ctrl()
        wait(0.2)

    def compliant_insert(target_pose):
        """
        포위 배치용: 블록을 문 채로 힘 제어 하강 + 위글로 슬롯에 삽입.
        이웃 벽/스터드가 가이드 역할, 낮은 강성 + 위글이 걸림(재밍)을 풀어줌.
        """
        pose_start = deepcopy(target_pose)
        pose_start[2] -= INSERT_START_ABOVE
        movel(pose_start, vel=150, acc=ACC)
        wait(0.1)

        task_compliance_ctrl([2500, 2500, 3000, 200, 200, 100], 0)
        wait(0.2)
        set_desired_force([0, 0, INSERT_FORCE, 0, 0, 0],
                          dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        amove_periodic(amp=[0, 0, 0, 0, 0, INSERT_WIGGLE_AMP],
                       period=INSERT_WIGGLE_PERIOD,
                       repeat=int(INSERT_TIME / INSERT_WIGGLE_PERIOD),
                       ref=DR_TOOL)
        wait(INSERT_TIME + 0.5)

        release_force()
        release_compliance_ctrl()
        wait(0.2)

    def press_at_supports(target_pose, grid_x, grid_y, grid_z):
        """
        지지점 위를 순차적으로 눌러 최종 결합.
        1층/정층 = 중심 1곳, 브릿지 = 지지 결합부 2곳, 캔틸레버 = 지지쪽 1곳.
        (full_close 상태에서 호출할 것)
        """
        press_points = get_support_offsets(grid_x, grid_y, grid_z)
        node.get_logger().info(f"누르기 지점 {len(press_points)}곳: {press_points}")

        for (pdx, pdy) in press_points:
            pose_press = deepcopy(target_pose)
            pose_press[0] += pdx
            pose_press[1] += pdy
            pose_press[2] -= 12
            movel(pose_press, vel=VELOCITY, acc=ACC)

            task_compliance_ctrl([4000, 4000, 20000, 200, 200, 200], 0)
            wait(0.2)
            set_desired_force([0, 0, PRESS_FORCE, 0, 0, 0],
                              dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            wait(PRESS_TIME)
            release_force()
            release_compliance_ctrl()
            wait(0.2)

            # 다음 지점 이동 전 살짝 들어올림 (블록 윗면 긁힘 방지)
            pose_press[2] -= 15
            movel(pose_press, vel=150, acc=ACC)

    # =========================================================================
    # [5. 초기 구동 세팅]
    # =========================================================================
    open_gripper()
    movej(homej, vel=VELOCITY, acc=ACC)
    set_ref_coord(USER_COORD)
    node.get_logger().info(f"기준 좌표계를 {USER_COORD}번으로 설정")

    posepick = posx([PICK_POSE_X, PICK_POSE_Y, PICK_POSE_Z, PICK_FIX_A, PICK_FIX_B, PICK_FIX_C])
    pose_pick_ready = deepcopy(posepick)
    pose_pick_ready[2] -= 110

    # =========================================================================
    # [6. 지속 운영 메인 무한 루프]
    # =========================================================================
    try:
        while rclpy.ok():
            node.get_logger().info("[시스템 대기] 웹 UI에서 새 명령을 기다리는 중...")

            while rclpy.ok() and not received_blocks:
                rclpy.spin_once(node, timeout_sec=0.1)

            incoming_blocks = received_blocks.pop(0)

            # 중심 앵커 변환 (UI 블록 중심 -> TCP 기준점)
            converted_blocks = [
                [block_id, [x - 0.5, y - 0.5, z]]
                for block_id, (x, y, z) in incoming_blocks
            ]

            # 이미 조립된 블록은 필터링 (증분 조립)
            lego_queue = []
            for block in converted_blocks:
                gx, gy, gz = block[1]
                if (gx, gy, gz) in placed_blocks:
                    node.get_logger().info(f"스킵: 격자 ({gx}, {gy}, {gz})에는 이미 블록 존재")
                    continue
                lego_queue.append(block)

            if not lego_queue:
                node.get_logger().info("추가로 조립할 새 블록 없음. 대기 상태로 전환.")
                continue

            # z(층) -> y -> x 순 정렬: 포위 배치 발생을 최소화하는 래스터 순서
            lego_queue = sorted(lego_queue, key=lambda b: (b[1][2], b[1][1], b[1][0]))
            node.get_logger().info(f"신규 조립 프로세스 시작! 대상 큐: {lego_queue}")

            for current_block in lego_queue:
                block_type = current_block[0]
                grid_x, grid_y, grid_z = current_block[1]

                node.get_logger().info(f"조립 중: 타입={block_type}, 격자=({grid_x}, {grid_y}, {grid_z})")

                # --- 좌표 계산 (자세별 캘리브레이션) ---
                calc_x, calc_y, calc_z = grid_to_pos(grid_x, grid_y, grid_z)
                place_a, place_b, place_c, is_rotated = get_place_orientation(grid_x, grid_y, grid_z)

                if is_rotated:
                    calc_x, calc_y, calc_z = apply_rotated_correction(
                        calc_x, calc_y, grid_x, grid_y, grid_z)
                    node.get_logger().info("회전 자세 보정 적용 (19점 룩업 테이블)")
                else:
                    calc_z += DEFAULT_POSE_Z_OFFSET

                node.get_logger().info(
                    f"최종 목표(102 기준): x={calc_x:.2f}, y={calc_y:.2f}, z={calc_z:.2f}")

                posx1 = posx([calc_x, calc_y, calc_z, place_a, place_b, place_c])

                # --- place 모드 판단 ---
                enclosed = is_enclosed(grid_x, grid_y, grid_z)
                slide_x, slide_y = get_slide_direction(grid_x, grid_y, grid_z)
                use_slide = (not enclosed) and (slide_x != 0.0 or slide_y != 0.0)

                # -------------------------------------------------------------
                # 픽업 단계
                # -------------------------------------------------------------
                open_gripper()
                movel(pose_pick_ready, vel=VELOCITY, acc=ACC)
                movel(posepick, vel=VELOCITY, acc=ACC)
                close_gripper(); wait(0.2)
                movel(pose_pick_ready, vel=VELOCITY, acc=ACC)

                # -------------------------------------------------------------
                # 조립 접근
                # -------------------------------------------------------------
                pose_place_ready = deepcopy(posx1)
                pose_place_ready[2] -= 110
                movel(pose_place_ready, vel=VELOCITY, acc=ACC)

                # -------------------------------------------------------------
                # 놓기 메커니즘 (4모드 자동 분기)
                # -------------------------------------------------------------
                if enclosed:
                    # [모드 1: 컴플라이언트 삽입] 포위 배치 (마지막 블록 등)
                    # 문 채로 위글 하강 -> 이웃 벽이 가이드, 걸림은 위글이 풀어줌
                    node.get_logger().info("컴플라이언트 삽입 모드 (포위 배치)")
                    compliant_insert(posx1)
                    open_gripper(); wait(0.5)

                elif use_slide:
                    # [모드 2: 슬라이드 인서션] 같은 층 이웃 밀착
                    node.get_logger().info(
                        f"슬라이드 인서션 모드: 이웃 방향=({slide_x:+.1f}, {slide_y:+.1f})")
                    pose_slide_start = deepcopy(posx1)
                    pose_slide_start[0] -= slide_x * SLIDE_OFFSET
                    pose_slide_start[1] -= slide_y * SLIDE_OFFSET
                    pose_slide_start[2] -= SLIDE_HOVER

                    movel(pose_slide_start, vel=150, acc=ACC); wait(0.1)

                    task_compliance_ctrl([3000, 3000, 3000, 200, 200, 200], 0); wait(0.2)
                    fd_slide = [slide_x * SLIDE_FORCE, slide_y * SLIDE_FORCE,
                                SLIDE_DOWN_FORCE, 0, 0, 0]
                    fc_slide = [1 if slide_x != 0 else 0,
                                1 if slide_y != 0 else 0, 1, 0, 0, 0]
                    set_desired_force(fd_slide, dir=fc_slide, mod=DR_FC_MOD_REL)
                    wait(SLIDE_TIME)
                    release_force(); release_compliance_ctrl(); wait(0.2)

                    # 밀착 후 문 채로 예비 누르기 (15N/3초)
                    task_compliance_ctrl([3000, 3000, 3000, 200, 200, 200], 0); wait(0.2)
                    set_desired_force([0, 0, 15, 0, 0, 0],
                                      dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    wait(3.0)
                    release_force(); release_compliance_ctrl(); wait(0.2)

                    open_gripper(); wait(0.5)

                elif grid_z >= 1:
                    # [모드 3: 터치다운 + 위글] 2층 이상, 이웃 없음
                    # 힘 제어로 내려가 아래층 실제 높이에서 멈춤 -> 위글로 안착
                    node.get_logger().info("터치다운 place 모드 (2층 이상)")
                    pose_touch_start = deepcopy(posx1)
                    pose_touch_start[2] -= TOUCH_START_ABOVE
                    movel(pose_touch_start, vel=150, acc=ACC); wait(0.1)

                    task_compliance_ctrl([4000, 4000, 3000, 200, 200, 200], 0); wait(0.2)
                    set_desired_force([0, 0, TOUCH_FORCE, 0, 0, 0],
                                      dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    wait(TOUCH_TIME)
                    release_force(); release_compliance_ctrl(); wait(0.2)

                    # 닿은 상태에서 문 채로 위글 -> 아래층 스터드에 안착
                    wiggle_place()

                    open_gripper(); wait(0.5)

                else:
                    # [모드 4: soft place + 위글] 1층, 이웃 없음 (첫 블록 등)
                    pose_place_soft = deepcopy(posx1)
                    pose_place_soft[2] -= 2.0
                    movel(pose_place_soft, vel=150, acc=ACC); wait(0.1)

                    # 문 채로 위글 -> 원점에서 먼 좌표의 오차 흡수
                    wiggle_place()

                    open_gripper(); wait(0.5)

                # -------------------------------------------------------------
                # 최종 결합: 지지점 순차 프레스
                # -------------------------------------------------------------
                full_close_gripper(); wait(0.5)
                press_at_supports(posx1, grid_x, grid_y, grid_z)

                movel(pose_place_ready, vel=VELOCITY, acc=ACC); wait(0.2)
                open_gripper(); wait(0.5)

                # 적치 이력 등록 (다음 블록의 자세/모드/지지점 판단에 사용)
                placed_blocks.append((grid_x, grid_y, grid_z))
                node.get_logger().info(f"배치 완료 목록 갱신: {placed_blocks}")

            node.get_logger().info("현재 도면 갱신 완료! 홈 이동 후 다음 명령 대기.")
            movej(homej, vel=VELOCITY, acc=ACC)

    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 수동 종료되었습니다.")
    except Exception as e:
        node.get_logger().info(f"Robot Error: {e}")
        try:
            release_force(); release_compliance_ctrl()
        except Exception:
            pass
    # 변경 (Ctrl+C 시 컨텍스트 종료로 인한 traceback 방지)
    finally:
        try:
            movej(homej, vel=VELOCITY, acc=ACC)
        except Exception:
            pass
        try:
            set_ref_coord(0)
        except Exception:
            pass
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()