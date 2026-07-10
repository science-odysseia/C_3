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
VELOCITY, ACC = 300, 300

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
            DR_FC_MOD_REL, set_ref_coord, move_periodic
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
    # [4-3. 배치 이력 관리 및 도우미 함수]
    # =========================================================================
    # ⭐ 메인 루프 외부에서 선언하여 전체 공정 동안 적치 이력이 초기화되지 않고 누적됨
    placed_blocks = []  

    def is_occupied(gx, gy, gz):
        return (gx, gy, gz) in placed_blocks

    def get_place_orientation(grid_x, grid_y, grid_z):
        x_neighbor = is_occupied(grid_x + 2, grid_y, grid_z) or is_occupied(grid_x - 2, grid_y, grid_z)
        y_neighbor = is_occupied(grid_x, grid_y + 2, grid_z) or is_occupied(grid_x, grid_y - 2, grid_z)
        if not x_neighbor: return FIXED_A, FIXED_B, FIXED_C, False
        elif not y_neighbor: return FIXED_A, FIXED_B, FIXED_C + 90.0, True
        else: return FIXED_A, FIXED_B, FIXED_C, False

    def get_slide_direction(grid_x, grid_y, grid_z):
        sx, sy = 0.0, 0.0
        if is_occupied(grid_x + 2, grid_y, grid_z): sx += 1.0
        if is_occupied(grid_x - 2, grid_y, grid_z): sx -= 1.0
        if is_occupied(grid_x, grid_y + 2, grid_z): sy += 1.0
        if is_occupied(grid_x, grid_y - 2, grid_z): sy -= 1.0
        mag = (sx * sx + sy * sy) ** 0.5
        if mag > 0: sx, sy = sx / mag, sy / mag
        return sx, sy

    SLIDE_OFFSET, SLIDE_HOVER, SLIDE_FORCE, SLIDE_DOWN_FORCE, SLIDE_TIME = 4.0, 2.0, 5.0, 3.0, 2.5
    PUSH_START_OFFSET = 12.0  # 밀기 시작점 거리 (그리퍼 두께 여유분)
    SAFE_Z_HOVER = 25.0      # 블록 상공 대기 안전 높이
    PUSH_Z_MARGIN = 4.0      # 하강하여 블록 허리를 밀기 위한 Z축 보정값
    PUSH_FORCE = 8.0         # 밀어붙이는 수평 힘
    PUSH_DOWN_FORCE = 3.0    # 밀 때 블록이 들리지 않게 누르는 지시 가압

    # 초기 구동 세팅
    open_gripper()
    movej(homej, vel=VELOCITY, acc=ACC)
    set_ref_coord(USER_COORD)

    posepick = posx([PICK_POSE_X, PICK_POSE_Y, PICK_POSE_Z, PICK_FIX_A, PICK_FIX_B, PICK_FIX_C])
    pose_pick_ready = deepcopy(posepick)
    pose_pick_ready[2] -= 110  

    # =========================================================================
    # [5. 지속 운영 메인 무한 루프]
    # =========================================================================
    try:
        while rclpy.ok():
            node.get_logger().info("⏳ [시스템 대기] 웹 UI에서 '출력하기' 새 명령을 기다리는 중...")
            
            # 새 입력이 올 때까지 spin하며 대기
            while rclpy.ok() and not received_blocks:
                rclpy.spin_once(node, timeout_sec=0.1)

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

            # Z축 레이어 순서대로 정렬
            lego_queue = sorted(lego_queue, key=lambda block: block[1][2])
            node.get_logger().info(f"🚀 신규 조립 프로세스 시작! 대상 큐: {lego_queue}")

            # 리스트에 남은 새 블록들만 순차 제어
            for current_block in lego_queue:
                DEFAULT_POSE_Z_OFFSET = -8.0
                block_type = current_block[0]
                grid_x, grid_y, grid_z = current_block[1]

                node.get_logger().info(f"🧱 조립 중: 타입={block_type}, 격자=({grid_x}, {grid_y}, {grid_z})")

                calc_x, calc_y, calc_z = grid_to_pos(grid_x, grid_y, grid_z)
                place_a, place_b, place_c, is_rotated = get_place_orientation(grid_x, grid_y, grid_z)

                if is_rotated:
                    calc_x, calc_y, calc_z = apply_rotated_correction(calc_x, calc_y, grid_x, grid_y, grid_z)
                else:
                    calc_z += DEFAULT_POSE_Z_OFFSET

                posx1 = posx([calc_x, calc_y, calc_z, place_a, place_b, place_c])
                # 1. 이웃 판별을 먼저 수행
                has_x_neighbor = (grid_x + 2, grid_y, grid_z) in placed_blocks or (grid_x - 2, grid_y, grid_z) in placed_blocks
                has_y_neighbor = (grid_x, grid_y + 2, grid_z) in placed_blocks or (grid_x, grid_y - 2, grid_z) in placed_blocks

                use_corner = (has_x_neighbor and has_y_neighbor) # 코너 여부 확정

                # 2. 코너가 '아닐 때만' 슬라이드 모드를 활성화하도록 제한
                slide_x, slide_y = get_slide_direction(grid_x, grid_y, grid_z)
                use_slide = (not use_corner) and (slide_x != 0.0 or slide_y != 0.0)

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
                # 📥 [우선순위가 재정렬된 3단 분기 놓기 메커니즘]
                # =============================================================
                
                # 🌟 [우선순위 1등] X축, Y축 이웃이 모두 있는 '마지막 닫힌 구석(Corner)'일 때
                if use_corner:
                    node.get_logger().info("🚜 [단동 불도저] 구석 감지! 방향을 계산하여 푸시 공정을 시작합니다.")
                    
                    # 🌟 [방향 계산] 어느 쪽 벽이 막혔는지 파악하여 밀어야 할 방향(cx, cy) 설정
                    # 블록이 +방향에 있으면 -방향에서 시작해 +방향(1.0)으로 밀어야 함.
                    cx = 1.0 if (grid_x + 2, grid_y, grid_z) in placed_blocks else -1.0
                    cy = 1.0 if (grid_x, grid_y + 2, grid_z) in placed_blocks else -1.0
                    
                    # ---------------------------------------------------------
                    # STEP 1. 대각선 뒤쪽(DROP_OFFSET)에 바짝 붙여 가착
                    # ---------------------------------------------------------
                    pose_drop = deepcopy(posx1)
                    pose_drop[0] -= cx * SLIDE_OFFSET
                    pose_drop[1] -= cy * SLIDE_OFFSET
                    pose_drop[2] += -12
                    
                    movel(pose_drop, vel=100, acc=ACC); wait(0.2)
                    open_gripper(); wait(0.3)
                    
                    pose_hover = deepcopy(pose_drop)
                    pose_hover[2] -= SAFE_Z_HOVER  
                    movel(pose_hover, vel=150, acc=ACC)

                    # ---------------------------------------------------------
                    # STEP 2. X축 푸시
                    # ---------------------------------------------------------
                    pose_x_ready = deepcopy(pose_hover)
                    # Y축은 가착 위치(DROP_OFFSET)를 유지하고, X축만 밀기 시작점(PUSH_START)으로 더 물러남
                    pose_x_ready[0] = posx1[0] - (cx * PUSH_START_OFFSET)
                    movel(pose_x_ready, vel=150, acc=ACC)
                    
                    pose_x_down = deepcopy(pose_x_ready)
                    pose_x_down[2] = posx1[2] + PUSH_Z_MARGIN  
                    movel(pose_x_down, vel=100, acc=ACC); wait(0.1)
                    
                    task_compliance_ctrl([3000, 3000, 5000, 200, 200, 200], 0); wait(0.1)
                    # cx 부호에 따라 양수/음수 힘이 자동으로 결정되어 올바른 방향으로 밀어냄
                    set_desired_force([cx * PUSH_FORCE, 0.0, PUSH_DOWN_FORCE, 0, 0, 0], dir=[1, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    wait(1.2)  
                    release_force(); release_compliance_ctrl(); wait(0.1)

                    pose_x_escape = deepcopy(pose_x_down)
                    pose_x_escape[2] -= SAFE_Z_HOVER
                    movel(pose_x_escape, vel=150, acc=ACC)

                    # ---------------------------------------------------------
                    # STEP 3. Y축 푸시
                    # ---------------------------------------------------------
                    pose_y_ready = deepcopy(pose_hover)
                    # 이제 X축은 정수 타겟 위치로 갱신하고, Y축만 밀기 시작점으로 물러남
                    pose_y_ready[0] = posx1[0] 
                    pose_y_ready[1] = posx1[1] - (cy * PUSH_START_OFFSET)  
                    movel(pose_y_ready, vel=150, acc=ACC)
                    
                    pose_y_down = deepcopy(pose_y_ready)
                    pose_y_down[2] = posx1[2] + PUSH_Z_MARGIN
                    movel(pose_y_down, vel=100, acc=ACC); wait(0.1)
                    
                    task_compliance_ctrl([3000, 3000, 5000, 200, 200, 200], 0); wait(0.1)
                    # cy 부호에 따라 Y축 푸시 힘 자동 결정
                    set_desired_force([0.0, cy * PUSH_FORCE, PUSH_DOWN_FORCE, 0, 0, 0], dir=[0, 1, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    wait(1.2)  
                    release_force(); release_compliance_ctrl(); wait(0.1)

                    pose_y_escape = deepcopy(pose_y_down)
                    pose_y_escape[2] -= SAFE_Z_HOVER
                    movel(pose_y_escape, vel=150, acc=ACC)

                elif use_slide:
                    node.get_logger().info("📐 [슬라이드 모드] 한쪽 벽면을 타며 대각선 슬라이드 진입합니다.")
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

                # 🍃 [우선순위 3등] 주변에 아무 블록도 없는 맨땅일 때: 기존 수직 안착 조립
                else:
                    node.get_logger().info("🍃 [일반 모드] 방해물이 없으므로 정방향으로 안전하게 가압 정렬합니다.")
                    pose_place_soft = deepcopy(posx1)
                    pose_place_soft[2] -= 2.0
                    movel(pose_place_soft, vel=150, acc=ACC); wait(0.1)
                    
                    task_compliance_ctrl([4000, 4000, 20000, 200, 200, 200], 0); wait(0.2)
                    set_desired_force([0, 0, 25, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    move_periodic(amp=[0, 0, 0, 0, 0, 2], period=2.0, repeat=2, ref=USER_COORD); wait(1.0)
                    release_force(); release_compliance_ctrl(); wait(0.2)
                    
                    open_gripper(); wait(0.3)

                # =============================================================
                # 🎯 [최종 공통 결합 공정] 구석/슬라이드/일반 패턴 상관없이 상공에서 꾹 누르기
                # =============================================================
                node.get_logger().info("🏋️ [최종 프레스] 블록의 완전 정수 상공에서 수직 결합 가압을 실시합니다.")
                pose_press_ready = deepcopy(posx1)
                pose_press_ready[2] -= 30  # 목표 조립 위치의 30mm 위에서 대기
                movel(pose_press_ready, vel=VELOCITY, acc=ACC)
                full_close_gripper(); wait(0.5)

                # 단단하게 결합하기 위해 Z축 방향 강한 힘 제어 작동 및 미세 뒤틀기(X, Y 방향 주기 진동)
                task_compliance_ctrl([4000, 4000, 20000, 200, 200, 200], 0); wait(0.2)
                set_desired_force([0, 0, 25, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                move_periodic(amp=[0, 5, 0, 0, 0, 0], period=2.0, repeat=3, ref=USER_COORD)
                move_periodic(amp=[5, 0, 0, 0, 0, 0], period=2.0, repeat=2, ref=USER_COORD)
                wait(3.5)
                release_force(); release_compliance_ctrl(); wait(0.2)

                # 후퇴 및 리셋 오픈
                movel(pose_place_ready, vel=VELOCITY, acc=ACC); wait(0.2)
                open_gripper(); wait(0.5)

                # 💡 현재 적치 이력을 시스템 메모리에 영구 등록
                placed_blocks.append((grid_x, grid_y, grid_z))

            # 한 사이클 완료 후 원위치 및 재대기
            node.get_logger().info("✅ 현재 도면 갱신 완료! 안전을 위해 홈 포지션으로 이동 후 다음 명령을 대기합니다.")
            movej(homej, vel=VELOCITY, acc=ACC)

    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 수동 종료되었습니다.")
    except Exception as e:
        node.get_logger().info(f"Robot Error: {e}")
        try: release_force(); release_compliance_ctrl()
        except: pass
    finally:
        movej(homej, vel=VELOCITY, acc=ACC)
        try: set_ref_coord(0)
        except: pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()