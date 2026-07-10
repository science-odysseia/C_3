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

# 레고 조립판 기준으로 티칭된 사용자 좌표계 ID
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
            get_digital_input, wait, OFF, ON, task_compliance_ctrl,
            set_desired_force, release_force, release_compliance_ctrl,
            DR_FC_MOD_REL, set_ref_coord
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    # 2-1. WEB에서 보내는 json 리스트 수신
    received_blocks = []   # 수신 결과 저장용

    def centers_callback(msg):
        try:
            blocks_raw_list = json.loads(msg.data)
            received_blocks.append(blocks_raw_list)
            node.get_logger().info(f"블록 리스트 수신: {blocks_raw_list}")
        except Exception as e:
            node.get_logger().error(f"데이터 파싱 실패: {e}")

    node.create_subscription(String, '/centers', centers_callback, 10)

    # =========================================================================
    # [3. 그리퍼 서브 루틴]
    # =========================================================================
    def open_gripper():
        global gripper_status
        print("그리퍼 열기")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(3, OFF)
        set_digital_output(1, ON)
        wait(1.0)
        gripper_status = 0

    def close_gripper():
        global gripper_status
        print("그리퍼 스터드 집을 정도로만 닫기")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(3, OFF)
        set_digital_output(2, ON)
        wait(1.0)
        gripper_status = 1

    def full_close_gripper():
        global gripper_status
        print("그리퍼 완전 닫기")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(3, OFF)
        set_digital_output(3, ON)
        wait(1.0)
        gripper_status = 1

    # =========================================================================
    # [4. 공정 초기화 및 레고 규격 정의]
    # =========================================================================
    # ※ set_tool / set_tcp 는 컨트롤러에 이미 저장된 설정을 사용하므로 생략.

    homej = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])

    # 픽업 좌표 -- 102번 사용자 좌표계 기준
    PICK_POSE_X, PICK_POSE_Y, PICK_POSE_Z = -212.030, -24.820, 10.000
    PICK_FIX_A, PICK_FIX_B, PICK_FIX_C = 10.77, 0.56, 77.71

    # 조립 시 그리퍼 자세 (102번 좌표계 기준, 102_origin 티칭 자세값 사용)
    FIXED_A, FIXED_B, FIXED_C = 90.80, 0.17, -92.19

    # 레고 격자 1칸(grid 1)당 이론 크기 (mm)
    LEGO_W_X = 16.00
    LEGO_W_Y = 16.00
    LEGO_W_Z = 19.0

    # =========================================================================
    # [4-1. 9점 티칭 기반 2차(Biquadratic) 보간 캘리브레이션 -- 기본 자세용]
    # =========================================================================
    NOMINAL_SPAN = 144.0  # 명목 기준 간격 (9칸 x 16mm)

    CAL = [
        # i=0 : 명목 x = -144
        [
            [-143.63, -143.75, 0.03],   # (x-144, y-144) = 102_downright
            [-143.82,    0.20, -0.01],  # (x-144, y   0) = 102_right
            [-143.68,  143.79, 0.12],   # (x-144, y+144) = 102_upright
        ],
        # i=1 : 명목 x = 0
        [
            [   0.06, -142.55, -0.29],  # (x0, y-144) = 102_down
            [   0.23,   -0.17,  0.30],  # (x0, y   0) = 102_origin
            [  -0.17,  143.48,  0.14],  # (x0, y+144) = 102_up
        ],
        # i=2 : 명목 x = +144
        [
            [ 143.81, -144.18,  0.28],  # (x+144, y-144) = 102_downleft
            [ 143.27,   -0.42,  0.23],  # (x+144, y   0) = 102_left
            [ 143.66,  143.37,  0.33],  # (x+144, y+144) = 102_upleft
        ],
    ]

    def _lagrange_basis(t):
        """정규화 좌표 t(-1~+1)에 대한 2차 라그랑주 기저함수 3개 반환."""
        L0 = 0.5 * t * (t - 1.0)
        L1 = (1.0 - t) * (1.0 + t)
        L2 = 0.5 * t * (t + 1.0)
        return [L0, L1, L2]

    def grid_to_pos(grid_x, grid_y, grid_z):
        """
        격자 좌표 -> 102번 좌표계 기준 실좌표(mm). 9점 2차 보간.
        grid_z(층)는 보간된 z에 층 높이를 반영 (102 z축 아래 방향: 위층 = -z).
        """
        nominal_x = grid_x * LEGO_W_X
        nominal_y = grid_y * LEGO_W_Y
        u = nominal_x / NOMINAL_SPAN
        v = nominal_y / NOMINAL_SPAN

        if abs(u) > 1.0 or abs(v) > 1.0:
            node.get_logger().warn(
                f"주의: grid({grid_x},{grid_y})는 캘리브레이션 범위(±9칸) 밖입니다. "
                "외삽 계산되므로 정확도가 떨어질 수 있습니다."
            )

        Lu = _lagrange_basis(u)
        Lv = _lagrange_basis(v)

        px, py, pz = 0.0, 0.0, 0.0
        for i in range(3):
            for j in range(3):
                w = Lu[i] * Lv[j]
                px += CAL[i][j][0] * w
                py += CAL[i][j][1] * w
                pz += CAL[i][j][2] * w

        pz -= grid_z * LEGO_W_Z
        return px, py, pz

    # =========================================================================
    # [4-2. 90도 회전 자세 place 전용 보정 -- 19점 룩업 테이블]
    # =========================================================================
    # 블록을 문 상태 + 90도 회전 자세로 실측한 x라인(y=0) 19점.
    # key: 명목 x (= grid_x * 16), value: 실측 (x, y, z)
    ROT_TABLE = {
         144: (146.28,  0.69, -12.44),
         128: (128.60,  1.25, -12.14),
         112: (112.55,  1.60, -11.10),
          96: ( 97.19,  1.90, -11.26),
          80: ( 81.36,  2.52, -12.45),
          64: ( 64.92,  1.54, -12.58),
          48: ( 50.50,  0.98, -12.57),
          32: ( 33.45,  1.58, -11.92),
          16: ( 17.56,  1.91, -11.83),
           0: (  1.41,  1.90,  -9.73),
         -16: (-13.93, -0.03,  -8.67),
         -32: (-30.77,  0.79,  -7.48),
         -48: (-47.21,  1.62,  -6.98),
         -64: (-61.45,  2.29,  -7.91),
         -80: (-78.57,  2.96,  -8.39),
         -96: (-93.51,  4.12,  -5.94),
        -112: (-111.44, -0.04, -6.61),
        -128: (-125.56,  0.66,  -7.23),
        -144: (-142.82,  1.34,  -8.14),
    }
    ROT_KEYS = sorted(ROT_TABLE.keys())   # -144 ~ +144

    def apply_rotated_correction(px, py, grid_x, grid_y, grid_z):
        """
        회전 자세 place용 좌표: 19점 실측 테이블에서 직접 조회/보간.
        y != 0 위치는 9점 보간의 y 변화량을 티칭 y값에 더해 근사.
        """
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

        cz = tz - grid_z * LEGO_W_Z
        return tx, cy, cz

    # =========================================================================
    # [4-3. 배치 이력 기반 충돌 회피 자세 결정 + 슬라이드 방향 계산]
    # =========================================================================
    placed_blocks = []  # 조립 완료 블록의 grid 좌표 [(gx, gy, gz), ...]

    def is_occupied(gx, gy, gz):
        """해당 grid 좌표에 이미 블록이 배치되어 있는지 확인"""
        return (gx, gy, gz) in placed_blocks

    def get_place_orientation(grid_x, grid_y, grid_z):
        """
        이미 배치된 블록과 그리퍼 손가락이 겹치지 않는 자세를 자동 선택.
        반환값: (a, b, c, is_rotated)
        """
        x_neighbor = is_occupied(grid_x + 2, grid_y, grid_z) or \
                     is_occupied(grid_x - 2, grid_y, grid_z)

        y_neighbor = is_occupied(grid_x, grid_y + 2, grid_z) or \
                     is_occupied(grid_x, grid_y - 2, grid_z)

        if not x_neighbor:
            node.get_logger().info("자세 판단: 기본 자세로 place (x방향 이웃 없음)")
            return FIXED_A, FIXED_B, FIXED_C, False
        elif not y_neighbor:
            node.get_logger().info("자세 판단: 90도 회전 place (x방향 이웃 존재, y방향 비어있음)")
            return FIXED_A, FIXED_B, FIXED_C + 90.0, True
        else:
            node.get_logger().warn(
                "경고: x/y 양방향 모두 인접 블록 존재! 충돌 위험이 있으니 "
                "조립 순서를 조정하세요. (일단 기본 자세로 진행)"
            )
            return FIXED_A, FIXED_B, FIXED_C, False

    def get_slide_direction(grid_x, grid_y, grid_z):
        """
        슬라이드 인서션을 위한 "이웃 방향" 단위벡터 (sx, sy) 반환.
        이웃이 없으면 (0, 0) -> 슬라이드 없이 기존 방식으로 place.
        양쪽에 이웃이 있으면 상쇄되어 0이 될 수 있음 (그 경우도 일반 place).
        """
        sx, sy = 0.0, 0.0
        if is_occupied(grid_x + 2, grid_y, grid_z):
            sx += 1.0   # +x 쪽에 이웃 -> +x 방향으로 밀어 밀착
        if is_occupied(grid_x - 2, grid_y, grid_z):
            sx -= 1.0
        if is_occupied(grid_x, grid_y + 2, grid_z):
            sy += 1.0
        if is_occupied(grid_x, grid_y - 2, grid_z):
            sy -= 1.0
        # 정규화 (대각 이웃 동시 존재 시 대각 방향으로)
        mag = (sx * sx + sy * sy) ** 0.5
        if mag > 0:
            sx, sy = sx / mag, sy / mag
        return sx, sy

    # -------------------------------------------------------------------
    # [슬라이드 인서션 파라미터]
    # -------------------------------------------------------------------
    SLIDE_OFFSET = 4.0    # 이웃 반대쪽으로 떨어져서 시작하는 거리 (mm), 3~5 튜닝
    SLIDE_HOVER = 2.0     # 슬라이드 중 판 표면에서 뜨는 높이 (mm)
    #  - 너무 크면(5+) 블록 옆면끼리 안 닿고 이웃 위로 타넘음
    #  - 너무 작으면(0.5) 슬라이드 중 판 스터드에 걸림
    SLIDE_FORCE = 5.0     # 이웃 방향 수평 미는 힘 (N), 5~10 튜닝
    #  - 너무 세면 결합된 이웃 블록을 밀어 뽑을 수 있음
    SLIDE_DOWN_FORCE = 3.0  # 슬라이드 중 살짝 아래로 누르는 힘 (N)
    SLIDE_TIME = 2.5      # 밀착까지 기다리는 시간 (초)

    # =========================================================================
    # [5. 큐(Queue) 데이터 준비 및 Z축 정렬]
    # =========================================================================
    # 첫 번째 값(2)은 블록 타입 식별자 (2x2 블록 = 2로 고정)

    # 메시지 수신 받아서 리스트화 하기
    node.get_logger().info("웹 UI에서 centers 토픽 수신 대기 중...")
    while rclpy.ok() and not received_blocks:
        rclpy.spin_once(node, timeout_sec=0.1)

    incoming_blocks = received_blocks[0]

    incoming_blocks = [
        [block_id, [x - 0.5, y - 0.5, z]]
        for block_id, (x, y, z) in incoming_blocks
    ]

    lego_queue = sorted(incoming_blocks, key=lambda block: block[1][2])
    node.get_logger().info(f"정렬된 레고 조립 큐 목록: {lego_queue}")

    # =========================================================================
    # [6. 메인 공정 시작]
    # =========================================================================
    open_gripper()
    movej(homej, vel=VELOCITY, acc=ACC)

    # 기준 좌표계를 102번(조립판 좌표계)으로 전역 설정
    set_ref_coord(USER_COORD)
    node.get_logger().info(f"기준 좌표계를 {USER_COORD}번으로 설정")

    posepick = posx([PICK_POSE_X, PICK_POSE_Y, PICK_POSE_Z, PICK_FIX_A, PICK_FIX_B, PICK_FIX_C])
    pose_pick_ready = deepcopy(posepick)
    pose_pick_ready[2] -= 110  # 픽업 상공 대기 (102 z축 아래 방향: 위 = -z)

    open_gripper()

    try:
        for current_block in lego_queue:
            DEFAULT_POSE_Z_OFFSET = -6.0
            block_type = current_block[0]
            grid_x, grid_y, grid_z = current_block[1]

            node.get_logger().info(
                f"\n현재 조립 중인 블록: 타입={block_type}, 격자위치=({grid_x}, {grid_y}, {grid_z})"
            )

            # --- [9점 보간] 격자 좌표 -> 102 기준 보정 실좌표 ---
            calc_x, calc_y, calc_z = grid_to_pos(grid_x, grid_y, grid_z)

            # 배치 이력 기반 자세 자동 선택
            place_a, place_b, place_c, is_rotated = get_place_orientation(grid_x, grid_y, grid_z)

            # --- [회전 보정] 90도 회전 place는 19점 룩업 테이블 사용 ---
            if is_rotated:
                calc_x, calc_y, calc_z = apply_rotated_correction(
                    calc_x, calc_y, grid_x, grid_y, grid_z)
                node.get_logger().info("회전 자세 보정 적용됨 (19점 룩업 테이블)")
            else:
                calc_z += DEFAULT_POSE_Z_OFFSET
            node.get_logger().info(
                f"최종 목표 좌표(102 기준): x={calc_x:.2f}, y={calc_y:.2f}, z={calc_z:.2f}"
            )

            # 최종 타겟 좌표 (누르기 기준 정위치)
            posx1 = posx([calc_x, calc_y, calc_z, place_a, place_b, place_c])

            # 슬라이드 방향 계산 (이웃 있으면 그 방향, 없으면 (0,0))
            slide_x, slide_y = get_slide_direction(grid_x, grid_y, grid_z)
            use_slide = (slide_x != 0.0 or slide_y != 0.0)

            # -----------------------------------------------------------------
            # 픽업 단계
            # -----------------------------------------------------------------
            movel(pose_pick_ready, vel=VELOCITY, acc=ACC)
            movel(posepick, vel=VELOCITY, acc=ACC)

            close_gripper()
            wait(0.2)

            movel(pose_pick_ready, vel=VELOCITY, acc=ACC)

            # -----------------------------------------------------------------
            # 조립 위치 진입 및 놓기
            # -----------------------------------------------------------------
            pose_place_ready = deepcopy(posx1)
            pose_place_ready[2] -= 110

            movel(pose_place_ready, vel=VELOCITY, acc=ACC)  # 상공 이동

            if use_slide:
                # =========================================================
                # [슬라이드 인서션] 이웃 블록을 물리적 기준면으로 사용
                # 1) 이웃 반대쪽 SLIDE_OFFSET 만큼 떨어진 지점,
                #    표면에서 SLIDE_HOVER 만큼 뜬 높이로 하강
                # 2) 블록을 문 채로 이웃 방향으로 힘 제어 수평 슬라이드
                #    -> 블록 옆면이 이웃 옆면에 닿으며 자동 밀착 (힘 평형)
                # 3) 밀착 상태에서 그리퍼 열어 놓기
                # =========================================================
                node.get_logger().info(
                    f"슬라이드 인서션 모드: 이웃 방향=({slide_x:+.1f}, {slide_y:+.1f})"
                )

                pose_slide_start = deepcopy(posx1)
                pose_slide_start[0] -= slide_x * SLIDE_OFFSET  # 이웃 반대쪽 이격
                pose_slide_start[1] -= slide_y * SLIDE_OFFSET
                pose_slide_start[2] -= SLIDE_HOVER             # 표면에서 살짝 뜸

                movel(pose_slide_start, vel=150, acc=ACC)      # 슬라이드 시작점 하강
                wait(0.1)

                # 힘 제어 수평 슬라이드 (블록 문 상태)
                node.get_logger().info("힘 제어 슬라이드: 이웃 블록에 밀착 중...")
                task_compliance_ctrl([3000, 3000, 3000, 200, 200, 200], 0)
                wait(0.2)

                fd_slide = [slide_x * SLIDE_FORCE,
                            slide_y * SLIDE_FORCE,
                            SLIDE_DOWN_FORCE, 0, 0, 0]
                fc_slide = [1 if slide_x != 0 else 0,
                            1 if slide_y != 0 else 0,
                            1, 0, 0, 0]
                set_desired_force(fd_slide, dir=fc_slide, mod=DR_FC_MOD_REL)
                wait(SLIDE_TIME)   # 이웃에 닿아 힘 평형으로 정지할 때까지

                release_force()
                release_compliance_ctrl()
                wait(0.2)
                
                node.get_logger().info("힘 제어 슬라이드: 이웃 블록에 밀착 중...")
                task_compliance_ctrl([3000, 3000, 3000, 200, 200, 200], 0)
                wait(0.2)

                fd_slide = [0, 0, 15, 0, 0, 0]
                fc_slide = [0, 0, 1, 0, 0, 0]
                set_desired_force(fd_slide, dir=fc_slide, mod=DR_FC_MOD_REL)
                wait(3.0)   # 바닥 닿아 힘 평형으로 정지할 때까지

                release_force()
                release_compliance_ctrl()
                wait(0.2)

                # 밀착 상태에서 놓기
                open_gripper()
                wait(0.5)
            else:
                # =========================================================
                # [일반 place] 이웃 없음: 표면 5mm 위에서 살짝 놓기
                # =========================================================
                pose_place_soft = deepcopy(posx1)
                pose_place_soft[2] -= 5.0

                movel(pose_place_soft, vel=150, acc=ACC)
                wait(0.1)

                open_gripper()
                wait(0.5)

            # -----------------------------------------------------------------
            # 힘 제어 수직 누르기 (최종 결합 -- 공통)
            # -----------------------------------------------------------------
            pose_press_ready = deepcopy(posx1)
            pose_press_ready[2] -= 35
            movel(pose_press_ready, vel=VELOCITY, acc=ACC)

            full_close_gripper()
            wait(0.5)

            node.get_logger().info("힘 제어 활성화: 블록 누르기 시작")
            fd = [0, 0, 25, 0, 0, 0]
            fc_dir = [0, 0, 1, 0, 0, 0]

            # x/y 강성을 낮춰 누르는 동안 블록이 스터드를 따라
            # 정위치로 미끄러져 들어갈 수 있게 함
            task_compliance_ctrl([4000, 4000, 20000, 200, 200, 200], 0)
            wait(0.2)

            set_desired_force(fd, dir=fc_dir, mod=DR_FC_MOD_REL)
            wait(4.0)

            node.get_logger().info("힘 제어 해제 및 이번 블록 조립 완료")
            release_force()
            release_compliance_ctrl()
            wait(0.2)

            # 조립 완료 후 상공 복귀 및 그리퍼 열기
            movel(pose_place_ready, vel=VELOCITY, acc=ACC)
            wait(0.2)
            open_gripper()
            wait(0.5)

            # 배치 완료 등록 (다음 블록의 자세/슬라이드 판단에 사용)
            placed_blocks.append((grid_x, grid_y, grid_z))
            node.get_logger().info(f"배치 완료 목록 갱신: {placed_blocks}")

        node.get_logger().info("모든 레고 블록 큐 조립이 완료되었습니다!")

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")
    except Exception as e:
        node.get_logger().info(f"Robot Error: {e}")
        # 힘 제어가 걸린 채 에러가 났을 수 있으므로 안전 해제 시도
        try:
            release_force()
            release_compliance_ctrl()
        except Exception:
            pass
    finally:
        movej(homej, vel=VELOCITY, acc=ACC)

        try:
            set_ref_coord(0)
            node.get_logger().info("기준 좌표계를 베이스(0)로 복원")
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()