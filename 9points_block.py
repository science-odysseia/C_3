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
VELOCITY, ACC = 100, 100

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

    # # 2-1. WEB에서 보내는 json 리스트 수신
    # received_blocks = []   # 수신 결과 저장용 (리스트로 감싸서 클로저에서 수정 가능하게)

    # def centers_callback(msg):
    #     try:
    #         blocks_raw_list = json.loads(msg.data)
    #         received_blocks.append(blocks_raw_list)
    #         node.get_logger().info(f"블록 리스트 수신: {blocks_raw_list}")
    #     except Exception as e:
    #         node.get_logger().error(f"데이터 파싱 실패: {e}")

    # node.create_subscription(String, 'centers', centers_callback, 10)

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

    # 픽업 좌표 -- 102번 사용자 좌표계 기준 (102 기준 재티칭값, b=0.56으로
    # 102 좌표계 수직 자세 패턴과 일치함을 확인)
    PICK_POSE_X, PICK_POSE_Y, PICK_POSE_Z = -212.030, -24.820, 10.000
    PICK_FIX_A, PICK_FIX_B, PICK_FIX_C = 10.77, 0.56, 77.71

    # 조립 시 그리퍼 자세 (102번 좌표계 기준, 102_origin 티칭 자세값 사용)
    # ※ 9점의 a,b,c가 제각각으로 보이지만 b가 전부 0 근처(오일러각 특이점)라
    #   실제 그리퍼 방향은 거의 동일함. 대표값으로 origin 자세를 고정 사용.
    FIXED_A, FIXED_B, FIXED_C = 90.80, 0.17, -92.19

    # 레고 격자 1칸(grid 1)당 이론 크기 (mm) -- 보간의 명목(nominal) 좌표 계산용
    LEGO_W_X = 16.00
    LEGO_W_Y = 16.00
    LEGO_W_Z = 19.0

    # =========================================================================
    # [4-1. 9점 티칭 기반 2차(Biquadratic) 보간 캘리브레이션 -- 기본 자세용]
    # =========================================================================
    # 티칭펜던트에서 실측한 9점 (102번 좌표계 기준 x, y, z)
    # 각 점의 "명목 위치"는 원점에서 ±144mm (= 9칸 x 16mm) 지점.
    # 실측값과 명목값의 미세한 차이(판 뒤틀림, 스케일 오차, z 기울기)를
    # 2차 보간으로 전 영역에 걸쳐 자동 보정한다.
    #
    # 3x3 배열 인덱스: CAL[i][j]
    #   i: 명목 x가 -144(0), 0(1), +144(2)
    #   j: 명목 y가 -144(0), 0(1), +144(2)
    NOMINAL_SPAN = 144.0  # 명목 기준 간격 (9칸 x 16mm)

    CAL = [
        # i=0 : 명목 x = -144  (티칭명 기준 "right" 계열)
        [
            [-143.63, -143.75, 0.03],   # j=0: (x-144, y-144) = 102_downright
            [-143.82,    0.20, -0.01],  # j=1: (x-144, y   0) = 102_right
            [-143.68,  143.79, 0.12],   # j=2: (x-144, y+144) = 102_upright
        ],
        # i=1 : 명목 x = 0
        [
            [   0.06, -142.55, -0.29],  # j=0: (x0, y-144) = 102_down
            [   0.23,   -0.17,  0.30],  # j=1: (x0, y   0) = 102_origin
            [  -0.17,  143.48,  0.14],  # j=2: (x0, y+144) = 102_up
        ],
        # i=2 : 명목 x = +144  (티칭명 기준 "left" 계열)
        [
            [ 143.81, -144.18,  0.28],  # j=0: (x+144, y-144) = 102_downleft
            [ 143.27,   -0.42,  0.23],  # j=1: (x+144, y   0) = 102_left
            [ 143.66,  143.37,  0.33],  # j=2: (x+144, y+144) = 102_upleft
        ],
    ]

    def _lagrange_basis(t):
        """
        정규화 좌표 t (-1 ~ +1)에 대한 2차 라그랑주 기저함수 3개 반환.
        t = -1, 0, +1 지점에서 각각 자기 기저만 1이 되는 2차 다항식.
        """
        L0 = 0.5 * t * (t - 1.0)    # t=-1 지점 기저
        L1 = (1.0 - t) * (1.0 + t)  # t=0  지점 기저
        L2 = 0.5 * t * (t + 1.0)    # t=+1 지점 기저
        return [L0, L1, L2]

    def grid_to_pos(grid_x, grid_y, grid_z):
        """
        격자 좌표(grid) -> 102번 좌표계 기준 실좌표(mm)로 변환.
        9점 실측 데이터를 2차 보간하여 판의 뒤틀림/스케일/기울기를 자동 보정.
        grid_z(층)는 보간된 z에 층 높이를 더해 처리.
        """
        # grid -> 명목 mm 좌표 -> 정규화 좌표(-1 ~ +1)
        nominal_x = grid_x * LEGO_W_X
        nominal_y = grid_y * LEGO_W_Y
        u = nominal_x / NOMINAL_SPAN
        v = nominal_y / NOMINAL_SPAN

        # 티칭 범위(±144mm) 밖은 외삽이 되어 정확도가 떨어질 수 있음
        if abs(u) > 1.0 or abs(v) > 1.0:
            node.get_logger().warn(
                f"주의: grid({grid_x},{grid_y})는 캘리브레이션 범위(±9칸) 밖입니다. "
                "외삽 계산되므로 정확도가 떨어질 수 있습니다."
            )

        Lu = _lagrange_basis(u)
        Lv = _lagrange_basis(v)

        # 3x3 점을 기저함수 가중합으로 보간 (x, y, z 각각)
        px, py, pz = 0.0, 0.0, 0.0
        for i in range(3):
            for j in range(3):
                w = Lu[i] * Lv[j]
                px += CAL[i][j][0] * w
                py += CAL[i][j][1] * w
                pz += CAL[i][j][2] * w

        # 층(z index) 반영
        pz -= grid_z * LEGO_W_Z

        return px, py, pz

    # =========================================================================
    # [4-2. 90도 회전 자세 place 전용 보정]
    # =========================================================================
    # 블록을 문 상태 + 90도 회전 자세로 x라인 19점을 실측 티칭한 데이터에서
    # 최소제곱으로 도출한 계통 보정값.
    # - 회전 자세로 물면 그리퍼-블록 편심 때문에 TCP가 x/y로 약 +1.5mm씩 밀림
    # - z는 x 위치에 따라 선형으로 기울어짐 (라인 전체에 걸쳐 약 6.6mm 변화)
    #   (19점 z 실측값의 1차 근사: z = ROT_Z0 + ROT_Z_SLOPE * x, 잔차 RMS 1.15mm)
    ROT_DX = 1.50            # 회전 자세 x 오프셋 (mm)
    ROT_DY = 1.56            # 회전 자세 y 오프셋 (mm)
    ROT_Z0 = -9.76           # 회전 자세 z(x) 근사 절편
    ROT_Z_SLOPE = -0.02293   # 회전 자세 z(x) 근사 기울기

    def apply_rotated_correction(px, py, grid_z):
        """
        회전 자세 place용 좌표 보정.
        x, y : 9점 보간 결과에 회전 편심 오프셋을 더함
        z    : 19점 라인 실측 기반 z(x) 근사값 사용
               (블록을 문 상태에서 안착 완료된 TCP 높이 그 자체)
               + 층 높이 반영 (102 z축이 아래 방향이므로 위층 = -z)
        ※ 주의: 19점은 y≈0 라인에서만 딴 데이터라 z의 y 의존성은 미반영.
          y가 크게 다른 위치의 회전 place에서 오차가 보이면 y라인 추가 티칭 필요.
        """
        cx = px + ROT_DX
        cy = py + ROT_DY
        cz = (ROT_Z0 + ROT_Z_SLOPE * px) - grid_z * LEGO_W_Z
        return cx, cy, cz

    # =========================================================================
    # [4-3. 배치 이력 기반 충돌 회피 자세 결정]
    # =========================================================================
    # 조립 완료된 블록의 grid 좌표를 기억하는 리스트
    placed_blocks = []  # [(grid_x, grid_y, grid_z), ...]

    def is_occupied(gx, gy, gz):
        """해당 grid 좌표에 이미 블록이 배치되어 있는지 확인"""
        return (gx, gy, gz) in placed_blocks

    def get_place_orientation(grid_x, grid_y, grid_z):
        """
        이미 배치된 블록과 그리퍼 손가락이 겹치지 않는 자세를 자동 선택.
        - 그리퍼 기본 자세: 손가락이 x축 방향으로 벌어짐
          -> x방향 이웃(±2 grid)에 블록이 있으면 충돌 위험
        - 90도 회전 자세: 손가락이 y축 방향으로 벌어짐
          -> y방향 이웃(±2 grid)에 블록이 있으면 충돌 위험
        같은 층(grid_z)의 이웃만 검사 (아래층 블록은 손가락과 부딪히지 않음).

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

    # =========================================================================
    # [5. 큐(Queue) 데이터 준비 및 Z축 정렬]
    # =========================================================================
    # 리스트의 첫 번째 값(2)은 좌표가 아니라 "블록 타입" 식별자이며,
    # 2x2 블록 타입을 의미하므로 모든 블록에서 항상 2로 고정된다.
    incoming_blocks = [
        [2, [0, 0, 0]],
        [2, [2, 0, 0]],
        [2, [4, 0, 0]],
        [2, [-2, 0, 0]],
        [2, [-4, 0, 0]],
        # [2, [0, 0, 1]],
        # [2, [2, 0, 1]],
        # [2, [-2, 0, 1]],
        # [2, [0, 2, 1]],
        # [2, [0, -2, 1]]
    ]

    # 메시지 수신 받아서 리스트화 하기
    # node.get_logger().info("웹 UI에서 centers 토픽 수신 대기 중...")
    # while rclpy.ok() and not received_blocks:
    #     rclpy.spin_once(node, timeout_sec=0.1)   # 콜백이 돌 수 있게 spin

    # incoming_blocks = received_blocks[0]

    # # x, y에 0.5씩 빼서 TCP 이동 좌표로 변환 (블록 중심 -> 조립 기준점 보정)
    # incoming_blocks = [
    #     [block_id, [x - 0.5, y - 0.5, z]]
    #     for block_id, (x, y, z) in incoming_blocks
    # ]

    # [핵심 로직] 각 요소의 block[1][2], 즉 z 좌표를 기준으로 오름차순 정렬 (큐 생성)
    lego_queue = sorted(incoming_blocks, key=lambda block: block[1][2])
    node.get_logger().info(f"정렬된 레고 조립 큐 목록: {lego_queue}")

    # =========================================================================
    # [6. 메인 공정 시작]
    # =========================================================================
    open_gripper()
    movej(homej, vel=VELOCITY, acc=ACC)

    # [핵심] 기준 좌표계를 102번(조립판 좌표계)으로 전역 설정
    # 이후 모든 movel의 좌표가 102 기준으로 해석됨 (movej는 관절 기반이라 무관)
    set_ref_coord(USER_COORD)
    node.get_logger().info(f"기준 좌표계를 {USER_COORD}번으로 설정")

    posepick = posx([PICK_POSE_X, PICK_POSE_Y, PICK_POSE_Z, PICK_FIX_A, PICK_FIX_B, PICK_FIX_C])
    pose_pick_ready = deepcopy(posepick)
    pose_pick_ready[2] -= 110  # 픽업 상공 대기 위치 계산 (102 기준 z)

    open_gripper()

    try:
        # 정렬된 큐에서 블록을 하나씩 꺼내어 순차 조립 수행
        for current_block in lego_queue:
            block_type = current_block[0]
            grid_x, grid_y, grid_z = current_block[1]

            node.get_logger().info(
                f"\n현재 조립 중인 블록: 타입={block_type}, 격자위치=({grid_x}, {grid_y}, {grid_z})"
            )

            # --- [9점 보간] 격자 좌표 -> 102번 좌표계 기준 보정된 실좌표 ---
            calc_x, calc_y, calc_z = grid_to_pos(grid_x, grid_y, grid_z)

            # 배치 이력 기반으로 충돌 없는 place 자세 자동 선택
            place_a, place_b, place_c, is_rotated = get_place_orientation(grid_x, grid_y, grid_z)

            # --- [회전 보정] 90도 회전 place인 경우 19점 실측 기반 보정 적용 ---
            if is_rotated:
                calc_x, calc_y, calc_z = apply_rotated_correction(calc_x, calc_y, grid_z)
                node.get_logger().info("회전 자세 보정 적용됨 (19점 라인 실측 기반)")

            node.get_logger().info(
                f"최종 목표 좌표(102 기준): x={calc_x:.2f}, y={calc_y:.2f}, z={calc_z:.2f}"
            )

            # 최종 타겟 좌표 생성 (102번 좌표계 기준 값)
            posx1 = posx([calc_x, calc_y, calc_z, place_a, place_b, place_c])
            pose_place_soft = deepcopy(posx1)
            pose_place_soft[2] -= 5.0   # 표면 5mm 위에서 놓기 (102 z축이 아래 방향이므로 위 = -z)

            # -----------------------------------------------------------------
            # 픽업 단계: 매 블록마다 픽업 위치로 하강 -> 집기 -> 후퇴
            # -----------------------------------------------------------------
            movel(pose_pick_ready, vel=VELOCITY, acc=ACC)  # 픽업 상공 이동
            movel(posepick, vel=VELOCITY, acc=ACC)         # 픽업 위치 하강

            close_gripper()  # 블록 집기
            wait(0.2)

            movel(pose_pick_ready, vel=VELOCITY, acc=ACC)  # 집은 후 상공 후퇴

            # -----------------------------------------------------------------
            # 조립 위치 진입
            # -----------------------------------------------------------------
            pose_place_ready = deepcopy(posx1)
            pose_place_ready[2] -= 110  # 조립 목표 상공 대기 위치 계산

            movel(pose_place_ready, vel=VELOCITY, acc=ACC)  # 상공 이동
            movel(pose_place_soft, vel=150, acc=ACC)        # 가안착 위치 하강 (표면 5mm 위)

            wait(0.1)

            # -----------------------------------------------------------------
            # 힘 제어 기반 누르기 동작
            # -----------------------------------------------------------------
            open_gripper()  # 살짝 놓기
            wait(0.5)

            pose_press_ready = deepcopy(posx1)
            pose_press_ready[2] -= 35
            movel(pose_press_ready, vel=VELOCITY, acc=ACC)  # 위로 대기

            full_close_gripper()  # 누르기 전용 그리퍼 오므리기
            wait(0.5)

            node.get_logger().info("힘 제어 활성화: 블록 누르기 시작")
            fd = [0, 0, 25, 0, 0, 0]
            fc_dir = [0, 0, 1, 0, 0, 0]

            task_compliance_ctrl([20000, 20000, 20000, 200, 200, 200], 0)
            wait(0.2)

            set_desired_force(fd, dir=fc_dir, mod=DR_FC_MOD_REL)
            wait(4.0)

            node.get_logger().info("힘 제어 해제 및 이번 블록 조립 완료")
            release_force()
            release_compliance_ctrl()
            wait(0.2)

            # 조립 완료 후 상공으로 복귀하고 그리퍼 열기
            movel(pose_place_ready, vel=VELOCITY, acc=ACC)
            wait(0.2)
            open_gripper()
            wait(0.5)

            # [중요] 이 블록을 "배치 완료" 리스트에 등록
            # (다음 블록의 자세 판단에 사용됨)
            placed_blocks.append((grid_x, grid_y, grid_z))
            node.get_logger().info(f"배치 완료 목록 갱신: {placed_blocks}")

        node.get_logger().info("모든 레고 블록 큐 조립이 완료되었습니다!")

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")
    except Exception as e:
        node.get_logger().info(f"Robot Error: {e}")
    finally:
        # 에러가 나거나 전체 루프가 끝나면 안전하게 홈으로 이동 후 종료
        movej(homej, vel=VELOCITY, acc=ACC)

        # 기준 좌표계를 베이스(0)로 복원 (다른 프로그램에 영향 주지 않도록)
        try:
            set_ref_coord(0)
            node.get_logger().info("기준 좌표계를 베이스(0)로 복원")
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
