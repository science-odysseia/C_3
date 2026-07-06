import rclpy
import DR_init
from copy import deepcopy

# =========================================================================
# [1. 로봇 및 환경 설정 환경 상수]
# =========================================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 40, 40

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

gripper_status = 0


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # =========================================================================
    # [2. API 임포트]
    # =========================================================================
    try:
        from DSR_ROBOT2 import (
            set_tool, set_tcp, movej, movel, set_digital_output,
            get_digital_input, wait, OFF, ON, task_compliance_ctrl,
            set_desired_force, release_force, release_compliance_ctrl, DR_FC_MOD_REL
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    # =========================================================================
    # [3. 그리퍼 서브 루틴]
    # =========================================================================
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
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1.0)
        gripper_status = 1
    
    # =========================================================================
    # [4. 공정 초기화 및 레고 규격 정의]
    # =========================================================================
    set_tool("Tool Weight_1")
    set_tcp("Tooltcp_v1")

    homej = posj([0.0, 0.0, 90.0, 0.0, 00.0, 0.0])
    
    # [중요] 레고 판의 기준이 되는 (0, 0, 0) 절대 좌표 (티칭 포인트)
    BASE_POSE_X = 368.0
    BASE_POSE_Y = 9.0
    BASE_POSE_Z = 8.0
    
    # 고정된 레고 조립 회전 각도 (수직 자세)
    FIXED_A, FIXED_B, FIXED_C = 164.9, 179.48, 164.84

    # 알려주신 레고 블록 격자당 실제 물리적 크기 (mm)
    LEGO_W_X = 14.0
    LEGO_W_Y = 14.0
    LEGO_W_Z = 24.0

    # =========================================================================
    # [5. 큐(Queue) 데이터 준비 및 Z축 정렬]
    # =========================================================================
    # 외부에서 들어올 데이터 예시 (순서가 뒤죽박죽 섞여있다고 가정)
    incoming_blocks = [
        [2, [0, 0, 1]],  # 2층 블록 (Z=1)
        [2, [2, 0, 0]],  # 1층 블록 (Z=0)
        [2, [0, 0, 0]],  # 1층 블록 (Z=0)
        [2, [2, 0, 1]]   # 3층 블록 (Z=2)
    ]

    # [핵심 로직] 각 요소의 block[1][2], 즉 z 좌표를 기준으로 오름차순 정렬 (큐 생성)
    # 정렬 결과: [0, 0, 0] -> [1, 0, 0] -> [0, 0, 1] -> [0, 1, 2] 순으로 바뀜
    lego_queue = sorted(incoming_blocks, key=lambda block: block[1][2])
    node.get_logger().info(f"정렬된 레고 조립 큐 목록: {lego_queue}")

    # =========================================================================
    # [6. 메인 공정 시작]
    # =========================================================================
    # 로봇을 홈으로 보내고 그리퍼를 열어 준비
    movej(homej, vel=VELOCITY, acc=ACC)
    open_gripper()

    try:
        # 정렬된 큐에서 블록을 하나씩 꺼내어 순차 조립 수행
        for current_block in lego_queue:
            block_id = current_block[0]
            grid_x, grid_y, grid_z = current_block[1]
            
            node.get_logger().info(f"\n현재 조립 중인 블록: ID={block_id}, 격자위치=({grid_x}, {grid_y}, {grid_z})")

            # --- [수식 치환] 격자 좌표를 mm 단위 로봇 좌표로 자동 변환 ---
            calc_x = BASE_POSE_X + (grid_x * LEGO_W_X)
            calc_y = BASE_POSE_Y + (grid_y * LEGO_W_Y)
            calc_z = BASE_POSE_Z + (grid_z * LEGO_W_Z)
            
            # 최종 타겟 좌표 생성
            posx1 = posx([calc_x, calc_y, calc_z, FIXED_A, FIXED_B, FIXED_C])

            # -----------------------------------------------------------------
            # 13번 버튼 대기 (사람이 로봇에게 새 블록을 쥐여주고 승인 버튼을 누르는 구간)
            # -----------------------------------------------------------------
            node.get_logger().info("새 블록을 장착한 후 13번 버튼을 누르세요...")
            while rclpy.ok():
                if get_digital_input(13) == True:
                    close_gripper() # 블록 집기
                    break
                wait(0.1)

            # -----------------------------------------------------------------
            # 조립 위치 진입 및 14번 확인 버튼 대기
            # -----------------------------------------------------------------
            pose_press_ready = deepcopy(posx1)
            pose_press_ready[2] += 30  # 30mm 상공 대기 위치 계산
            
            movel(pose_press_ready, vel=VELOCITY, acc=ACC) # 상공 이동
            movel(posx1, vel=VELOCITY, acc=ACC)            # 가안착 위치 하강
            
            node.get_logger().info("가안착 상태 확인 후 14번 버튼을 누르세요...")
            while rclpy.ok():
                if get_digital_input(14) == True:
                    break
                wait(0.1)

            # -----------------------------------------------------------------
            # 힘 제어 기반 누르기 동작
            # -----------------------------------------------------------------
            open_gripper() # 살짝 놓기
            wait(0.5)
            movel(pose_press_ready, vel=VELOCITY, acc=ACC) # 위로 대기

            close_gripper() # 누르기 전용 그리퍼 오므리기
            wait(0.5)

            node.get_logger().info("힘 제어 활성화: 블록 누르기 시작")
            fd = [0, 0, -25, 0, 0, 0]
            fc_dir = [0, 0, 1, 0, 0, 0]
            
            task_compliance_ctrl([20000, 20000, 20000, 200, 200, 200], 0)
            wait(0.2)
            
            set_desired_force(fd, dir=fc_dir, mod=DR_FC_MOD_REL)
            wait(4.0)  # 쌓기 동작이 많으므로 효율성을 위해 누르는 시간을 6초에서 4초로 약간 단축

            node.get_logger().info("힘 제어 해제 및 이번 블록 조립 완료")
            release_force()
            release_compliance_ctrl()
            wait(0.2)

            # 조립 완료 후 상공으로 복귀하고 그리퍼 열기
            movel(pose_press_ready, vel=VELOCITY, acc=ACC)
            wait(0.2)
            movej(homej, vel=VELOCITY, acc=ACC)
            open_gripper()
            wait(0.5)

        node.get_logger().info("🎉 모든 레고 블록 큐 조립이 완료되었습니다! 🎉")

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")
    except Exception as e:
        node.get_logger().info(f"Robot Error: {e}")
    finally:
        # 에러가 나거나 전체 루프가 끝나면 안전하게 홈으로 이동 후 종료
        movej(homej, vel=VELOCITY, acc=ACC)
        node.destroy_node()
        rclpy.shutdown()      


if __name__ == "__main__":
    main()
