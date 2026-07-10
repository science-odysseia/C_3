import rclpy
import DR_init
from copy import deepcopy

# =========================================================================
# [1. 로봇 및 환경 설정 환경 상수]
# =========================================================================
ROBOT_ID = "dsr01"      # 제어할 두산 로봇의 고유 ID (네임스페이스)
ROBOT_MODEL = "m0609"   # 로봇 모델명 (가반하중 6kg, 작업반경 900mm)
VELOCITY, ACC = 40, 40  # 로봇 모션의 기본 속도 및 가속도 설정 (안전을 위해 40으로 제한)

# 두산 로봇 초기화 라이브러리에 로봇 정보 주입
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 그리퍼의 현재 상태를 저장하는 전역 변수 (0: 열림, 1: 닫힘)
gripper_status = 0


def main(args=None):
    # ROS2 초기화 및 노드 생성
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)

    # 두산 로봇 API가 인식할 수 있도록 현재 ROS2 노드를 주입
    DR_init.__dsr__node = node

    # =========================================================================
    # [2. 두산 로봇 전용 라이브러리 (API) 임포트]
    # =========================================================================
    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            set_digital_output,
            get_digital_input,
            wait,
            OFF,
            ON,
            task_compliance_ctrl,
            set_desired_force,
            release_force,
            release_compliance_ctrl,
            DR_FC_MOD_REL
        )
        from DR_common2 import posx, posj

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    # =========================================================================
    # [3. 하드웨어 제어 서브 루틴 (그리퍼 함수)]
    # =========================================================================
    def open_gripper():
        """ 디지털 출력을 제어하여 공압/전기 그리퍼를 완전히 여는 함수 """
        global gripper_status
        print("그리퍼 열기")
        
        set_digital_output(2, OFF)  # 닫기 밸브 OFF
        set_digital_output(1, ON)   # 열기 밸브 ON
        wait(1.0)                   # 그리퍼가 물리적으로 열릴 때까지 1초 대기

        gripper_status = 0          # 상태 갱신

    def close_gripper():
        """ 디지털 출력을 제어하여 공압/전기 그리퍼를 완전히 닫는 함수 """
        global gripper_status
        print("그리퍼 닫기")
        
        set_digital_output(1, OFF)  # 열기 밸브 OFF
        set_digital_output(2, ON)   # 닫기 밸브 ON
        wait(1.0)                   # 그리퍼가 물리적으로 물체를 잡을 때까지 1초 대기

        gripper_status = 1          # 상태 갱신
    
    # =========================================================================
    # [4. 공정 초기화 및 좌표 정의]
    # =========================================================================
    set_tool("Tool Weight_1")  # 티칭 펜던트에 등록된 툴 무게 프로파일 적용
    set_tcp("Tooltcp_v1")      # 그리퍼 끝단 중심점(Tool Center Point) 설정

    # 로봇 기본 안전 자세 (각 관절 각도 기반: Joint Position)
    homej = posj([0.0, 0.0, 90.0, 0.0, 00.0, 0.0])
    
    # 최종 레고 조립 목표 위치 (작업 공간 좌표계 기반: Task Position)
    posx1 = posx([368.0, 9.0, 8.0, 164.9, 179.48, 164.84])

    # =========================================================================
    # [5. 1단계: 홈 이동 및 최초 집기 단계 (13번 버튼)]
    # =========================================================================
    # 안전 구역인 홈 위치로 이동 후 그리퍼를 열어 준비
    movej(homej, vel=VELOCITY, acc=ACC)
    open_gripper()
    
    # 13번 디지털 입력(시작 스위치)이 켜질 때까지 가만히 대기하는 루프
    while rclpy.ok():
        if get_digital_input(13) == True:
            node.get_logger().info("13번 버튼 입력 확인! 그리퍼를 닫고 동작을 시작합니다.")
            close_gripper() # 사람이 공급해 준 레고 블록을 집음
            break           # 대기 루프 탈출 후 조립 공간으로 출발
        
        wait(0.1)  # CPU 과점유 방지 및 ROS2 통신 주기 확보

    # =========================================================================
    # [6. 2단계: 조립 영역 진입 및 가안착 단계 (14번 버튼)]
    # =========================================================================
    try:
        node.get_logger().info(f"Moving to task position: {posx1}")
        
        # 충돌 방지를 위해 조립 목표 위치의 30mm 상공 대기 좌표(Ready) 계산
        pose_press_ready = deepcopy(posx1)
        pose_press_ready[2] += 30
        
        # 조립 상공 대기 위치(Ready)를 거쳐 실제 조립 위치(posx1)로 정밀 직선 하강
        movel(pose_press_ready, vel=VELOCITY, acc=ACC)
        movel(posx1, vel=VELOCITY, acc=ACC)
        
        # 14번 디지털 입력(가안착 및 누르기 승인 스위치) 대기 루프
        # 로봇이 레고 판 위에 블록을 살짝 얹어놓은 상태에서 사람이 최종 확인하는 구간
        while rclpy.ok():
            if get_digital_input(14) == True:
                node.get_logger().info("14번 버튼 입력 확인! 조립 시퀀스를 시작합니다.")
                break  # 대기 루프 탈출
            
            wait(0.1)

        # 그리퍼를 열어 레고 블록을 조립 판 위에 가안착 (살짝 놓기)
        open_gripper()
        wait(0.5)
        
        # 그리퍼 손가락이 레고 블록 옆면에 걸리지 않도록 잠시 상공 대기 위치로 도피
        movel(pose_press_ready, vel=VELOCITY, acc=ACC)

        # =========================================================================
        # [7. 3단계: 힘 제어(Force Control) 기반 결합 누르기 단계]
        # =========================================================================
        # 그리퍼를 다시 닫음 
        # (※ 이 때 그리퍼 손가락 끝단 바닥면으로 블록의 윗면(대가리)을 누르는 덮개 역할을 하게 됨)
        close_gripper()
        wait(0.5)

        node.get_logger().info("힘 제어 활성화: 블록 누르기 시작")
        
        # 제어 파라미터 설정: Z축 아래 방향(-)으로 25N(약 2.5kg)의 힘으로 지속해서 누름
        fd = [0, 0, -25, 0, 0, 0]    
        fc_dir = [0, 0, 1, 0, 0, 0]  # Z축 방향에 대해서만 힘 제어 활성화
        
        # 태스크 컴플라이언스 제어 ON (로봇의 손목 관절을 유연하게 만들어 충돌 손상 방지)
        # 스프링처럼 유연해진 상태에서 기계적 유격(돌기 구멍)을 찾아 들어감
        task_compliance_ctrl([20000, 20000, 20000, 200, 200, 200], 0)
        wait(0.2)
        
        # 설정한 목표 힘(-25N) 인가 (레고가 "딸깍" 소리를 내며 완전히 결합되도록 유도)
        set_desired_force(fd, dir=fc_dir, mod=DR_FC_MOD_REL)
        wait(6.0)  # 결합이 확실하게 유지되도록 6초간 꾹 눌러줌

        # 결합 완료 후 힘 제어 해제 및 로봇 제어 모드를 일반 위치 제어 모드로 복구
        node.get_logger().info("힘 제어 해제 및 조립 완료")
        release_force()
        release_compliance_ctrl()
        wait(0.2)

        # =========================================================================
        # [8. 4단계: 공정 마감 및 안전 상공 퇴출]
        # =========================================================================
        # 조립이 끝났으므로 툴을 상공 대기 위치로 안전하게 들어 올림
        movel(pose_press_ready, vel=VELOCITY, acc=ACC)
        
        # 다음 블록 작업을 위해 그리퍼를 완전히 열어 둠
        open_gripper()
        wait(0.5)
    
    # 예외 처리 및 안전 종료 구역
    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")

    except Exception as e:
        node.get_logger().info(f"Robot Error: {e}")

    finally:
        # 에러가 나거나 공정이 끝나도 언제나 안전하게 홈 위치로 복귀
        movej(homej, vel=VELOCITY, acc=ACC)
        
        # ROS2 노드 파괴 및 통신 종료
        node.destroy_node()
        rclpy.shutdown()      


if __name__ == "__main__":
    main()