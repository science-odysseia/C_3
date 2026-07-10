# pick and place in 1 method. from pos1 to pos2 @20241104
import rclpy
import DR_init
import time
import threading # [추정 원인 해결 1] 스레드 추가

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model= ROBOT_MODEL

ON, OFF = 1, 0
CYCLE = 3
CHECK_INTERVAL = 0.1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_grip_simple", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    # [수정] 백그라운드에서 ROS2 서비스를 수신 대기할 스레드 구동
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            wait,
        )
        from DR_common2 import posj
    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    # 초기화
    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")

    def grip():
        node.get_logger().info("set for digital output 1 0 for grip")
        try:
            set_digital_output(1, OFF)
            set_digital_output(2, OFF)
            set_digital_output(1, OFF)
            set_digital_output(2, ON)
        except Exception as e:
            node.get_logger().warn(f"Digital Output Error (Simulator limitation?): {e}")
        wait(1)

    def release():
        node.get_logger().info("set for digital output 0 1 for release")
        try:
            set_digital_output(1, OFF)
            set_digital_output(2, OFF)
            set_digital_output(1, ON)
            set_digital_output(2, OFF)
        except Exception as e:
            node.get_logger().warn(f"Digital Output Error (Simulator limitation?): {e}")
        wait(1)
     
    homej = posj([0, 0, 90, 0, 90, 0])

    # 1. 홈 포지션 이동 테스트
    node.get_logger().info(f"Moving to joint position: {homej}")
    try:
        movej(homej, vel=VELOCITY, acc=ACC)
    except Exception as e:
        node.get_logger().error(f"Initial movej failed: {e}")

    # 2. 그리퍼 루프 테스트
    try:
        for i in range(CYCLE):
            node.get_logger().info(f"=== Cycle {i+1}/{CYCLE} ===")
            grip()
            wait(1)
            release()
            wait(1)

        node.get_logger().info("Gripper Test Complete")

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")
    except Exception as e:
        node.get_logger().error(f"Unexpected Error: {e}")

    finally:
        try:
            movej(homej, vel=VELOCITY, acc=ACC)
        except Exception as e:
            node.get_logger().error(f"Failed to move home: {e}")

        # 종료 처리
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()