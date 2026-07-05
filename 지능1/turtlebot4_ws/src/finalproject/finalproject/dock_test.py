import threading
import tkinter as tk

import rclpy
from std_msgs.msg import Empty

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator


class TurtleBotDockingUI:
    """
    TurtleBot4의 도킹 / 언도킹을 제어하는 간단한 GUI 클래스

    기능:
    1. 현재 로봇이 도킹 상태인지 확인
    2. 상태에 따라 버튼 텍스트를 '도킹' 또는 '언도킹'으로 변경
    3. 버튼 클릭 시 도킹 또는 언도킹 동작 실행
    4. 동작 중에는 버튼을 비활성화하여 중복 실행 방지
    """

    def __init__(self, root):
        """
        GUI 화면을 초기화하고 TurtleBot4 제어 객체를 생성하는 부분

        root:
            Tkinter에서 사용하는 최상위 윈도우 객체
        """

        # Tkinter 메인 윈도우 저장
        self.root = root

        # 창 제목 설정
        self.root.title("TurtleBot4 Docking UI")

        # 창 크기 설정
        self.root.geometry("360x180")

        # TurtleBot4 제어를 위한 Navigator 객체 생성
        # namespace='/robot3'을 지정했기 때문에
        # 이 객체는 /robot3 네임스페이스에 있는 로봇을 제어함
        self.navigator = TurtleBot4Navigator(namespace='/robot3')

        # 도킹 명령을 보낼 ROS2 Publisher 생성
        # 상대 토픽 이름 'dock'을 사용했기 때문에 실제 토픽은 /robot3/dock
        # Empty 메시지는 데이터 내용 없이 "신호"만 보낼 때 사용
        self.dock_pub = self.navigator.create_publisher(Empty, 'dock', 10)

        # 언도킹 명령을 보낼 ROS2 Publisher 생성
        # 실제 토픽은 /robot3/undock
        self.undock_pub = self.navigator.create_publisher(Empty, 'undock', 10)

        # 현재 로봇이 도킹 상태인지 저장하는 변수
        # True  : 도킹 상태
        # False : 언도킹 상태
        # 처음에는 실제 상태를 아직 모르기 때문에 기본값을 False로 설정
        self.is_docked = False

        # 현재 상태를 화면에 표시하는 Label 위젯
        self.status_label = tk.Label(
            root,
            text="현재 상태 확인 중...",
            font=("Arial", 14)
        )

        # Label을 창에 배치
        # pady는 위아래 여백
        self.status_label.pack(pady=30)

        # 도킹과 언도킹을 하나의 버튼으로 처리하는 버튼
        # 현재 상태에 따라 버튼 텍스트가 '도킹' 또는 '언도킹'으로 바뀜
        self.toggle_button = tk.Button(
            root,
            text="상태 확인 중...",
            font=("Arial", 13),
            width=15,
            command=self.on_toggle_button  # 버튼 클릭 시 실행될 함수
        )

        # 버튼을 창에 배치
        self.toggle_button.pack(pady=10)

        # 창의 X 버튼을 눌렀을 때 바로 종료하지 않고
        # ROS2 노드 정리 후 종료하도록 on_close 함수 연결
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # 프로그램 실행 직후 로봇의 현재 도킹 상태를 확인하고 UI에 반영
        self.update_initial_status()

    def set_status(self, text):
        """
        상태 표시 Label의 텍스트를 변경하는 함수

        Tkinter UI는 메인 스레드에서만 안전하게 수정하는 것이 좋다.
        그래서 root.after(0, ...)를 사용해서
        UI 변경 작업을 Tkinter 메인 루프에 맡긴다.
        """

        self.root.after(0, lambda: self.status_label.config(text=text))

    def set_button_state(self, state):
        """
        버튼의 활성화 / 비활성화 상태를 변경하는 함수

        state 값 예시:
            tk.NORMAL   : 버튼 활성화
            tk.DISABLED : 버튼 비활성화
        """

        self.root.after(0, lambda: self.toggle_button.config(state=state))

    def set_button_text(self, text):
        """
        버튼에 표시되는 텍스트를 변경하는 함수

        예:
            '도킹'
            '언도킹'
            '도킹 중...'
            '언도킹 중...'
        """

        self.root.after(0, lambda: self.toggle_button.config(text=text))

    def update_ui_by_status(self):
        """
        현재 도킹 상태에 따라 화면 표시를 갱신하는 함수

        self.is_docked 값에 따라:
            True  -> 현재 상태는 도킹, 버튼은 '언도킹'
            False -> 현재 상태는 언도킹, 버튼은 '도킹'
        """

        if self.is_docked:
            # 로봇이 이미 도킹되어 있으면
            # 다음에 수행할 수 있는 동작은 언도킹
            self.status_label.config(text="현재 상태는 도킹")
            self.toggle_button.config(text="언도킹")

        else:
            # 로봇이 도킹되어 있지 않으면
            # 다음에 수행할 수 있는 동작은 도킹
            self.status_label.config(text="현재 상태는 언도킹")
            self.toggle_button.config(text="도킹")

    def update_initial_status(self):
        """
        프로그램 시작 시 TurtleBot4의 현재 도킹 상태를 확인하는 함수

        navigator.getDockedStatus()를 통해
        현재 로봇이 도킹 상태인지 확인하고,
        그 결과를 UI에 반영한다.
        """

        try:
            # TurtleBot4의 현재 도킹 상태 확인
            self.is_docked = self.navigator.getDockedStatus()

            # 확인된 상태를 화면에 반영
            self.update_ui_by_status()

        except Exception as e:
            # 상태 확인에 실패한 경우
            # 예: 로봇과 연결되지 않았거나 ROS2 통신 문제 발생
            self.status_label.config(text=f"상태 확인 실패: {e}")

            # 상태를 모르면 잘못된 명령이 나갈 수 있으므로 버튼 비활성화
            self.toggle_button.config(text="상태 확인 실패", state=tk.DISABLED)

    def on_toggle_button(self):
        """
        도킹 / 언도킹 버튼을 눌렀을 때 실행되는 함수

        도킹이나 언도킹 동작은 시간이 걸릴 수 있다.
        이 작업을 메인 스레드에서 바로 실행하면 GUI가 멈춘 것처럼 보일 수 있다.

        그래서 별도의 스레드에서 toggle_docking()을 실행한다.
        """

        thread = threading.Thread(target=self.toggle_docking)

        # daemon=True로 설정하면
        # 메인 프로그램 종료 시 이 스레드도 함께 종료될 수 있음
        thread.daemon = True

        # 스레드 시작
        thread.start()

    def toggle_docking(self):
        """
        현재 상태에 따라 도킹 또는 언도킹을 선택해서 실행하는 함수

        self.is_docked 값이 True이면 언도킹,
        False이면 도킹을 수행한다.
        """

        # 동작이 진행되는 동안 버튼을 비활성화하여
        # 사용자가 버튼을 여러 번 누르는 것을 방지
        self.set_button_state(tk.DISABLED)

        try:
            if self.is_docked:
                # 현재 도킹 상태이면 언도킹 수행
                self.undock_robot()
            else:
                # 현재 언도킹 상태이면 도킹 수행
                self.dock_robot()

        finally:
            # 성공 / 실패 여부와 관계없이
            # 작업이 끝나면 버튼을 다시 활성화
            self.set_button_state(tk.NORMAL)

    def dock_robot(self):
        """
        로봇을 도킹시키는 함수

        순서:
        1. /robot3/dock 토픽에 Empty 메시지 발행
        2. UI를 '도킹 중...' 상태로 변경
        3. navigator.dock()으로 실제 도킹 명령 실행
        4. 성공 시 상태를 도킹 완료로 변경
        """

        try:
            # dock 토픽에 Empty 메시지를 발행
            # Empty 메시지는 내용은 없지만 '도킹 요청' 신호 역할을 함
            self.dock_pub.publish(Empty())

            # 사용자에게 현재 동작 상태 표시
            self.set_status("도킹 중...")
            self.set_button_text("도킹 중...")

            # TurtleBot4 Navigator의 도킹 기능 실행
            # 이 함수는 실제 도킹 동작이 끝날 때까지 시간이 걸릴 수 있음
            self.navigator.dock()

            # 도킹이 성공했다고 판단하고 상태 변수 갱신
            self.is_docked = True

            # UI 상태 갱신
            self.set_status("도킹 완료")
            self.set_button_text("언도킹")

        except Exception as e:
            # 도킹 중 예외가 발생한 경우
            # 에러 내용을 화면에 표시
            self.set_status(f"도킹 실패: {e}")

            # 실패했으므로 다시 도킹을 시도할 수 있도록 버튼 텍스트 유지
            self.set_button_text("도킹")

    def undock_robot(self):
        """
        로봇을 언도킹시키는 함수

        순서:
        1. /robot3/undock 토픽에 Empty 메시지 발행
        2. UI를 '언도킹 중...' 상태로 변경
        3. navigator.undock()으로 실제 언도킹 명령 실행
        4. 성공 시 상태를 언도킹 완료로 변경
        """

        try:
            # undock 토픽에 Empty 메시지를 발행
            # Empty 메시지는 내용 없이 '언도킹 요청' 신호만 전달
            self.undock_pub.publish(Empty())

            # 사용자에게 현재 동작 상태 표시
            self.set_status("언도킹 중...")
            self.set_button_text("언도킹 중...")

            # TurtleBot4 Navigator의 언도킹 기능 실행
            self.navigator.undock()

            # 언도킹이 성공했다고 판단하고 상태 변수 갱신
            self.is_docked = False

            # UI 상태 갱신
            self.set_status("언도킹 완료")
            self.set_button_text("도킹")

        except Exception as e:
            # 언도킹 중 예외가 발생한 경우
            self.set_status(f"언도킹 실패: {e}")

            # 실패했으므로 다시 언도킹을 시도할 수 있도록 버튼 텍스트 유지
            self.set_button_text("언도킹")

    def on_close(self):
        """
        GUI 창을 닫을 때 실행되는 함수

        ROS2 노드와 rclpy를 정상적으로 종료하지 않으면
        프로세스가 완전히 종료되지 않거나 리소스가 남을 수 있다.
        """

        try:
            # TurtleBot4 Navigator 노드 제거
            self.navigator.destroy_node()

        except Exception:
            # 종료 과정에서 예외가 발생해도 프로그램 종료는 계속 진행
            pass

        # ROS2 Python 클라이언트 종료
        rclpy.shutdown()

        # Tkinter 창 종료
        self.root.destroy()


def main():
    """
    프로그램의 시작점

    순서:
    1. ROS2 초기화
    2. Tkinter GUI 생성
    3. TurtleBotDockingUI 객체 생성
    4. Tkinter 이벤트 루프 실행
    """

    # ROS2 Python 라이브러리 초기화
    # ROS2 노드, Publisher, Subscriber 등을 사용하기 전에 반드시 필요
    rclpy.init()

    # Tkinter 메인 윈도우 생성
    root = tk.Tk()

    # TurtleBot 도킹 GUI 객체 생성
    app = TurtleBotDockingUI(root)

    # Tkinter 이벤트 루프 시작
    # 이 코드가 실행되면 창이 유지되고 버튼 클릭 같은 이벤트를 처리함
    root.mainloop()


# 이 파일을 직접 실행했을 때만 main() 실행
# 다른 파일에서 import할 경우에는 main()이 자동 실행되지 않음
if __name__ == '__main__':
    main()