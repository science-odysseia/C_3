#!/usr/bin/env python3

import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# 예약 간격을 입력받는 토픽 이름
# 예: "00:10" 이 들어오면 10분 뒤 예약 시간을 계산한다.
RESERVE_INTERVAL_TOPIC_NAME = '/robot3/reserve'

# 계산된 실제 예약 시간을 발행하는 토픽 이름
# 예: 현재 시간이 13:00이고 "00:10"을 받으면 "2026-06-26 13:10 KST" 형태로 발행한다.
RESERVE_TIME_TOPIC_NAME = '/robot3/reserve_time'


def get_time_text_from_timestamp(timestamp):
    """
    timestamp 값을 사람이 읽기 쉬운 날짜/시간 문자열로 변환한다.

    time.time()으로 얻은 timestamp는 초 단위 숫자이므로,
    datetime으로 변환한 뒤 현재 시스템의 시간대 정보를 반영한다.
    """
    return datetime.fromtimestamp(
        timestamp
    ).astimezone().strftime('%Y-%m-%d %H:%M %Z')


class ReserveTimeCalculatorNode(Node):
    """
    예약 간격을 받아서 실제 예약 시간을 계산하는 ROS2 노드.

    동작 흐름:
    1. /robot3/reserve 토픽에서 예약 간격 문자열을 구독한다.
       예: "00:01", "01:30"

    2. 수신한 문자열을 시(hour), 분(minute) 단위로 파싱한다.

    3. 현재 시간에 예약 간격을 더해서 실제 예약 시간을 계산한다.

    4. 계산된 예약 시간을 /robot3/reserve_time 토픽으로 발행한다.
    """

    def __init__(self):
        """
        노드 초기화 함수.

        구독자와 발행자를 생성하고,
        어떤 토픽을 구독/발행하는지 로그로 출력한다.
        """
        super().__init__('reserve_time_calculator_node')

        # 예약 간격을 받기 위한 구독자 생성
        # /robot3/reserve 토픽에 String 메시지가 들어오면 reserve_callback 함수가 실행된다.
        self.reserve_subscriber = self.create_subscription(
            String,
            RESERVE_INTERVAL_TOPIC_NAME,
            self.reserve_callback,
            10
        )

        # 계산된 실제 예약 시간을 발행하기 위한 발행자 생성
        self.reserve_time_publisher = self.create_publisher(
            String,
            RESERVE_TIME_TOPIC_NAME,
            10
        )

        # 노드 실행 시 어떤 토픽을 구독하는지 확인하기 위한 로그
        self.get_logger().info(
            f'Subscribe: {RESERVE_INTERVAL_TOPIC_NAME}'
        )

        # 노드 실행 시 어떤 토픽으로 발행하는지 확인하기 위한 로그
        self.get_logger().info(
            f'Publish: {RESERVE_TIME_TOPIC_NAME}'
        )

    def reserve_callback(self, msg):
        """
        예약 간격 메시지를 수신했을 때 실행되는 콜백 함수.

        입력 메시지 예:
        - "00:01" → 1분 뒤 예약
        - "01:30" → 1시간 30분 뒤 예약

        처리 순서:
        1. 메시지 문자열 양쪽 공백 제거
        2. HH:MM 형식인지 검사 및 파싱
        3. 예약 간격을 초 단위로 변환
        4. 현재 시간에 예약 간격을 더해 예약 시각 계산
        5. 계산된 예약 시각을 토픽으로 발행
        """
        # 수신한 문자열에서 앞뒤 공백 제거
        reserve_text = msg.data.strip()

        try:
            # "HH:MM" 형태의 문자열을 시간, 분 정수값으로 변환
            interval_hours, interval_minutes = self.parse_reserve_text(
                reserve_text
            )

        except ValueError:
            # 형식이 잘못된 경우 에러 로그를 출력하고 함수 종료
            self.get_logger().error(
                f'Invalid reserve data: "{reserve_text}" '
                f'expected format is HH:MM, example: 00:01'
            )
            return

        # 예약 간격을 초 단위로 변환
        # 예: 01:30 → (1 * 60 + 30) * 60 = 5400초
        interval_sec = (interval_hours * 60 + interval_minutes) * 60

        # 00:00처럼 예약 간격이 0인 경우는 의미가 없으므로 에러 처리
        if interval_sec <= 0:
            self.get_logger().error(
                'Reserve interval must be greater than 0 minutes'
            )
            return

        # 현재 시간을 timestamp 값으로 가져온다.
        now_timestamp = time.time()

        # 현재 시간에 예약 간격을 더해서 다음 예약 시각을 계산한다.
        next_publish_timestamp = now_timestamp + interval_sec

        # 계산된 timestamp를 사람이 읽기 쉬운 문자열로 변환한다.
        next_publish_time_text = get_time_text_from_timestamp(
            next_publish_timestamp
        )

        # 발행할 ROS2 String 메시지 생성
        publish_msg = String()
        publish_msg.data = next_publish_time_text

        # 계산된 예약 시간을 /robot3/reserve_time 토픽으로 발행
        self.reserve_time_publisher.publish(publish_msg)

        # 수신한 예약 간격을 로그로 출력
        self.get_logger().info(
            f'Received interval: {interval_hours:02d}:{interval_minutes:02d}'
        )

        # 발행한 실제 예약 시간을 로그로 출력
        self.get_logger().info(
            f'Published next reserve time: {publish_msg.data}'
        )

    def parse_reserve_text(self, reserve_text):
        """
        예약 간격 문자열을 검증하고 시간, 분 값으로 변환한다.

        입력 형식:
        - "HH:MM"

        정상 예:
        - "00:01"
        - "01:30"
        - "23:59"

        오류 예:
        - "1"
        - "01-30"
        - "24:00"
        - "00:60"
        - "abc:def"

        반환값:
        - interval_hours: int
        - interval_minutes: int

        형식이 잘못되면 ValueError를 발생시킨다.
        """
        # ":" 기준으로 문자열을 나눈다.
        # 정상 입력이라면 ["HH", "MM"] 형태가 되어야 한다.
        split_text = reserve_text.split(':')

        # "HH:MM" 형태가 아니면 오류 처리
        if len(split_text) != 2:
            raise ValueError

        # 문자열로 들어온 시간과 분을 정수로 변환
        # int 변환이 불가능한 문자열이면 ValueError가 발생한다.
        interval_hours = int(split_text[0])
        interval_minutes = int(split_text[1])

        # 시간 값은 0~23 사이만 허용
        if not (0 <= interval_hours <= 23):
            raise ValueError

        # 분 값은 0~59 사이만 허용
        if not (0 <= interval_minutes <= 59):
            raise ValueError

        # 검증이 끝난 시간, 분 값을 반환
        return interval_hours, interval_minutes


def main(args=None):
    """
    ROS2 노드를 실행하는 메인 함수.

    실행 흐름:
    1. rclpy 초기화
    2. ReserveTimeCalculatorNode 객체 생성
    3. rclpy.spin()으로 콜백 대기
    4. 종료 시 노드 정리 및 rclpy 종료
    """
    # ROS2 Python 클라이언트 라이브러리 초기화
    rclpy.init(args=args)

    # 예약 시간 계산 노드 생성
    node = ReserveTimeCalculatorNode()

    try:
        # 노드를 계속 실행하면서 토픽 메시지가 들어오기를 기다린다.
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Ctrl+C로 종료해도 에러 메시지가 출력되지 않도록 처리
        pass

    finally:
        # 노드와 ROS2 리소스를 안전하게 정리
        node.destroy_node()
        rclpy.shutdown()


# 이 파일을 직접 실행했을 때만 main() 함수 실행
if __name__ == '__main__':
    main()