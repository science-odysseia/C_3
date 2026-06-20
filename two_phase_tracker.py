import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.action import ActionClient

from sensor_msgs.msg import Image as ROSImage, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

from cv_bridge import CvBridge
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from ultralytics import YOLO

import numpy as np
import cv2
import threading
import math
import time

from rclpy.time import Time
from message_filters import Subscriber, ApproximateTimeSynchronizer

# ──────────────────────────────────────────────────────────────
# 동작 단계 정의
#
# PHASE 1: 외부 웹캠으로 mycar 탐지 → AMR이 해당 위치로 이동
# PHASE 2: AMR 탑재 OAK-D 카메라로 전환 → mycar 실시간 추적
# ──────────────────────────────────────────────────────────────
PHASE_WEBCAM  = 1   # 웹캠으로 최초 탐지 및 접근
PHASE_OAKD    = 2   # OAK-D로 전환하여 실시간 추적


class TwoPhaseTracker(Node):
    def __init__(self):
        super().__init__('two_phase_tracker_node')

        self.bridge = CvBridge()
        self.K      = None
        self.lock   = threading.Lock()

        ns = self.get_namespace()

        # ── 토픽 이름 ──────────────────────────────────────────
        # 웹캠 (Phase 1용)
        self.webcam_topic = '/camera/image_raw'

        # OAK-D (Phase 2용)
        self.oakd_rgb_topic   = f'{ns}/oakd/rgb/image_raw/compressed'
        self.oakd_depth_topic = '/robot3/oakd/stereo/image_raw/compressedDepth'
        self.oakd_info_topic  = f'{ns}/oakd/rgb/camera_info'

        # ── 이미지 버퍼 ────────────────────────────────────────
        self.rgb_image    = None   # 현재 단계에서 사용하는 RGB
        self.depth_image  = None   # OAK-D Depth (Phase 2에서 사용)
        self.camera_frame = None

        # ── 단계 상태 ──────────────────────────────────────────
        self.phase = PHASE_WEBCAM
        self.get_logger().info('=== PHASE 1 시작: 웹캠으로 mycar 탐지 ===')

        # ── YOLO 모델 ──────────────────────────────────────────
        self.model = YOLO('/home/yswbulb/turtlebot4_ws/best.pt')
        self.get_logger().info('YOLO model loaded')

        # ── Nav2 비동기 Action ─────────────────────────────────
        self.action_client            = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle              = None
        self.latest_map_point         = None

        # 근접 잠금 (Phase 1 → 2 전환 트리거)
        self.close_enough_distance    = 2.0
        self.close_distance_hit_count = 0
        self.block_goal_updates       = False
        self.last_feedback_log_time   = 0.0

        # ── TF2 ────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── TurtleBot4 초기화 ──────────────────────────────────
        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        self.logged_intrinsics  = False
        self.logged_rgb_shape   = False
        self.logged_depth_shape = False

        # rqt 디버깅용 퍼블리셔
        self.rgb_pub   = self.create_publisher(ROSImage, f'{ns}/rgb_processed', 1)
        self.depth_pub = self.create_publisher(ROSImage, f'{ns}/depth_colored',  1)

        # ── Phase 1: 웹캠 구독 ─────────────────────────────────
        self.webcam_sub = self.create_subscription(
            ROSImage,
            self.webcam_topic,
            self.webcam_callback,
            10
        )
        self.get_logger().info(f'Subscribed to webcam: {self.webcam_topic}')

        # ── Phase 2용 구독자 (나중에 활성화) ─────────────────
        # OAK-D 카메라 정보는 미리 구독 (K값 준비)
        self.create_subscription(CameraInfo, self.oakd_info_topic,
                                 self.camera_info_callback, 1)

        self.oakd_rgb_sub   = None   # Phase 2 전환 시 생성
        self.oakd_depth_sub = None
        self.ts             = None

        # ── GUI 스레드 ─────────────────────────────────────────
        self.gui_thread_stop = threading.Event()
        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

        # ── TF 안정화 후 탐지 루프 시작 ───────────────────────
        self.get_logger().info('TF Tree stabilization: 5초 대기...')
        self.detect_timer = None
        self.start_timer  = self.create_timer(5.0, self.start_detection)

    # ══════════════════════════════════════════════════════════
    # 초기화 완료 → 탐지 타이머 시작
    # ══════════════════════════════════════════════════════════
    def start_detection(self):
        self.get_logger().info('TF Tree 안정화 완료. 탐지 루프 시작.')
        self.detect_timer = self.create_timer(0.5, self.detection_loop)
        self.start_timer.cancel()

    # ══════════════════════════════════════════════════════════
    # 카메라 내부 파라미터 수신 (OAK-D, Phase 2용으로 미리 받아둠)
    # ══════════════════════════════════════════════════════════
    def camera_info_callback(self, msg):
        with self.lock:
            self.K = np.array(msg.k).reshape(3, 3)
            if not self.logged_intrinsics:
                self.get_logger().info(
                    f'Camera intrinsics: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, '
                    f'cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}'
                )
                self.logged_intrinsics = True

    # ══════════════════════════════════════════════════════════
    # Phase 1: 웹캠 이미지 수신 콜백
    # ══════════════════════════════════════════════════════════
    def webcam_callback(self, msg):
        if self.phase != PHASE_WEBCAM:
            return   # Phase 2에서는 무시
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame is not None and frame.size > 0:
                if not self.logged_rgb_shape:
                    self.get_logger().info(f'Webcam image shape: {frame.shape}')
                    self.logged_rgb_shape = True
                with self.lock:
                    self.rgb_image = frame
        except Exception as e:
            self.get_logger().error(f'Webcam callback failed: {e}')

    # ══════════════════════════════════════════════════════════
    # Phase 2: OAK-D RGB + compressedDepth 동기화 수신 콜백
    # compressedDepth 포맷: 앞 12바이트 헤더 제거 후 cv2.imdecode
    # ══════════════════════════════════════════════════════════
    def synced_oakd_callback(self, rgb_msg, depth_msg):
        if self.phase != PHASE_OAKD:
            return
        try:
            with self.lock:
                # RGB 디코딩
                np_arr = np.frombuffer(rgb_msg.data, np.uint8)
                rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if rgb is not None and rgb.size > 0:
                    self.rgb_image = rgb

                # compressedDepth 디코딩: 앞 12바이트 헤더 제거
                if len(depth_msg.data) <= 12:
                    self.get_logger().warn('compressedDepth 데이터가 너무 짧습니다.')
                    return
                depth_arr = np.frombuffer(depth_msg.data[12:], np.uint8)
                depth = cv2.imdecode(depth_arr, cv2.IMREAD_UNCHANGED)
                if depth is not None and depth.size > 0:
                    self.depth_image  = depth
                    self.camera_frame = depth_msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f'OAK-D synced callback failed: {e}')

    # ══════════════════════════════════════════════════════════
    # Phase 1 → Phase 2 전환
    # ══════════════════════════════════════════════════════════
    def switch_to_phase2(self):
        self.get_logger().info('=== PHASE 2 전환: OAK-D로 실시간 추적 시작 ===')
        self.phase = PHASE_OAKD

        # 근접 잠금 해제 (Phase 2에서는 계속 추적)
        self.block_goal_updates       = False
        self.close_distance_hit_count = 0
        self.logged_rgb_shape         = False
        self.logged_depth_shape       = False

        # OAK-D RGB + compressedDepth ApproximateTimeSynchronizer 구독 시작
        # compressedDepth 토픽은 CompressedImage 타입
        self.oakd_rgb_sub   = Subscriber(self, CompressedImage, self.oakd_rgb_topic)
        self.oakd_depth_sub = Subscriber(self, CompressedImage, self.oakd_depth_topic)
        self.ts = ApproximateTimeSynchronizer(
            [self.oakd_rgb_sub, self.oakd_depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synced_oakd_callback)
        self.get_logger().info(
            f'OAK-D 구독 시작: {self.oakd_rgb_topic}, {self.oakd_depth_topic}'
        )

    # ══════════════════════════════════════════════════════════
    # 핵심 탐지 루프 (0.5초 주기, Phase 공통)
    # ══════════════════════════════════════════════════════════
    def detection_loop(self):
        with self.lock:
            rgb      = self.rgb_image.copy()   if self.rgb_image   is not None else None
            depth    = self.depth_image.copy() if self.depth_image is not None else None
            K        = self.K.copy()           if self.K           is not None else None
            frame_id = self.camera_frame

        if rgb is None:
            return

        # Phase 1에서는 Depth 없이 YOLO만 돌림
        # Phase 2에서는 Depth 필수
        if self.phase == PHASE_OAKD and (depth is None or K is None or frame_id is None):
            self.get_logger().warn('Phase 2: OAK-D Depth 아직 미수신, 대기 중...')
            return

        # ── YOLO 추론 ─────────────────────────────────────────
        results = self.model(rgb, verbose=False, conf=0.8)[0]
        frame   = rgb.copy()

        depth_colored = None
        if depth is not None:
            depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_colored    = cv2.applyColorMap(
                depth_normalized.astype(np.uint8), cv2.COLORMAP_JET
            )

        detected = False
        for det in results.boxes:
            cls   = int(det.cls[0])
            label = self.model.names[cls]
            conf  = float(det.conf[0])
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if label.lower() != 'mycar':
                continue

            detected = True
            u = (x1 + x2) // 2
            v = (y1 + y2) // 2
            cv2.circle(frame, (u, v), 6, (0, 0, 255), -1)

            # ── Phase 1: Depth 없이 bbox 중심만으로 접근 ─────
            if self.phase == PHASE_WEBCAM:
                # 웹캠은 Depth가 없으므로 고정 접근 거리로 목표 전송
                # → OAK-D의 Depth가 없는 대신, 탐지만 확인하고
                #   AMR이 "웹캠 화면 기준 중앙 방향"으로 이동하도록
                #   map 좌표는 OAK-D camera_info 기반으로 추정 불가.
                #   따라서 Phase 1에서는 이미 이전에 계산된 최신 map 좌표를
                #   유지하거나, OAK-D의 Depth도 함께 받아야 함.
                #
                # ★ 실용적 해결책:
                #   웹캠 탐지 확인 후 OAK-D Depth를 같이 구독하여
                #   bbox 중심 픽셀을 OAK-D Depth에 매핑해 좌표 계산.
                #   (웹캠과 OAK-D가 동일 방향을 향한다고 가정)
                #
                # Phase 1에서도 OAK-D Depth를 함께 참조
                if depth is not None and K is not None and frame_id is not None:
                    map_point = self.pixel_to_map(u, v, depth, K, frame_id)
                    if map_point is not None:
                        self.latest_map_point = map_point
                        phase_label = '[Phase 1 - 웹캠 탐지]'
                        self.get_logger().info(
                            f'{phase_label} mycar at map: '
                            f'({map_point.point.x:.2f}, {map_point.point.y:.2f})'
                        )
                        if self.goal_handle is not None:
                            self.goal_handle.cancel_goal_async()
                        self.send_goal_async(map_point)
                else:
                    self.get_logger().warn(
                        'Phase 1: OAK-D Depth 미수신. '
                        'oakd/stereo/image_raw 토픽 확인 필요.'
                    )
                break

            # ── Phase 2: OAK-D Depth로 실시간 추적 ──────────
            elif self.phase == PHASE_OAKD:
                if v >= depth.shape[0] or u >= depth.shape[1]:
                    continue

                map_point = self.pixel_to_map(u, v, depth, K, frame_id)
                if map_point is None:
                    continue

                self.latest_map_point = map_point
                self.get_logger().info(
                    f'[Phase 2 - OAK-D 추적] mycar at map: '
                    f'({map_point.point.x:.2f}, {map_point.point.y:.2f})'
                )

                # 근접 잠금 확인 (Phase 2에서는 잠금 없이 계속 추적)
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.send_goal_async(map_point)

                if depth_colored is not None:
                    z = float(depth[v, u]) / 1000.0
                    cv2.putText(depth_colored, f'{z:.2f}m', (u, v),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                break

        if not detected:
            self.get_logger().info(
                f'[Phase {"1-웹캠" if self.phase == PHASE_WEBCAM else "2-OAK-D"}] '
                'mycar 미탐지 → 마지막 목표 유지'
            )

        # ── rqt 퍼블리시 ──────────────────────────────────────
        try:
            rgb_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            rgb_msg.header.stamp = self.get_clock().now().to_msg()
            self.rgb_pub.publish(rgb_msg)

            if depth_colored is not None:
                depth_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding='bgr8')
                depth_msg.header.stamp = self.get_clock().now().to_msg()
                self.depth_pub.publish(depth_msg)
        except Exception as e:
            self.get_logger().warn(f'Publish error: {e}')

    # ══════════════════════════════════════════════════════════
    # 픽셀 → 3D → map 좌표 변환 (공통 유틸)
    # ══════════════════════════════════════════════════════════
    def pixel_to_map(self, u, v, depth, K, frame_id):
        z = float(depth[v, u]) / 1000.0   # mm → m
        if not (0.2 < z < 5.0):
            self.get_logger().warn(f'Invalid depth at ({u},{v}): {z:.2f}m')
            return None

        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        Z = z

        pt_camera = PointStamped()
        pt_camera.header.stamp    = Time().to_msg()
        pt_camera.header.frame_id = frame_id
        pt_camera.point.x, pt_camera.point.y, pt_camera.point.z = X, Y, Z

        try:
            pt_map = self.tf_buffer.transform(
                pt_camera, 'map', timeout=Duration(seconds=1.0)
            )
            return pt_map
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return None

    # ══════════════════════════════════════════════════════════
    # 비동기 Goal 전송
    # ══════════════════════════════════════════════════════════
    def send_goal_async(self, pt_map):
        pose = PoseStamped()
        pose.header.frame_id    = 'map'
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.pose.position.x    = pt_map.point.x
        pose.pose.position.y    = pt_map.point.y
        pose.pose.position.z    = 0.0
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f'Goal 전송 → ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    # ══════════════════════════════════════════════════════════
    # Nav2 Action 콜백
    # ══════════════════════════════════════════════════════════
    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')
        self.goal_handle.get_result_async().add_done_callback(
            self.goal_result_callback
        )

    def goal_result_callback(self, future):
        self.get_logger().info(f'Goal finished. Status: {future.result().status}')
        self.goal_handle = None

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining

        # ── Phase 1 전용: 2m 이내 3회 연속 → Phase 2 전환 ──
        if self.phase == PHASE_WEBCAM:
            if distance < self.close_enough_distance:
                self.close_distance_hit_count += 1
            else:
                self.close_distance_hit_count = 0

            if self.close_distance_hit_count >= 3:
                self.switch_to_phase2()
                return

        now = time.time()
        if now - self.last_feedback_log_time > 1.0:
            phase_label = 'Phase 1' if self.phase == PHASE_WEBCAM else 'Phase 2'
            self.get_logger().info(
                f'[{phase_label}] Distance remaining: {distance:.2f}m'
            )
            self.last_feedback_log_time = now

    # ══════════════════════════════════════════════════════════
    # GUI 스레드
    # ══════════════════════════════════════════════════════════
    def gui_loop(self):
        cv2.namedWindow('Two-Phase Tracker', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Two-Phase Tracker', 640, 480)

        while not self.gui_thread_stop.is_set():
            with self.lock:
                img = self.rgb_image.copy() if self.rgb_image is not None else None

            if img is not None:
                # 현재 단계 표시
                phase_text  = 'PHASE 1: Webcam Detection' \
                              if self.phase == PHASE_WEBCAM \
                              else 'PHASE 2: OAK-D Tracking'
                phase_color = (255, 100, 0) if self.phase == PHASE_WEBCAM else (0, 200, 0)
                cv2.putText(img, phase_text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, phase_color, 2)

                cv2.imshow('Two-Phase Tracker', img)
                key = cv2.waitKey(10) & 0xFF

                if key == ord('q'):
                    self.get_logger().info('종료 요청 (q키)')
                    self.navigator.dock()
                    self.gui_thread_stop.set()
                    rclpy.shutdown()

                elif key == ord('r') and self.phase == PHASE_WEBCAM:
                    # Phase 1에서 수동으로 Phase 2 강제 전환
                    self.get_logger().info('수동 Phase 2 전환 (r키)')
                    self.switch_to_phase2()
            else:
                cv2.waitKey(10)

    def destroy_node(self):
        self.gui_thread_stop.set()
        super().destroy_node()


# ──────────────────────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = TwoPhaseTracker()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.gui_thread_stop.set()
    node.gui_thread.join()
    node.destroy_node()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
