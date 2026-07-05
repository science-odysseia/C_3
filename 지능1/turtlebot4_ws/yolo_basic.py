#!/usr/bin/env python3

import cv2
from ultralytics import YOLO

# 모델 경로
model = YOLO("/home/yswbulb/turtlebot4_ws/best.pt")

# 카메라 번호: 웹캠이면 보통 0
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

CONF_THRES = 0.8

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    # YOLO 추론
    results = model(frame, conf=CONF_THRES, verbose=False)

    # 결과 그리기
    annotated_frame = results[0].plot()

    cv2.imshow("YOLO Detection conf >= 0.8", annotated_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord("q"):  # ESC 또는 q 종료
        break

cap.release()
cv2.destroyAllWindows()