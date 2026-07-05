import cv2
import torch
from ultralytics import YOLO

# ============================================================
# GPU 확인
# ============================================================
print(torch.cuda.is_available())
print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "GPU 없음 → CPU 사용 중")

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
print(f"사용 디바이스: {DEVICE}")

# ============================================================
# 설정 파라미터
# ============================================================
MODEL_PATH = "yolov8s.pt"       # 모델 경로 (n/s/m/l/x)
CONF_THRESHOLD = 0.2            # 신뢰도 임계값 (0~1)
IOU_THRESHOLD = 0.25            # NMS IOU 임계값 (0~1)
CAMERA_INDEX = 2                # 웹캠 인덱스

# 감지할 클래스 지정 (None이면 전체 클래스 감지)
# COCO 기준 예시: 0=person, 2=car, 15=cat, 16=dog
TARGET_CLASSES = [0, 2]         # 원하는 클래스 id 리스트
# TARGET_CLASSES = None         # 전체 감지

# 바운딩 박스 색상 (BGR)
BOX_COLOR = (0, 255, 0)
TEXT_COLOR = (0, 0, 0)
# ============================================================

model = YOLO(MODEL_PATH)
model.to(DEVICE)

cap = cv2.VideoCapture(CAMERA_INDEX)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 인퍼런스
    results = model(
        frame,
        conf=CONF_THRESHOLD,
        iou=IOU_THRESHOLD,
        classes=TARGET_CLASSES,
        device=DEVICE,
        verbose=False
    )

    # 바운딩 박스 시각화
    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])
        cls_name = model.names[cls_id]

        # 박스
        cv2.rectangle(frame, (x1, y1), (x2, y2), BOX_COLOR, 2)

        # 라벨 배경
        label = f"{cls_name} {conf:.2f}"
        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.rectangle(frame, (x1, y1 - lh - 8), (x1 + lw, y1), BOX_COLOR, -1)

        # 라벨 텍스트
        cv2.putText(frame, label, (x1, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1)

    cv2.imshow("YOLO Webcam", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()