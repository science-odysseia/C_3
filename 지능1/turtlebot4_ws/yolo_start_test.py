import cv2
import time
import subprocess
from ultralytics import YOLO

# =====================
# 설정
# =====================
MODEL_PATH = "/home/yswbulb/turtlebot4_ws/best.pt"
CAMERA_INDEX = 2

TARGET_CLASS = 0
CONF_THRESHOLD = 0.8
DETECT_SECONDS = 2.0

WS_SETUP = "/home/yswbulb/turtlebot4_ws/install/setup.bash"

LAUNCH_COMMAND = """
source install/setup.bash &&
ros2 launch miniproject initiating.launch.py
"""


def main():
    model = YOLO(MODEL_PATH)

    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    detect_start_time = None

    print("YOLO detection started")
    print(f"Target class: {TARGET_CLASS}, conf >= {CONF_THRESHOLD}")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        # 0번 클래스만 추론
        results = model(
            frame,
            verbose=False,
            conf=CONF_THRESHOLD,
            classes=[TARGET_CLASS]
        )

        detected = False

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])

                if cls == TARGET_CLASS and conf >= CONF_THRESHOLD:
                    detected = True
                    print(f"Detected class {cls}, conf: {conf:.2f}")
                    break

            if detected:
                break

        annotated_frame = results[0].plot()

        if detected:

            if detect_start_time is None:
                detect_start_time = time.time()

            elapsed = time.time() - detect_start_time

            cv2.putText(
                annotated_frame,
                f"Class {TARGET_CLASS} Detected ({elapsed:.1f}s)",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            if elapsed >= DETECT_SECONDS:

                print(
                    f"0번 클래스가 {CONF_THRESHOLD} 이상으로 "
                    f"{DETECT_SECONDS}초 이상 탐지됨"
                )

                print("Undock 후 Launch 실행")

                cap.release()
                cv2.destroyAllWindows()

                subprocess.Popen(
                    f"gnome-terminal -- bash -c 'source {WS_SETUP} && {LAUNCH_COMMAND}; exec bash'",
                    shell=True
                )

                return

        else:

            detect_start_time = None

            cv2.putText(
                annotated_frame,
                "No target class",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )

        cv2.imshow("YOLO Detector", annotated_frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()