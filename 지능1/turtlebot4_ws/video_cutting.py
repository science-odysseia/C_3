import cv2
import os

# 동영상 파일 경로
video_path = r"/home/turtlebot4_images/recs.mp4"

# 저장 폴더
save_dir = "/home/yswbulb/turtlebot4_images"
os.makedirs(save_dir, exist_ok=True)

cap = cv2.VideoCapture(video_path)

img_count = 0

while True:
    ret, frame = cap.read()

    if not ret:
        print("동영상 종료")
        break

    cv2.imshow("Video", frame)

    key = cv2.waitKey(30) & 0xFF

    # ESC 종료
    if key == 27:
        break

    # r 누르면 현재 프레임 저장
    elif key == ord('r'):
        filename = os.path.join(save_dir, f"image_{img_count:04d}.jpg")
        cv2.imwrite(filename, frame)
        print(f"저장됨: {filename}")
        img_count += 1

cap.release()
cv2.destroyAllWindows()