# final

기존 웹 LEGO CAD와 Doosan 충돌 Recovery 관절 조그 프로그램을 하나로 통합한
ROS2(ament_python) 패키지입니다. `ros2 run final final`을 실행하면 브라우저 UI가 열립니다.

- 평상시: 기존 LEGO CAD 화면과 기능이 그대로 동작
- `/collision` (`std_msgs/Bool`)이 `true`: 웹페이지 위에 Recovery J1~J6 조그 UI 표시
- 복귀 버튼: 조그 정지, Recovery 해제, Mode 1(AUTONOMOUS) 복귀 후 LEGO 화면으로 돌아감
- 출력 중에만 `중단하기` 활성화: `/interrupt=true` 발행, 출력 표시 중단 후 TCP 수동 조작 UI 표시
- `emstop.py`가 `/interrupt_ready=true`를 발행하기 전까지 수동 조작 버튼 잠금
- 수동 TCP 이동은 반복 `speedl` 대신 Doosan 연속 조그 전용 `jog_multi` 사용
- 수동 조작 종료: `/interrupt=false` 발행 후 LEGO 화면으로 돌아감

- 3D 뷰: Three.js (WebGL)
- 조작: 좌클릭 = 셀 선택/배치 위치 지정, 우클릭 드래그 = 화면 회전, 휠 = 확대/축소, 가운데버튼 드래그 = 이동(pan)
- 기능: 색상 선택, 블록 종류(1x1/1x2/2x2/2x3) 선택, 블록 회전, 좌표 입력으로 추가/삭제, 마지막 블록 삭제,
  **"출력하기" 버튼으로 현재 배치된 모든 블록의 중심 좌표를 `/centers` 토픽에 한 번 발행**
  → 원본 파이썬 코드의 겹침 검사·받침(지지) 검사·마인크래프트 방식 클릭 배치 로직을 그대로 이식했습니다.

## /centers 토픽

메시지 타입: `std_msgs/String` (JSON 문자열)

웹 UI에서 **"출력하기"** 버튼을 누르면, 그 시점에 배치돼 있는 모든 블록에 대해

```
[블록타입 번호, [중심x, 중심y, 중심z]]
```

형태의 항목을 모은 리스트를 JSON으로 인코딩해서 `/centers` 에 **한 번만** 발행합니다.

- 블록타입 번호: `1x1 = 0`, `1x2 = 1`, `2x2 = 2`, `2x3 = 3`
- 중심 좌표 계산 (앵커 좌표 `(x, y, z)`, 크기 `sx x sy` 기준):
  ```
  cx = x + (sx - 1) / 2
  cy = y + (sy - 1) / 2
  cz = z
  ```
  예) `(0,0,0)`에 1x1 블록 → 중심 `(0, 0, 0)`
      `(0,0,0)~(1,1,0)` 4칸을 차지하는 2x2 블록 → 중심 `(0.5, 0.5, 0)`

예시 발행 내용:
```json
[[0, [0.0, 0.0, 0.0]], [2, [0.5, 0.5, 0.0]]]
```

받는 쪽(구독 노드) 예시:
```python
import json
from std_msgs.msg import String

def callback(msg: String):
    centers = json.loads(msg.data)
    for type_index, (cx, cy, cz) in centers:
        print(type_index, cx, cy, cz)
```

## 폴더 구조

```
final/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/final
└── final/
    ├── __init__.py
    ├── web_server_node.py      # 웹 서버 + LEGO ROS 연동 + 통합 실행점
    ├── recovery_controller.py  # Doosan 충돌 Recovery 및 H2R 조그
    ├── manual_controller.py    # 인위적 중단 후 TCP 수동 조작
    └── static/
        ├── index.html
        ├── style.css
        └── app.js              # Three.js 3D 뷰 + 전체 UI 로직
```

## 빌드 및 실행

ROS2 워크스페이스의 `src/` 아래에 이 폴더를 그대로 넣고 빌드합니다.
Doosan Recovery 기능은 기존 로봇 워크스페이스의 `DR_init`, `DSR_ROBOT2`,
`dsr_msgs2`가 설치·빌드되어 있어야 활성화됩니다. 해당 모듈이 없는 PC에서도
LEGO CAD 웹 UI는 실행되지만 Recovery 기능은 비활성화됩니다.

```bash
# 1) 워크스페이스로 복사
cp -r final ~/ws_cobot_pjt/ws_dsr/src/

# 2) 빌드
cd ~/ws_cobot_pjt/ws_dsr
colcon build --packages-select final
source install/setup.bash

# 3) 실행
ros2 run final final
```

터미널에 아래와 같이 뜨면 정상 실행된 것입니다.

```
[INFO] [final_server]: LEGO CAD 웹 서버 시작됨 -> http://localhost:8080 ...
```

브라우저에서 **http://localhost:8080** 접속하면 3D 화면과 조작 패널이 나타납니다.
같은 네트워크의 다른 기기(예: 태블릿)에서도 `http://<이_PC의_IP>:8080` 으로 접속할 수 있습니다.

## 실행 파라미터 (선택)

포트/호스트/저장 경로를 바꾸고 싶으면 ROS2 파라미터로 지정할 수 있습니다.

```bash
ros2 run final final --ros-args -p port:=9090 -p host:=127.0.0.1
```

- `port` (기본 8080)
- `host` (기본 0.0.0.0, 외부 접속 허용)

## colcon build 없이 바로 테스트하고 싶을 때

패키지를 빌드/설치하지 않아도, 소스 트리 안에서 바로 실행할 수 있도록
`web_server_node.py`가 `share/` 디렉토리를 못 찾으면 자체 `static/` 폴더를 사용하도록 되어 있습니다
(단, 이 경우 `rclpy`가 파이썬 환경에 설치되어 있어야 합니다).

```bash
cd final
python3 -m final.web_server_node
```

## 참고

- 3D 렌더링 라이브러리(Three.js, OrbitControls)는 CDN에서 로드하므로, 브라우저를 사용하는 PC/기기에 인터넷 연결이 필요합니다.
- 좌표계: 원본 파이썬 코드와 동일하게 `x, y`는 바닥 그리드 두 축, `z`는 쌓이는 높이입니다.
  화면상에서는 Three.js가 y축을 위로 사용하므로 내부적으로만 `y ↔ 높이` 축을 매핑해서 그립니다.
