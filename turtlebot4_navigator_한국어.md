# TurtleBot 4 Navigator 한국어 문서

TurtleBot 4 Navigator는 Nav2 Simple Commander를 기반으로 하는 Python 노드입니다. 도킹/언도킹과 같은 TurtleBot 4 전용 기능과 함께 사용하기 편리한 내비게이션 메서드를 제공합니다.

> **참고**
> TurtleBot 4 Navigator는 Nav2 Simple Commander 버전 1.0.11 이상이 필요합니다.

---

## 튜토리얼 패키지 설치

아래 예제들은 `sudo apt install ros-$ROS_DISTRO-turtlebot4-tutorials` 명령어로 설치할 수 있으며, https://github.com/turtlebot/turtlebot4_tutorials 에서도 확인할 수 있습니다. 각 예제에서 로봇은 맵의 원점(origin)에 있는 도크 위에서 시작합니다.

> **참고**
> 이 예제들은 모두 시뮬레이션 환경을 기준으로 설계되었습니다. 실제 로봇에서 실행하려면 소스에서 튜토리얼 패키지를 설치하고, 목적지 좌표를 실제 맵에 맞게 수정해야 합니다. 또한 SLAM, Nav2, 위치 추정(Localization), RViz를 별도로 실행하고 localization 실행 시 환경 맵을 지정해야 합니다. 이 방법은 중급 이상 사용자에게만 권장되며, 별도의 안내 문서는 제공되지 않습니다.

---

## 1. 목표 위치로 이동 (Navigate to Pose)

이 예제는 Nav2 Goal과 동일한 동작을 보여줍니다. Nav2 스택에 맵 위의 목표 자세(pose)를 전달하면 경로를 계산하고, 로봇이 해당 경로를 따라 이동을 시도합니다. 이 예제는 TurtleBot 4 시뮬레이션의 depot 월드에서 실행됩니다.

### 실행 방법

Ignition Gazebo 시뮬레이션 시작:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```

시뮬레이션이 시작되면 Gazebo의 "Play" 버튼을 눌러 시뮬레이션을 시작하세요.

다른 터미널에서 실행:

```bash
ros2 run turtlebot4_python_tutorials nav_to_pose
```

### 코드 설명

`main` 함수를 살펴보겠습니다.

```python
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # 도크에서 시작
    if not navigator.getDockedStatus():
        navigator.info('자세 초기화 전 도킹 중')
        navigator.dock()

    # 초기 자세 설정
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Nav2 준비 대기
    navigator.waitUntilNav2Active()

    # 목표 자세 설정
    goal_pose = navigator.getPoseStamped([-13.0, 9.0], TurtleBot4Directions.EAST)

    # 언도킹
    navigator.undock()

    # 목표 자세로 이동
    navigator.startToPose(goal_pose)

    rclpy.shutdown()
```

#### 노드 초기화

rclpy를 초기화하고 `TurtleBot4Navigator` 객체를 생성합니다. 이 과정에서 필요한 ROS 2 퍼블리셔, 서브스크라이버, 액션 클라이언트가 초기화됩니다.

```python
rclpy.init()

navigator = TurtleBot4Navigator()
```

#### 로봇 도킹

로봇이 도킹 상태인지 확인하고, 도킹되어 있지 않으면 도킹 액션 목표를 전송합니다. 도킹을 통해 로봇이 맵의 `[0.0, 0.0]` 좌표에 있음을 보장합니다.

```python
if not navigator.getDockedStatus():
    navigator.info('자세 초기화 전 도킹 중')
    navigator.dock()
```

#### 초기 자세 설정

로봇이 도킹된 것을 확인했으므로, 초기 자세를 `[0.0, 0.0]`, 방향은 "북쪽(North)"으로 설정합니다.

```python
initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
navigator.setInitialPose(initial_pose)
```

TurtleBot 4 Navigator는 방위 방향을 사용하여 맵 기준으로 로봇의 방향을 설정합니다. 더 정밀한 방향이 필요하다면 정수나 실수값을 직접 사용할 수 있습니다.

```python
class TurtleBot4Directions(IntEnum):
    NORTH = 0        # 북쪽
    NORTH_WEST = 45  # 북서쪽
    WEST = 90        # 서쪽
    SOUTH_WEST = 135 # 남서쪽
    SOUTH = 180      # 남쪽
    SOUTH_EAST = 225 # 남동쪽
    EAST = 270       # 동쪽
    NORTH_EAST = 315 # 북동쪽
```

> **참고**
> 여기서 방위 방향은 지구의 실제 자북(磁北)이 아닌 맵을 기준으로 합니다. 북쪽은 맵에서 위쪽, 서쪽은 맵에서 왼쪽 방향을 의미합니다.

#### Nav2 준비 대기

초기 위치가 설정되면 Nav2 스택은 해당 위치에 로봇을 배치하고 위치 추정을 시작합니다. 내비게이션 목표를 보내기 전에 Nav2가 준비될 때까지 기다립니다.

```python
navigator.waitUntilNav2Active()
```

> **참고**
> 이 호출은 Nav2가 준비될 때까지 블로킹됩니다. Nav2가 실행 중인지 확인하세요.

#### 목표 자세 설정

`getPoseStamped` 메서드를 사용하여 `geometry_msgs/PoseStamped` 메시지를 생성합니다. x, y 좌표와 목표 지점에 도달했을 때 로봇이 향할 방향을 전달합니다.

```python
goal_pose = navigator.getPoseStamped([-13.0, 9.0], TurtleBot4Directions.EAST)
```

#### 언도킹 및 목표 위치로 이동

목표 자세로 이동할 준비가 되었습니다. 먼저 언도킹하여 로봇이 도크를 통과하지 않도록 한 뒤, 목표 자세를 전송합니다. 로봇이 이동하는 동안 액션으로부터 예상 도착 시간이 포함된 피드백을 받습니다.

```python
navigator.undock()

navigator.startToPose(goal_pose)
```

로봇이 목표에 도달하면 `rclpy.shutdown()`을 호출하여 rclpy 컨텍스트를 정상 종료합니다.

#### RViz에서 내비게이션 진행 상황 확인

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

---

## 2. 여러 자세를 통과하며 이동 (Navigate Through Poses)

이 예제는 Navigate Through Poses 행동 트리를 보여줍니다. Nav2 스택에 맵 위의 자세 목록을 전달하면, 각 자세를 순서대로 통과하여 마지막 자세에 도달하는 경로를 생성합니다. 이 예제는 TurtleBot 4 시뮬레이션의 warehouse 월드에서 실행됩니다.

### 실행 방법

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```

```bash
ros2 run turtlebot4_python_tutorials nav_through_poses
```

### 코드 설명

```python
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # 도크에서 시작
    if not navigator.getDockedStatus():
        navigator.info('자세 초기화 전 도킹 중')
        navigator.dock()

    # 초기 자세 설정
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Nav2 준비 대기
    navigator.waitUntilNav2Active()

    # 목표 자세 설정
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-3.0, -0.0], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([-3.0, -3.0], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped([3.0, -3.0], TurtleBot4Directions.NORTH_WEST))
    goal_pose.append(navigator.getPoseStamped([9.0, -1.0], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped([9.0, 1.0], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST))

    # 언도킹
    navigator.undock()

    # 자세들을 통과하며 이동
    navigator.startThroughPoses(goal_pose)

    # 이동 완료 후 도킹
    navigator.dock()

    rclpy.shutdown()
```

이 예제는 "목표 위치로 이동"과 동일하게 시작합니다. 노드를 초기화하고, 로봇이 도킹되어 있는지 확인한 뒤 초기 자세를 설정합니다. 그런 다음 Nav2가 활성화될 때까지 기다립니다.

#### 목표 자세 목록 설정

로봇이 통과해야 하는 자세들을 `PoseStamped` 메시지 리스트로 생성합니다.

```python
goal_pose = []
goal_pose.append(navigator.getPoseStamped([-3.0, -0.0], TurtleBot4Directions.EAST))
goal_pose.append(navigator.getPoseStamped([-3.0, -3.0], TurtleBot4Directions.NORTH))
goal_pose.append(navigator.getPoseStamped([3.0, -3.0], TurtleBot4Directions.NORTH_WEST))
goal_pose.append(navigator.getPoseStamped([9.0, -1.0], TurtleBot4Directions.WEST))
goal_pose.append(navigator.getPoseStamped([9.0, 1.0], TurtleBot4Directions.SOUTH))
goal_pose.append(navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST))
```

#### 자세를 통과하며 이동

언도킹 후 각 지점을 통과하며 이동합니다. 마지막 자세에 도달하면 도크로 돌아갑니다.

```python
navigator.undock()

navigator.startThroughPoses(goal_pose)

navigator.dock()
```

---

## 3. 웨이포인트 따라가기 (Follow Waypoints)

이 예제는 웨이포인트 추종 방법을 보여줍니다. Nav2 스택에 웨이포인트 목록을 전달하면, 마지막 웨이포인트에 도달할 때까지 각 웨이포인트를 순서대로 통과하는 경로를 생성합니다.

**Navigate Through Poses와의 차이점**: 웨이포인트 추종은 각 웨이포인트에 개별적으로 도달하도록 경로를 계획하는 반면, Navigate Through Poses는 다른 자세들을 통과하여 마지막 자세에 도달하는 하나의 경로를 계획합니다. 이 예제는 depot 월드에서 실행됩니다.

### 실행 방법

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```

```bash
ros2 run turtlebot4_python_tutorials follow_waypoints
```

### 코드 설명

```python
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # 도크에서 시작
    if not navigator.getDockedStatus():
        navigator.info('자세 초기화 전 도킹 중')
        navigator.dock()

    # 초기 자세 설정
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Nav2 준비 대기
    navigator.waitUntilNav2Active()

    # 목표 자세 설정
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-3.3, 5.9], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped([2.1, 6.3], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([2.0, 1.0], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped([-1.0, 0.0], TurtleBot4Directions.NORTH))

    # 언도킹
    navigator.undock()

    # 웨이포인트 추종
    navigator.startFollowWaypoints(goal_pose)

    # 이동 완료 후 도킹
    navigator.dock()

    rclpy.shutdown()
```

이 예제는 Navigate Through Poses와 매우 유사합니다. 차이점은 다른 웨이포인트 좌표를 사용하고, 내비게이션에 `startFollowWaypoints` 메서드를 사용한다는 것입니다.

---

## 4. 경로 생성 (Create Path)

이 예제는 런타임 중에 RViz에서 내비게이션 경로를 생성하는 방법을 보여줍니다. 2D Pose Estimate 도구를 사용하여 TurtleBot 4 Navigator에 자세 목록을 전달하고, Follow Waypoints 동작으로 해당 자세들을 따라갑니다. 이 예제는 실제 TurtleBot 4 로봇에서 실행된 것입니다.

### 실행 방법

PC 또는 Raspberry Pi에서 환경 맵을 사용하여 내비게이션을 시작한 뒤:

```bash
ros2 run turtlebot4_python_tutorials create_path
```

PC에서 RViz 시작:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

### 코드 설명

```python
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # 목표 자세 설정
    goal_pose = navigator.createPath()

    if len(goal_pose) == 0:
        navigator.error('자세가 설정되지 않았습니다. 종료합니다.')
        exit(0)

    # 도크에서 시작
    if not navigator.getDockedStatus():
        navigator.info('자세 초기화 전 도킹 중')
        navigator.dock()

    # 초기 자세 설정
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.clearAllCostmaps()
    navigator.setInitialPose(initial_pose)

    # Nav2 준비 대기
    navigator.waitUntilNav2Active()

    # 언도킹
    navigator.undock()

    # 자세들을 통과하며 이동
    navigator.startFollowWaypoints(goal_pose)

    # 이동 완료 후 도킹
    navigator.dock()

    rclpy.shutdown()
```

#### 경로 생성

초기화 후 사용자는 2D Pose Estimate 도구를 사용하여 경로를 생성하도록 안내받습니다. 최소 하나의 자세를 설정해야 하며, 모든 자세가 설정되면 로봇이 이동을 시작합니다.

```python
goal_pose = navigator.createPath()

if len(goal_pose) == 0:
    navigator.error('자세가 설정되지 않았습니다. 종료합니다.')
    exit(0)
```

#### 초기 자세 설정 및 코스트맵 초기화

초기 자세를 설정하고 모든 코스트맵을 초기화합니다. 2D Pose Estimate 도구는 Nav2 스택도 구독하고 있어, 사용할 때마다 Nav2가 로봇이 해당 위치에 있다고 잘못 인식합니다. 코스트맵 초기화로 경로 생성 중 발생한 잘못된 코스트맵을 제거할 수 있습니다.

```python
if not navigator.getDockedStatus():
    navigator.info('자세 초기화 전 도킹 중')
    navigator.dock()

initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
navigator.clearAllCostmaps()  # 잘못된 코스트맵 제거
navigator.setInitialPose(initial_pose)

navigator.waitUntilNav2Active()
```

#### 경로 따라가기

언도킹 후 생성된 경로를 따라갑니다. 이 예제에서는 Follow Waypoints 동작을 사용하지만, Navigate Through Poses로 쉽게 대체할 수 있습니다.

```python
navigator.undock()

navigator.startFollowWaypoints(goal_pose)

navigator.dock()  # 마지막 자세가 도크 근처인 경우에만 사용
```

> **참고**
> 경로를 생성하는 동안 클릭한 위치에 로봇이 배치되는 것처럼 보일 수 있습니다. 이는 정상적인 동작이며, TurtleBot 4 Navigator가 초기 자세를 설정하면 정리됩니다.

---

## 5. 배달 서비스 (Mail Delivery)

이 예제는 대화형 배달 경로를 만드는 방법을 보여줍니다. 사용자가 터미널 인터페이스를 통해 미리 정의된 위치로 로봇을 보낼 수 있습니다. Navigate to Pose 동작을 사용하여 목표 위치로 이동합니다.

### 실행 방법

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```

```bash
ros2 run turtlebot4_python_tutorials mail_delivery
```

### 코드 설명

```python
def main(args=None):
    rclpy.init(args=args)

    navigator = TurtleBot4Navigator()

    # 도크에서 시작
    if not navigator.getDockedStatus():
        navigator.info('자세 초기화 전 도킹 중')
        navigator.dock()

    # 초기 자세 설정
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Nav2 준비 대기
    navigator.waitUntilNav2Active()

    # 언도킹
    navigator.undock()

    # 목표 위치 옵션 준비
    goal_options = [
        {'name': '홈',
         'pose': navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST)},

        {'name': '위치 1',
         'pose': navigator.getPoseStamped([10.0, 6.0], TurtleBot4Directions.EAST)},

        {'name': '위치 2',
         'pose': navigator.getPoseStamped([-9.0, 9.0], TurtleBot4Directions.NORTH)},

        {'name': '위치 3',
         'pose': navigator.getPoseStamped([-12.0, 2.0], TurtleBot4Directions.NORTH_WEST)},

        {'name': '위치 4',
         'pose': navigator.getPoseStamped([3.0, -7.0], TurtleBot4Directions.WEST)},

        {'name': '종료',
         'pose': None}
    ]

    navigator.info('배달 서비스에 오신 것을 환영합니다.')

    while True:
        # 목표 목록을 표시할 문자열 생성
        options_str = '원하는 로봇 목표 위치에 해당하는 번호를 입력하세요:\n'
        for i in range(len(goal_options)):
            options_str += f'    {i}. {goal_options[i]["name"]}\n'

        # 사용자에게 목표 위치 입력 요청
        raw_input = input(f'{options_str}선택: ')

        selected_index = 0

        # 입력값이 숫자인지 확인
        try:
            selected_index = int(raw_input)
        except ValueError:
            navigator.error(f'잘못된 목표 선택: {raw_input}')
            continue

        # 입력값이 유효한 범위 내에 있는지 확인
        if (selected_index < 0) or (selected_index >= len(goal_options)):
            navigator.error(f'목표 선택 범위 초과: {selected_index}')

        # 종료 확인
        elif goal_options[selected_index]['name'] == '종료':
            break

        else:
            # 요청한 위치로 이동
            navigator.startToPose(goal_options[selected_index]['pose'])

    rclpy.shutdown()
```

#### 목표 자세 준비

로봇이 이동할 수 있는 모든 위치를 `PoseStamped` 메시지 리스트로 생성합니다.

#### 목표 자세 선택 (반복 루프)

목표 목록을 터미널에 표시하고, 사용자 입력을 기다립니다. 입력값이 정수인지, 유효한 범위 내에 있는지 검증합니다. 잘못된 입력이면 루프를 다시 시작합니다.

---

## 6. 순찰 루프 (Patrol Loop)

이 예제는 자동 충전 기능을 갖춘 무한 순찰 루프를 만드는 방법을 보여줍니다. 로봇이 설정된 자세들을 계속 순환하다가 배터리가 부족해지면 충전을 위해 도크로 돌아갑니다. 충분히 충전되면 다시 순찰을 시작합니다.

### 실행 방법

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```

```bash
ros2 run turtlebot4_python_tutorials patrol_loop
```

### 코드 설명

#### 배터리 모니터링 노드

```python
class BatteryMonitor(Node):

    def __init__(self, lock):
        super().__init__('battery_monitor')

        self.lock = lock

        # /battery_state 토픽 구독
        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data)

    # 콜백 함수
    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage  # 배터리 잔량 저장

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()
```

이 클래스는 Create3® 배터리 충전량을 모니터링하고, 의사 결정에 활용할 수 있도록 배터리 정보를 제공합니다.

**구독**: `battery_state` 토픽을 구독하여 Create3®가 발행하는 최신 배터리 상태를 받아 `battery_state_callback`을 호출합니다.

**배터리 상태 콜백**: 메시지를 받을 때마다 충전 잔량을 멤버 변수에 저장합니다. GIL(Global Interpreter Lock)을 사용하여 메인 함수와 배터리 모니터링 함수가 동시에 해당 변수에 접근하거나 수정하지 못하도록 합니다.

**스레드 준비**: 새 메시지를 수신하고 콜백을 처리하려면 노드가 지속적으로 스피닝(spinning) 해야 합니다. 내비게이션 코드와 동시에 스피닝하기 위해 멀티스레딩을 사용합니다.

#### 메인 함수

```python
def main(args=None):
    rclpy.init(args=args)

    lock = Lock()
    battery_monitor = BatteryMonitor(lock)

    navigator = TurtleBot4Navigator()
    battery_percent = None
    position_index = 0

    thread = Thread(target=battery_monitor.thread_function, daemon=True)
    thread.start()

    # 도크에서 시작
    if not navigator.getDockedStatus():
        navigator.info('자세 초기화 전 도킹 중')
        navigator.dock()

    # 초기 자세 설정
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Nav2 준비 대기
    navigator.waitUntilNav2Active()

    # 언도킹
    navigator.undock()

    # 순찰 목표 자세 준비
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-5.0, 1.0], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([-5.0, -23.0], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped([9.0, -23.0], TurtleBot4Directions.NORTH_WEST))
    goal_pose.append(navigator.getPoseStamped([10.0, 2.0], TurtleBot4Directions.WEST))

    while True:
        with lock:
            battery_percent = battery_monitor.battery_percent  # 배터리 잔량 업데이트

        if (battery_percent is not None):
            navigator.info(f'배터리 잔량: {(battery_percent*100):.2f}%')

            # 배터리 충전 수준 확인
            if (battery_percent < BATTERY_CRITICAL):
                navigator.error('배터리 위험 수준. 충전하거나 전원을 끄세요')
                break
            elif (battery_percent < BATTERY_LOW):
                # 도크 근처로 이동
                navigator.info('충전을 위해 도킹 중')
                navigator.startToPose(navigator.getPoseStamped([-1.0, 1.0],
                                      TurtleBot4Directions.EAST))
                navigator.dock()

                if not navigator.getDockedStatus():
                    navigator.error('로봇 도킹 실패')
                    break

                # 충전 완료까지 대기
                navigator.info('충전 중...')
                battery_percent_prev = 0
                while (battery_percent < BATTERY_HIGH):
                    sleep(15)
                    battery_percent_prev = floor(battery_percent*100)/100
                    with lock:
                        battery_percent = battery_monitor.battery_percent

                    # 1% 증가할 때마다 충전량 출력
                    if battery_percent > (battery_percent_prev + 0.01):
                        navigator.info(f'배터리 잔량: {(battery_percent*100):.2f}%')

                # 언도킹
                navigator.undock()
                position_index = 0  # 순찰 루프 처음부터 재시작

            else:
                # 다음 순찰 위치로 이동
                navigator.startToPose(goal_pose[position_index])

                position_index = position_index + 1
                if position_index >= len(goal_pose):
                    position_index = 0  # 인덱스 순환

    battery_monitor.destroy_node()
    rclpy.shutdown()
```

**멀티스레딩**: GIL을 사용하여 동시 접근을 방지합니다. lock과 배터리 모니터 노드를 생성하고, 같은 lock을 공유합니다. 배터리 모니터 스레드를 생성하고 시작합니다.

**동작 루프 로직**:
- **배터리 위험(CRITICAL)**: 루프를 중단하고 프로그램을 종료합니다.
- **배터리 부족(LOW)**: 도크 근처로 이동 → 도킹 → 충전 완료까지 대기 → 언도킹 → 순찰 루프 처음부터 재시작
- **정상**: 다음 순찰 위치로 이동하고, `position_index`로 위치를 순환 추적합니다.

---

## 7. 대형 언어 모델(LLM) 통합

이 예제는 OpenAI Chat Completions API와 TurtleBot 4를 통합한 것으로, 'Code as Policies: Language Model Programs for Embodied Control (Liang et al.)' 연구에 크게 영감을 받았습니다.

LLM이 몇 가지 예시만으로 TurtleBot 4 Navigation API 사용법을 '학습'할 수 있으며, 자연어 명령을 중간 파싱 단계 없이 직접 API 호출로 변환할 수 있음을 보여줍니다.

> 제품에 적용하기 전에 원본 논문의 주의사항 목록을 꼭 확인하세요! :smile:

### 설치

1. OpenAI 계정과 API 키를 생성하세요. API 키는 생성 후 다시 확인할 수 없으니 안전한 곳에 보관하세요.
2. OpenAI 계정에 크레딧이 있는지 확인하세요. 초기 테스트에는 몇 센트면 충분합니다.
3. `pip install openai`로 OpenAI Python 라이브러리를 설치하세요.
4. 워크스페이스가 없다면 생성: `mkdir ~/turtlebot4_ws/src -p`
5. 코드를 워크스페이스에 다운로드 또는 클론하세요.
6. `cd ~/turtlebot4_ws/ && colcon build`로 빌드하세요.

### 실행 방법

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true world:=depot map:=/opt/ros/humble/share/turtlebot4_navigation/maps/depot.yaml
```

```bash
ros2 launch turtlebot4_openai_tutorials natural_language_nav_launch.py openai_api_key:=API_KEY parking_brake:=false
```

`parking_brake:=true`로 설정하면 명령이 로봇에 전달되지 않습니다. 로봇 초기화에 시간이 걸리므로 처음에는 `parking_brake:=true`를 권장합니다.

로봇이 언도킹되면 세 번째 터미널에서:

```bash
ros2 topic pub --once /user_input std_msgs/msg/String "data: STRING"
```

사용 예시:

```bash
ros2 topic pub --once /user_input std_msgs/msg/String "data: Dock"
ros2 topic pub --once /user_input std_msgs/msg/String "data: Go to -1,0, face East"
ros2 topic pub --once /user_input std_msgs/msg/String "data: Go to 5,5"
ros2 topic pub --once /user_input std_msgs/msg/String "data: Move to the wooden object"
ros2 topic pub --once /user_input std_msgs/msg/String "data: Navigate to the item which can hold oil"
ros2 topic pub --once /user_input std_msgs/msg/String "data: Travel to the room containing a toilet"
```

LLM 덕분에 코드에 직접 없는 관계도 추론할 수 있습니다. 예를 들어 'oil(기름)'이나 'toilet(변기)'이라는 단어는 코드에 전혀 없지만 API와 연결하여 추론합니다.

> **주의**: `--once` 플래그를 반드시 사용하세요. 그렇지 않으면 터미널이 계속 크레딧을 소모합니다.

시스템이 새 입력을 받을 준비가 됐는지 확인:

```bash
ros2 topic echo /ready_for_input
```

### 코드 설명

#### 런치파일

```python
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('openai_api_key', default_value='',
                              description='https://platform.openai.com/account/api-keys 에서 설정한 API 키'),
        DeclareLaunchArgument('model_name', default_value='gpt-3.5-turbo',
                              description='https://platform.openai.com/docs/guides/gpt 에서 선택하는 OpenAI 모델명'),
        DeclareLaunchArgument('parking_brake', default_value='true',
                              description='false로 설정하면 로봇에 명령이 실행됨'),
        Node(
            package='turtlebot4_openai_tutorials',
            executable='natural_language_nav',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'openai_api_key': LaunchConfiguration('openai_api_key')},
                {'model_name': LaunchConfiguration('model_name')},
                {'parking_brake': LaunchConfiguration('parking_brake')}
            ]
        ),
    ])
```

세 가지 핵심 파라미터: `openai_api_key`(필수), `model_name`(기본값 사용 가능), `parking_brake`(명령 실행 여부 제어)

#### GPTNode

```python
def __init__(self, navigator):
    super().__init__('gpt_node')
    self.declare_parameter('openai_api_key', '')
    self.declare_parameter('model_name', 'gpt-3.5-turbo')
    self.declare_parameter('parking_brake', True)

    # OpenAI 키, 모델, 프롬프트 설정
    openai.api_key = self.get_parameter('openai_api_key').value
    self.model_name = self.get_parameter('model_name').value
    self.prompts = []
    self.full_prompt = ""

    # ROS 관련 설정: navigator 노드 핸들, 토픽
    self.navigator = navigator
    self.sub_input = self.create_subscription(String, 'user_input', self.user_input, 10)
    self.pub_ready = self.create_publisher(Bool, 'ready_for_input', 10)
    self.publish_status(False)
```

**OpenAI 쿼리 함수**: 프롬프트(컨텍스트)와 쿼리를 받아 OpenAI에 전송하고 응답을 반환합니다.

```python
def query(self, base_prompt, query, stop_tokens=None, query_kwargs=None, log=True):
    new_prompt = f'{base_prompt}\n{query}'
    """ OpenAI API에 프롬프트와 쿼리를 전송하고 응답 반환 """

    use_query_kwargs = {
        'model': self.model_name,
        'max_tokens': 512,
        'temperature': 0,
    }
    if query_kwargs is not None:
        use_query_kwargs.update(query_kwargs)

    messages = [
        {"role": "user", "content": new_prompt}
    ]
    response = openai.ChatCompletion.create(
        messages=messages, stop=stop_tokens, **use_query_kwargs
    )['choices'][0]['message']['content'].strip()

    if log:
        self.info(query)
        self.info(response)

    return response
```

**사용자 입력 처리**: 시스템이 준비된 경우 입력을 포맷하고, OpenAI API를 호출하여 결과 코드를 선택적으로 실행합니다.

```python
def user_input(self, msg):
    """사용자 입력을 처리하고 결과 코드를 선택적으로 실행합니다."""
    if not self.ready_for_input:
        # 퍼블리셔가 하나뿐이라면 문제없지만, 좋은 습관
        self.info(f"준비되지 않은 상태에서 입력 <{msg.data}> 수신, 건너뜀")
        return
    self.publish_status(False)
    self.info(f"입력 수신: <{msg.data}>")

    # 코드 샘플 형식에 맞게 "# " 추가 후 쿼리 실행
    query = '# ' + msg.data
    result = self.query(f'{self.full_prompt}', query, ['#', 'objects = ['])

    # 예제 API는 'self.navigator'가 아닌 'navigator'를 사용하기 때문
    navigator = self.navigator

    # 실행 여부 확인
    if not self.get_parameter('parking_brake').value:
        try:
            exec(result, globals(), locals())
        except Exception:
            self.error("결과 코드 실행 실패:")
            self.error("---------------\n"+result)
            self.error("---------------")
    self.publish_status(True)
```

#### 메인 스크립트

**프롬프트 파일 읽기**: `ament_index_python`을 사용하여 패키지의 'prompts' 디렉터리에서 예제 프롬프트 파일을 읽습니다.

```python
def read_prompt_file(prompt_file):
    """패키지 'prompts' 디렉터리에 있는 파일을 읽습니다."""
    data_path = ament_index_python.get_package_share_directory('turtlebot4_openai_tutorials')
    prompt_path = os.path.join(data_path, 'prompts', prompt_file)

    with open(prompt_path, 'r') as file:
        return file.read()
```

**시스템 초기화**:

```python
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()
    gpt = GPTNode(navigator)

    gpt.prompts.append(read_prompt_file('turtlebot4_api.txt'))
    for p in gpt.prompts:
        gpt.full_prompt = gpt.full_prompt + '\n' + p

    # 명령을 실행하지 않을 경우 로봇과 통신할 필요 없음
    if not gpt.get_parameter('parking_brake').value:
        gpt.warn("파킹 브레이크 해제됨, 로봇이 명령을 실행합니다!")
        # 도크에서 시작
        if not navigator.getDockedStatus():
            navigator.info('자세 초기화 전 도킹 중')
            navigator.dock()

        # 초기 자세 설정
        initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        navigator.setInitialPose(initial_pose)

        # Nav2 준비 대기
        navigator.waitUntilNav2Active()

        # 언도킹
        navigator.undock()
    else:
        gpt.warn("파킹 브레이크 설정됨, 로봇이 명령을 실행하지 않습니다!")
```

**사용자 입력 루프**: 내비게이션 시스템이 초기화되면 커스텀 컨텍스트를 추가합니다. 객체 감지 시스템이 없으므로 환경 내 특정 객체와 위치 간의 관계를 수동으로 추가합니다.

```python
    # 커스텀 컨텍스트 추가 (객체명과 좌표 매핑)
    context = "destinations = {'wood crate': [0.0, 3.0, 0], 'steel barrels': [2.0, 2.0, 90], \
        'bathroom door': [-6.0, -6.0, 180] }"
    exec(context, globals())
    gpt.info("컨텍스트와 함께 입력 파싱 루프 진입:")
    gpt.info(context)
    gpt.full_prompt = gpt.full_prompt + '\n' + context

    # 메인 루프
    gpt.publish_status(True)
    try:
        rclpy.spin(gpt)
    except KeyboardInterrupt:
        pass

    gpt.destroy_node()
    navigator.destroy_node()

    rclpy.shutdown()
```

#### 프롬프트 파일

프롬프트 파일에는 올바른 API 사용 예시가 포함되어 있습니다. 각 API 호출 묶음 앞에 대표 프롬프트가 붙습니다. 이 파일의 예시에 버그가 없어야 합니다.

```python
# 도킹
navigator.dock()
# 언도킹
navigator.undock()
# 위치 0.0, 1.0으로 이동, 북쪽을 향함
navigator.info('0.0, 1.0, 북쪽으로 이동 중')
goal = navigator.getPoseStamped([0.0, 1.0], TurtleBot4Directions.NORTH)
navigator.startToPose(goal)
# 위치 -1.0, 5.0으로 이동, 남동쪽을 향함
navigator.info('-1.0, 5.0, 남동쪽으로 이동 중')
goal = navigator.getPoseStamped([-1.0, 5.0], TurtleBot4Directions.SOUTH_EAST)
navigator.startToPose(goal)
# 선반으로 이동
dest = destinations['metal shelf']
navigator.info('금속 선반으로 이동 중')
goal = navigator.getPoseStamped(dest[0:2], dest[2])
navigator.startToPose(goal)
# 나무 물체로 이동
dest = destinations['wooden crate']
navigator.info('나무 상자로 이동 중')
goal = navigator.getPoseStamped(dest[0:2], dest[2])
navigator.startToPose(goal)
# 지게차로 이동
dest = destinations['forklift']
navigator.info('지게차로 이동 중')
goal = navigator.getPoseStamped(dest[0:2], dest[2])
navigator.startToPose(goal)
```
