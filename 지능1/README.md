제출필요 문서

1. 시스템 설계 문서

2. 최종 발표 ppt


business requirement

내부설계

재실행절차

1. ssh ubuntu@(ip주소)
2. pw : turtlebot4
3. turtlebot4-service-restart
4. 대기
5. exit


## 터틀봇4 기본 Setting
```bash
# === Clone TurtleBot4 and related packages ===
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4.git -b humble
git clone https://github.com/turtlebot/turtlebot4_simulator.git -b humble
git clone https://github.com/turtlebot/turtlebot4_desktop.git -b humble
git clone https://github.com/turtlebot/turtlebot4_tutorials.git
git clone https://github.com/robo-friends/m-explore-ros2.git

# === Install dependencies with rosdep ===
cd ~/turtlebot4_ws

# Initialize rosdep if not already initialized
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

# Update rosdep and install dependencies
rosdep update
cd ~/turtlebot4_ws
rosdep install --from-path src -yi --rosdistro humble

# build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
<br>

<br>

gazebo설치(필수아님)

```bash
# === Install Gazebo and ROS-Ignition bridge ===

# Install wget if not already installed
sudo apt-get update && sudo apt-get install -y wget

# Install ROS-Ignition bridge for Humble
sudo apt install -y ros-humble-ros-ign

# Confirm installation of ros_ign packages
ros2 pkg list | grep ros_ign

# Add OSRF’s Gazebo (Ignition) repository
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Import the OSRF public key
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Update apt cache and install Ignition Fortress
sudo apt-get update && sudo apt-get install -y ignition-fortress



cd ~/turtlebot4_ws
rosdep update
rosdep install --from-path src -yi --rosdistro humble



source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

```bash
# === Set the Ignition version environment variable ===
echo 'export IGNITION_VERSION=fortress' >> ~/.bashrc
source ~/.bashrc
```


실행확인
```bash
cd ~/turtlebot4_ws
source install/setup.bash
ros2 pkg list | grep turtlebot4
```

    turtlebot4_description
    turtlebot4_ignition_bringup
    turtlebot4_ignition_gui_plugins
    turtlebot4_ignition_toolbox
    turtlebot4_msgs
    turtlebot4_navigation
    turtlebot4_node
    turtlebot4_simulator
    turtlebot4_viz 

가 다 뜨면 성공.

#### Yolo 셋업
labelImg설치

```bash
sudo apt update
sudo apt install pyqt5-dev-tools qttools5-dev-tools python3-pyqt5 -y
cd ~
git clone https://github.com/tzutalin/labelImg.git
cd labelImg
make qt5py3
```
**Label 편집**
- labelImg > data 로 이동해서 predefined_classes.txt를 수정한다.
- label이름 기입한다.
    
LabelImg 실행
```bash
cd .. #move up to lableimg directory
python3 labelImg.py
```

YOLO설치
```bash
pip install --upgrade pip

nvidia-smi  # Check if the GPU is available
# If you have a GPU, install the CUDA version of PyTorch
# $ pip install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu124

pip install ultralytics
```

호환성을 위한 다운그레이드

```bash
pip uninstall opencv-python -y
pip install "opencv-python==4.9.0.80"
#install numpy version supported by yolo tracking
pip install "numpy==1.26.4"
```

#### 네트워크 셋업
(공식문서 참고 : https://turtlebot.github.io/turtlebot4-user-manual/setup/networking.html)


아래명령실행

```bash
# make sure pc is connected to the same wifi router as the robot
wget -qO - https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/turtlebot4_discovery/configure_discovery.sh | bash <(cat) </dev/tty
```

ROS_DOMAIN_ID 로봇번호로 설정

Discovery Server ID : 이거도 본인로봇번호

Discovery Server IP : 로봇의 IP(로봇에 뜸)

Discovery Server Port : 그냥엔터(11811 들어감)

마지막으로 d를 쳐서 완료(a를 치면 추가작업 할 수 있고 이건 나중에 멀티로봇할때 할거임)


bashrc에 `source /etc/turtlebot4_discovery/setup.bash`등록됬는지 확인하고 안되면 넣어주자.

```bash
source .bashrc
ros2 daemon stop
ros2 daemon start
```
2번정도 반복

`ros2 topic list` 쳤을 때 /robot(로봇번호)/xxx 이런 토픽이 떠야함.

설정내용 보거나 수정하려면 

```bash
nano /etc/turtlebot4_discovery/setup.bash
```



#### undock, dock, joystick

```bash
#Undock the robot if not undocked; enter your robot namespace
ros2 action send_goal /robot<n>/undock irobot_create_msgs/action/Undock "{}"

#dock명령
ros2 action send_goal /robot<n>/dock irobot_create_msgs/action/Dock "{}"


#make sure update the <n> to match your robot namespace(joystick)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot<n>/cmd_vel
```



로봇 라즈베리파이 접속

ssh ubuntu@로봇ip주소

비번은 turtlebot4

나가는 명령어는 exit


응급차 사이렌 소리내는 cli

```bash
 ros2 topic pub --once /robot<n>/cmd_audio irobot_create_msgs/msg/AudioNoteVector "{append: false, notes: [
  {frequency: 880.0, max_runtime: {sec: 0, nanosec: 300000000}},  #삐
  {frequency: 440.0, max_runtime: {sec: 0, nanosec: 300000000}},  #뽀
  {frequency: 880.0, max_runtime: {sec: 0, nanosec: 300000000}},  #삐
  {frequency: 440.0, max_runtime: {sec: 0, nanosec: 300000000}}   #뽀
  ]}"
  ```
  
  
(여기서부턴 선택사항)

도킹상태에서도 카메라 활성화하기

ssh ubuntu@로봇ip주소 로 로봇에 접속한 후

```bash
cd /opt/ros/humble/share/turtlebot4_bringup/config

#원본 백업
sudo cp turtlebot4.yaml turtlebot4_origin.yaml

#설정
sudo nano turtlebot4.yaml
  
power_saver: true  → false로 수정

sudo reboot
```

### SLAM

#### 1. Undocking

```bash
#Undock the robot if not undocked; enter your robot namespace
ros2 action send_goal /robot<n>/undock irobot_create_msgs/action/Undock "{}"
```

#### 2. SLAM 및 Rviz 키기

```bash
# SLAM
ros2 launch turtlebot4_navigation slam.launch.py namespace:=/robot<n>

# Rviz
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot<n>
```

#### 3 - 1. 키보드 제어로 돌아다니며 맵 만들기(권장)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot<n>/cmd_vel
```

#### 3 - 2. 자동항해로 돌아다니며 맵 만들기(되긴 하지만 키보드가 훨씬 더 나음.)

```bash
# map, costmap, planner, controller 등 활성화
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=robot<n>

# explore_lite launch
ros2 launch explore_lite explore.launch.py namespace:=/robot<n>
```

#### 4. Rviz상에서 적당히 맵이 완성되었으면 저장하기

```bash
# 저장할 폴더 생성(굳이 이 경로가 아니어도 됨.)
mkdir -p $HOME/turtlebot4_ws/maps

# 위 폴더로 이동
cd $HOME/turtlebot4_ws/maps

# 맵 저장(<map_name>은 원하는 이름으로 저장.)
ros2 run nav2_map_server map_saver_cli -f "<map_name>" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/robot<n> 
```

해당 폴더에 pgm과 yaml파일이 생성되면 성공.

### navigation 활용해보기

시뮬레이션 시간 설정을 해제방법(선택사항. 필수 아님.)

```bash
# nav2를 실행하기 위한 파라미터 파일이 저장된 config 디렉토리 이동
cd ~/turtlebot4_ws/src/turtlebot4/turtlebot4_navigation/config

sudo nano localization.yaml
모든 use_sim_time=True을 찾아 다음과 같이 False로 설정:
use_sim_time=False

sudo nano nav2.yaml
모든 use_sim_time=True을 찾아 다음과 같이 False로 설정:
use_sim_time=False

# 변경 후 확인: 모두 False면 정상
grep -rn "use_sim_time" localization.yaml nav2.yaml 
```

#### 1. localization

```bash
# <map_directory>는 아까 만든 맵의 경로, <map_name>은 아까 만든 맵의 이름.
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot<n> map:=<map_directory>/<map_name>.yaml
```

#### 2. Rviz 실행

```bash
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot<n>
```

#### 3. 출발지 및 초기 방향 설정

Rviz에서 2D Pose Estimate 버튼을 클릭하고 드래그하여 초기 위치와 방향을 설정한다.

#### 4. navigation 실행

```bash
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot<n>
```

