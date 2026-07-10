from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 두산 로봇 브링업 (즉시 실행 시작)
    m0609_rg2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('m0609_rg2_bringup'),
                'launch',
                'bringup.launch.py'
            ])
        ),
        launch_arguments={
            'mode': 'real',
            'host': '192.168.1.100',
            'port': '12345',
            'model': 'm0609'
        }.items()
    )

    # 2. 웹 서버 노드 (on_exit 추가로 크래시 전파 방지)
    robot_ui = Node(
        package='lego22',
        executable='lego22',
        name='web_server',
        output='screen',
        on_exit=[] # 👈 이 노드가 죽어도 다른 노드를 죽이지 않음
    )

    # 3. 로봇 블록 조립 제어 노드 (on_exit 추가로 크래시 전파 방지)
    robot_move_blocks = Node(
        package='rokey',
        executable='corner',
        name='rokey_move',
        output='screen', # 👈 에러 로그를 터미널에 뿌려주기 위해 필수
        on_exit=[] # 👈 이 노드가 죽어도 로봇 드라이버를 죽이지 않음
    )

    # 10초 뒤 가동
    delayed_nodes = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="⌛ 10초 대기 완료! 웹 서버와 제어 노드를 기동합니다."),
            robot_ui,
            robot_move_blocks
        ]
    )

    return LaunchDescription([
        m0609_rg2_bringup,
        delayed_nodes,
    ])
