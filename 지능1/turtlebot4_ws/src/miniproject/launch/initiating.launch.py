from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    yolo_node = Node(
        package='miniproject',
        executable='img_yolo_higher',
        output='screen',
        emulate_tty=True
    )

    beep_node = Node(
        package='miniproject',
        executable='beep_test',
        output='screen',
        emulate_tty=True
    )

    follow_node = Node(
        package='miniproject',
        executable='follow_waypoints_stop',
        output='screen',
        emulate_tty=True
    )

    depth_node = Node(
        package='miniproject',
        executable='infinite_tracking',
        output='screen',
        emulate_tty=True
    )

    back_home_node = Node(
        package='miniproject',
        executable='back_home',
        output='screen',
        emulate_tty=True
    )

    finale_node = Node(
        package='miniproject',
        executable='finale',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([

        yolo_node,
        beep_node,
        follow_node,

        # follow 종료 -> infinite_tracking 실행
        RegisterEventHandler(
            OnProcessExit(
                target_action=follow_node,
                on_exit=[
                    LogInfo(
                        msg='follow_waypoints_stop finished. Starting infinite_tracking...'
                    ),
                    depth_node
                ]
            )
        ),

        # infinite_tracking 종료 -> back_home 실행
        RegisterEventHandler(
            OnProcessExit(
                target_action=depth_node,
                on_exit=[
                    LogInfo(
                        msg='infinite_tracking finished. Starting back_home...'
                    ),
                    back_home_node
                ]
            )
        ),

        # back_home 종료 -> finale 실행
        RegisterEventHandler(
            OnProcessExit(
                target_action=back_home_node,
                on_exit=[
                    LogInfo(
                        msg='back_home finished. Starting finale...'
                    ),
                    finale_node
                ]
            )
        ),
    ])