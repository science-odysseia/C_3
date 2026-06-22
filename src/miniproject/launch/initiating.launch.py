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
        package='turtlebot4_beep',
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

    return LaunchDescription([

        yolo_node,
        beep_node,
        follow_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=follow_node,
                on_exit=[
                    LogInfo(msg='follow_waypoints_stop finished. Starting depth_to_nav_goal...'),
                    depth_node
                ]
            )
        ),
    ])