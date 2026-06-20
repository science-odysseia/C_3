from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='miniproject',
            executable='img_yolo_higher',
            namespace='robot3',
            output='screen'
        ),
        Node(
            package='turtlebot4_beep',
            executable='beep_test',
            namespace='robot3',
            output='screen'
        ),
        Node(
            package='miniproject',
            executable='nav_to_pose',
            namespace='robot3',
            output='screen'
        ),

    ])