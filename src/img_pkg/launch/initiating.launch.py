from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='img_pkg',
            executable='img_yolo',
            namespace='robot3',
            output='screen'
        ),

    ])