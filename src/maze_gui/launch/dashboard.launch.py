from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maze_gui',
            executable='maze_dashboard',
            name='maze_gui',
            output='screen',
        ),
    ])