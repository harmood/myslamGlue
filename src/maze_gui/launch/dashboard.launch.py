"""启动 maze_bot 控制台窗口。

启动方式：
    ros2 launch maze_gui dashboard.launch.py

该进程本身是一个 ROS 2 节点（订阅 /odom、/joint_states 等，发布开关指令），
同时创建 Tkinter 窗口，因此需要在有图形界面的环境（已设置 DISPLAY）中运行。
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # executable 名字取自 setup.py 中 console_scripts 注册的 maze_dashboard；
        # name 指定节点名（与 dashboard.py 里 super().__init__('maze_gui') 一致）。
        # output='screen' 让日志（例如无 DISPLAY 时的报错）直接打印到终端。
        Node(
            package='maze_gui',
            executable='maze_dashboard',
            name='maze_gui',
            output='screen',
        ),
    ])