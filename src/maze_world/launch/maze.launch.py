"""启动迷宫 Gazebo 世界的独立入口（不生成机器人）。

它做两件事：
    1. 复用 ros_gz_sim 官方 gz_sim.launch.py 拉起 Gazebo 并加载本包的
       worlds/maze.world；gz_args 里的 -r 表示加载后立即运行、-v 3 是日志级别；
       on_exit_shutdown 让 Gazebo 退出时顺带结束整个 launch（避免留下孤儿进程）。
    2. 启动 RViz2 并加载本包的 rviz/maze.rviz，供可视化查看。

该文件也被 maze_bot 的 spawn_maze.launch.py 复用（作为世界部分的入口），
所以这里只负责“世界 + 可视化”，机器人模型由调用方另行生成。

启动方式：
    ros2 launch maze_world maze.launch.py
    ros2 launch maze_world maze.launch.py use_sim_time:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 安装后世界与 RViz 配置都在 share/maze_world/ 下，
    # 用 get_package_share_directory 取绝对路径，避免依赖当前工作目录
    pkg_share = get_package_share_directory('maze_world')

    # 仿真场景必须用 Gazebo 的 /clock 对齐时间戳，所以默认 true
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(pkg_share, 'worlds', 'maze.world')
    rviz_config = os.path.join(pkg_share, 'rviz', 'maze.rviz')

    # 直接复用 ros_gz_sim 的启动文件，避免自己维护 Gazebo 的启动细节
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 3 {world}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # RViz2：-d 指定配置，省去每次手动添加显示项；
    # use_sim_time 与仿真时钟保持一致，否则 TF/传感器时间戳会对不上
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        # 暴露 use_sim_time 供命令行或上层 launch 覆盖
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
        gz_sim,
        rviz2,
    ])
