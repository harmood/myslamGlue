"""在迷宫世界中启动 Gazebo 并把四轮小车生成进去（仿真主入口）。

一次启动包含四部分：
    1. maze_world 的世界文件（Gazebo 仿真环境本身）；
    2. robot_state_publisher（URDF -> TF 链）；
    3. ros_gz_bridge（Gazebo 与 ROS 2 之间的话题桥接，见 config/ros_gz_bridge.yaml）；
    4. camera_relay / lidar_relay（把原始传感器话题中继成 RViz、SLAM 使用的干净话题）。

启动方式：
    ros2 launch maze_bot spawn_maze.launch.py
    ros2 launch maze_bot spawn_maze.launch.py x:=-4.0 y:=-4.0 yaw:=1.57
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_bot')
    # 迷宫世界由 maze_world 包提供（地面、墙体、光照、物理参数等）
    world_launch = os.path.join(
        get_package_share_directory('maze_world'), 'launch', 'maze.launch.py')
    default_model = os.path.join(pkg_share, 'urdf', 'maze_bot.urdf.xacro')
    # 桥接表：Gazebo 话题与 ROS 2 话题的对应关系全在这个文件里
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')

    # xacro -> URDF 文本；ParameterValue 声明为字符串，避免 launch 按 YAML 解析
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    # 直接复用 maze_world 的启动文件，把 Gazebo 世界拉起来
    maze_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
    )

    # 由 URDF 描述机器人，并根据 /joint_states 发布各连杆 TF。
    # use_sim_time=True 让它使用 Gazebo 的 /clock，时间戳与仿真同步。
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
    )

    # 把机器人实体插入 Gazebo：从 /robot_description 话题读取模型，
    # -name 指定实体名（后续 /model/maze_bot/... 话题名依赖它），
    # -x/-y/-z/-Y 是初始位姿（默认落在迷宫左下角附近）。
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_maze_bot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'maze_bot',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # 话题桥接节点：按 bridge_config 逐条建立 Gazebo <-> ROS 2 的双向通道。
    # 没有它，ROS 2 侧既收不到 /odom、/tf、图像、雷达，也发不出 /cmd_vel。
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'config_file': bridge_config},
        ],
    )

    # 相机中继：/camera/image_raw -> /camera/image。
    # 一是统一 QoS（BEST_EFFORT，避免大图像把队列堵死），
    # 二是提供 /camera_enable 开关，方便临时关掉图像转发。
    camera_relay = Node(
        package='maze_bot',
        executable='camera_relay.py',
        name='camera_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 雷达中继：/lidar/scan_raw -> /scan，并把 frame_id 统一成 lidar_link，
    # 使 RViz 与 SLAM 拿到的扫描数据与 TF 树一致。
    lidar_relay = Node(
        package='maze_bot',
        executable='lidar_relay.py',
        name='lidar_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        # 允许用命令行覆盖模型文件路径
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Absolute path to robot xacro file',
        ),
        # 机器人初始位姿；默认 (-4.8, -4.8) 位于迷宫一角，z=0.05 抬离地面
        # 以免生成瞬间与地面穿插导致弹飞，yaw 为绕 Z 轴的初始朝向
        DeclareLaunchArgument('x', default_value='-4.8', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='-4.8', description='Spawn y [m]'),
        DeclareLaunchArgument('z', default_value='0.05', description='Spawn z [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]'),
        maze_world,
        robot_state_publisher,
        bridge,
        camera_relay,
        lidar_relay,
        # 延迟 3 秒再插入机器人：等 Gazebo 完成加载、/robot_description 就绪，
        # 否则 create 节点可能取不到模型或找不到世界，导致生成失败。
        TimerAction(period=3.0, actions=[spawn_robot]),
    ])