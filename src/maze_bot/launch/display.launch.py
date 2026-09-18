"""四轮小车模型的可视化演示（不启动 Gazebo）。

只运行 robot_state_publisher + joint_state_publisher + RViz2，
用于在不启动仿真的情况下快速检查 URDF 是否正确、模型外形是否符合预期。

启动方式：
    ros2 launch maze_bot display.launch.py
    ros2 launch maze_bot display.launch.py use_sim_time:=true   # 配合仿真时钟
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 安装后 URDF 与 RViz 配置都放在 share/maze_bot/ 下，
    # 用 get_package_share_directory 取绝对路径，避免依赖当前工作目录。
    pkg_share = get_package_share_directory('maze_bot')
    default_model = os.path.join(pkg_share, 'urdf', 'maze_bot.urdf.xacro')
    default_rviz = os.path.join(pkg_share, 'rviz', 'maze_bot.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 用 xacro 把 .xacro 展开成纯 URDF 文本。
    # 必须包一层 ParameterValue(value_type=str)：Command 替换的结果默认会被
    # launch 当作 YAML 解析，而 URDF 是普通文本，不声明类型会解析失败。
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    # 机器人状态发布器：读取 robot_description，结合 /joint_states 计算并发布
    # 各连杆之间的 TF（base_footprint -> base_link -> 轮子/相机/雷达），
    # RViz 的 RobotModel 与 TF 显示都依赖它。
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # 关节状态发布器：没有 Gazebo 时，它按 URDF 中的关节定义发布 /joint_states，
    # 让模型能显示出来（否则轮子、转向柱没有位姿，模型不完整）。
    # 在 spawn_maze.launch.py 的仿真场景中，这个角色由 Gazebo 的
    # JointStatePublisher 插件承担，所以这里只用于纯模型预览。
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # RViz2：-d 指定配置文件，省去每次手动添加 Grid/TF/RobotModel 显示项。
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        # 允许通过命令行覆盖机器人模型文件路径
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Absolute path to robot xacro file',
        ),
        # 纯预览时用系统时间即可；接入 Gazebo 时需传 true 以使用仿真时钟
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        robot_state_publisher,
        joint_state_publisher,
        rviz2,
    ])