"""启动 RTAB-Map 建图节点与 SLAM 管理器。

启动方式：
    ros2 launch maze_slam slam.launch.py
    ros2 launch maze_slam slam.launch.py enabled:=true   # 启动即开始建图

涉及两个节点：
    slam_manager：转发 Gazebo 相机图像/深度给 rtabmap，并处理建图开关与保存；
    rtabmap（rtabmap_slam）：真正的视觉 SLAM，输出 /map 与点云地图。

建图默认关闭（enabled=false）：rtabmap 启动后处于暂停态，等 GUI 或命令行把
/slam_enable 置 true 再开始，避免无人操作时白白累积数据库。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from maze_slam.resource_paths import default_map_dir


def generate_launch_description():
    # 复用 resource_paths 的解析逻辑，保证 slam_manager 保存地图与 rtabmap
    # 的数据库落在同一目录，便于整体备份/迁移
    map_dir = default_map_dir()
    database_path = os.path.join(map_dir, 'rtabmap.db')

    enabled = LaunchConfiguration('enabled')

    # SLAM 管理器：use_sim_time 让它与 Gazebo 的 /clock 一致，
    # 否则数据时间戳会对不上而被判为过期丢弃
    slam_manager = Node(
        package='maze_slam',
        executable='slam_manager',
        name='slam_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'enabled': enabled,
            'map_dir': map_dir,
        }],
    )

    # RTAB-Map 视觉 SLAM 主节点。
    # 注意：rtabmap 的参数值必须写成字符串（如 'true'、'2.0'），它按字符串解析
    # 配置；若写成 YAML 的 bool/float 会导致启动即 abort。
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # 坐标系：里程计由 Gazebo 插件发布、带模型前缀（maze_bot/odom），
            # 车体基座为 base_footprint，地图固定坐标系为 map
            'frame_id': 'base_footprint',
            'map_frame_id': 'map',
            'odom_frame_id': 'maze_bot/odom',
            # 输入：同时使用彩色与深度图（slam_manager 转发的 /slam/*）
            'subscribe_depth': True,
            'subscribe_rgb': True,
            # 不使用激光：本包是纯视觉方案，雷达不参与建图
            'subscribe_scan': False,
            # 彩图与深度来自不同话题，用近似时间同步而非严格同步，
            # 队列放 30 帧以容忍 ros_gz_bridge 的抖动
            'approx_sync': True,
            'sync_queue_size': 30,
            'wait_for_transform': 0.5,
            # 数据库与地图同目录，便于连同地图文件一起备份
            'database_path': database_path,
            # 以下为 rtabmap 内部参数，同样必须是字符串形式
            'Rtabmap/DetectionRate': '2.0',
            'Mem/IncrementalMemory': 'true',
            'Grid/FromDepth': 'false',
            'Grid/RayTracing': 'true',
        }],
        # 把 rtabmap 期望的输入话题重映射到 slam_manager 转发出的 /slam/*；
        # 相机内参也由 slam_manager 合成（ros_gz_bridge 不桥接 camera_info）
        remappings=[
            ('rgb/image', '/slam/image'),
            ('rgb/camera_info', '/slam/camera_info'),
            ('depth/image', '/slam/depth_image'),
            ('depth/camera_info', '/slam/depth_camera_info'),
        ],
    )

    return LaunchDescription([
        # 建图开关初值：默认 false（暂停态），需要时用 enabled:=true 覆盖
        DeclareLaunchArgument('enabled', default_value='false',
                              description='Start SLAM mapping immediately'),
        # 先起 slam_manager 再起 rtabmap：前者是数据源，后者上线即可收到数据
        slam_manager,
        rtabmap,
    ])