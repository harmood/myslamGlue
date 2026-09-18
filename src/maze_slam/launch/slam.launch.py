import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from maze_slam.resource_paths import default_map_dir


def generate_launch_description():
    map_dir = default_map_dir()
    database_path = os.path.join(map_dir, 'rtabmap.db')

    enabled = LaunchConfiguration('enabled')

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

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_footprint',
            'map_frame_id': 'map',
            'odom_frame_id': 'maze_bot/odom',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'approx_sync': True,
            'sync_queue_size': 30,
            'wait_for_transform': 0.5,
            'database_path': database_path,
            'Rtabmap/DetectionRate': '2.0',
            'Mem/IncrementalMemory': 'true',
            'Grid/FromDepth': 'false',
            'Grid/RayTracing': 'true',
        }],
        remappings=[
            ('rgb/image', '/slam/image'),
            ('rgb/camera_info', '/slam/camera_info'),
            ('depth/image', '/slam/depth_image'),
            ('depth/camera_info', '/slam/depth_camera_info'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='false',
                              description='Start SLAM mapping immediately'),
        slam_manager,
        rtabmap,
    ])