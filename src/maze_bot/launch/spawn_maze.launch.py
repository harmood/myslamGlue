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
    world_launch = os.path.join(
        get_package_share_directory('maze_world'), 'launch', 'maze.launch.py')
    default_model = os.path.join(pkg_share, 'urdf', 'maze_bot.urdf.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    maze_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
    )

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

    camera_relay = Node(
        package='maze_bot',
        executable='camera_relay.py',
        name='camera_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    lidar_relay = Node(
        package='maze_bot',
        executable='lidar_relay.py',
        name='lidar_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Absolute path to robot xacro file',
        ),
        DeclareLaunchArgument('x', default_value='-4.8', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='-4.8', description='Spawn y [m]'),
        DeclareLaunchArgument('z', default_value='0.05', description='Spawn z [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]'),
        maze_world,
        robot_state_publisher,
        bridge,
        camera_relay,
        lidar_relay,
        TimerAction(period=3.0, actions=[spawn_robot]),
    ])