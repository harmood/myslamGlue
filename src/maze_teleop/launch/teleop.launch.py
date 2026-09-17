from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    linear_speed = LaunchConfiguration('linear_speed')
    max_steering = LaunchConfiguration('max_steering')
    steering_rate = LaunchConfiguration('steering_rate')
    wheelbase = LaunchConfiguration('wheelbase')
    steering_topic = LaunchConfiguration('steering_topic')
    release_timeout = LaunchConfiguration('release_timeout')
    hold_delay = LaunchConfiguration('hold_delay')
    tap_pulse = LaunchConfiguration('tap_pulse')
    debug_keys = LaunchConfiguration('debug_keys')
    input_mode = LaunchConfiguration('input_mode')

    teleop = ExecuteProcess(
        cmd=[
            'bash', '-c',
            [
                'ros2 run maze_teleop teleop_keyboard --ros-args'
                ' -p cmd_vel_topic:=', cmd_vel_topic,
                ' -p linear_speed:=', linear_speed,
                ' -p max_steering:=', max_steering,
                ' -p steering_rate:=', steering_rate,
                ' -p wheelbase:=', wheelbase,
                ' -p steering_topic:=', steering_topic,
                ' -p release_timeout:=', release_timeout,
                ' -p hold_delay:=', hold_delay,
                ' -p tap_pulse:=', tap_pulse,
                ' -p debug_keys:=', debug_keys,
                ' -p input_mode:=', input_mode,
                ' <> /dev/tty',
            ],
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='Velocity command topic'),
        DeclareLaunchArgument('linear_speed', default_value='0.2',
                              description='Initial linear speed [m/s]'),
        DeclareLaunchArgument('max_steering', default_value='0.5',
                              description='Maximum front wheel steering angle [rad]'),
        DeclareLaunchArgument('steering_rate', default_value='2.0',
                              description='Steering angle slew rate [rad/s]'),
        DeclareLaunchArgument('wheelbase', default_value='0.30',
                              description='Distance between front and rear axles [m]'),
        DeclareLaunchArgument('steering_topic', default_value='/steering_position',
                              description='Front steering position command topic'),
        DeclareLaunchArgument('release_timeout', default_value='0.0',
                              description='Seconds without key repeat before stopping, 0=auto'),
        DeclareLaunchArgument('hold_delay', default_value='0.0',
                              description='Grace period for first keyboard auto-repeat, 0=auto'),
        DeclareLaunchArgument('tap_pulse', default_value='0.0',
                              description='Short pulse per tap in compat mode, 0=disabled'),
        DeclareLaunchArgument('debug_keys', default_value='false',
                              description='Log raw key sequences for diagnostics'),
        DeclareLaunchArgument('input_mode', default_value='auto',
                              description='Input mode: auto, terminal or evdev (global keyboard)'),
        teleop,
    ])