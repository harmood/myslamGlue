"""启动键盘遥控节点 maze_teleop/teleop_keyboard。

启动方式：
    ros2 launch maze_teleop teleop.launch.py
    ros2 launch maze_teleop teleop.launch.py input_mode:=terminal   # 强制终端模式

为什么用 ExecuteProcess + bash 手动拼命令行，而不是 launch_ros 的 Node：
    键盘遥控必须真正拿到用户的终端，而 launch 默认会把节点 stdin 接到 /dev/null；
    这里用 bash 的 ``<> /dev/tty`` 把节点标准输入重定向到当前控制终端，
    节点才能读到按键（对应 teleop_keyboard.py 的 open_terminal / run_terminal）。
    同时用 bash -c 还能保证节点运行在前台，Ctrl+C 能直接作用到它。

所有可调参数都通过 --ros-args -p 传递，节点内的参数名与这里保持一致。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 每个参数先取 LaunchConfiguration，再在下面的命令行里展开为 -p 赋值
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

    # 命令行末尾的 <> /dev/tty 是关键：让节点的标准输入/输出都指向控制终端，
    # 这样 teleop_keyboard 才能读按键，也才能打印 HELP 与状态提示
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
        # 节点日志直接打到终端，用户能看到当前是 evdev 还是终端模式
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
        # 以下三个超时参数默认 0 表示“自动”：节点会根据系统键盘连发参数自行推算
        # （见 teleop_keyboard.py 的 detect_repeat_settings 与 __init__ 中的 auto_*）
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