"""maze_bot 键盘遥控节点（maze_teleop）。

这个模块存在的意义：仿真里的 maze_bot 需要人工操控去探索迷宫，而 Gazebo/RViz
本身不提供遥控入口，因此单独做一个小节点，把键盘输入翻译成机器人运动指令。

支持三种输入后端，由 main() 按以下顺序选取（参数 input_mode 可强制指定）：
  1. Linux evdev 全局键盘（input_mode 为 auto/evdev 且找到可用键盘设备时优先）：
     直接读 /dev/input/event*，因此不依赖终端窗口焦点，切到 RViz/Gazebo 窗口后
     依然能遥控；代价是拿不到“终端自己的按键上下文”，只能自行跟踪修饰键状态
     （见 EVDEV_MODIFIERS 与 handle_evdev）。
  2. 终端 raw 模式 + kitty 键盘协议（在交互式终端运行且终端支持 kitty 时）：
     用 termios 把 stdin 设为 raw 逐字符读取；再通过 CSI ?u 主动查询让终端上报
     增强按键序列（形如 keycode;modifiers:event），能区分按下/重复/抬起，
     因此前进与转向两个通道可以同时按住（查询见 enable_kitty_protocol，
     解析见 handle_kitty_event）。
  3. 终端兼容模式（终端不支持 kitty 协议时的兜底）：
     只能拿到字符流，无法感知按键何时抬起，于是用“按键连发”来推断——同一方向
     字符持续重复即视为仍按住，超时没有重复就自动停止（见 move_legacy/publish）。

按键功能：
    w/s（或上下方向键）前进/后退，a/d（或左右方向键）前轮转向，空格急停，
    F12 暂停/恢复遥控，+/-（含 =、小键盘）+ 调节速度，c 相机开关，
    m SLAM 建图开关，q 退出终端遥控；GUI 里按 E 也能开关遥控。

与其它节点的接口（话题名均可通过参数覆盖）：
    发布 /cmd_vel（geometry_msgs/Twist）：按自行车模型把线速度与前轮转角折算成
        线速度/角速度；后轮驱动、前轮转向，与 URDF 中 maze_bot 的关节命名一致。
    发布 /steering_position（std_msgs/Float64）：前轮目标转角，供控制器跟踪。
    发布 /teleop_enable、/camera_enable、/slam_enable（Bool）：下发开关指令。
    订阅 /teleop_state、/camera_state、/slam_state（Bool）：回读各节点状态用于显示。
"""

# 标准库依赖说明：
#   ctypes + fcntl 用于 ioctl（查询键盘设备支持哪些按键）；termios 把终端切到 raw；
#   glob 扫描 /dev/input/event*；select 做非阻塞多路等待；struct 解析 evdev 二进制
#   事件；subprocess 读取系统键盘连发参数；signal 自行接管退出信号。
import ctypes
import errno
import fcntl
import glob
import math
import os
import re
import select
import signal
import struct
import subprocess
import sys
import termios
import time

# ROS 2 部分：Twist 是速度指令，Bool 用于各开关，Float64 用于前轮转角。
# 显式导入 SignalHandlerOptions，稍后 init() 时禁用 rclpy 自带的信号处理：
# 自带处理会在 Ctrl+C 时直接抛 KeyboardInterrupt，可能打断随后的终端属性恢复。
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64

# 终端模式下打印给用户的按键速查表（纯提示文本，与 GUI 的 CONTROLS 表内容对应）
HELP = """
==================================================
  maze_bot 键盘遥控
--------------------------------------------------
  前后移动： w / ↑ 前进      s / ↓ 后退
  前轮转向： a / ← 左转      d / → 右转
  空格 立即停止          q 退出
  + / - 加快 / 减慢速度   c 相机开关   m SLAM建图开关
--------------------------------------------------"""

# 向终端请求的 kitty 协议能力位：disambiguate(1) | report event types(2) | report all keys(8)。
# “report event types”是关键——它让终端区分按下(1)/重复(2)/抬起(3)；
# 没有它就只能退回按键连发推断的兼容模式。
KITTY_FLAGS = 11  # disambiguate(1) | report event types(2) | report all keys(8)

# 键码 -> 方向符号（+1 前进/左转，-1 后退/右转）。同一张表里混有三种编码：
# ASCII 码（终端字符流的按键码）、kitty 功能键私有码（57352 起）、
# xterm 小键盘码（57417 起）；末尾大写 W/S、A/D 用于兼容实现不完整的终端——
# 这类终端不发 kitty 序列，只会把按键当作大写字母上报。
LINEAR_KEYS = {119: 1.0, 57352: 1.0, 57419: 1.0,    # w, up, keypad up
               115: -1.0, 57353: -1.0, 57420: -1.0,  # s, down, keypad down
               87: 1.0, 83: -1.0}                    # W, S（兼容实现不完整的终端）
ANGULAR_KEYS = {97: 1.0, 57350: 1.0, 57417: 1.0,    # a, left, keypad left
                100: -1.0, 57351: -1.0, 57418: -1.0,  # d, right, keypad right
                65: 1.0, 68: -1.0}                    # A, D（兼容实现不完整的终端）
# SS3/CSI 方向键结束符 A/B/C/D 对应的 kitty 键码。
# 注意 B=下、C=右、D=左，与 xterm 中 C/D 的含义一致，最终映射为 s/d/a
LETTER_KEYS = {'A': 57352, 'B': 57353, 'C': 57351, 'D': 57350}
# 加速/减速键：= 与 + 同义、连字符与下划线同义，另外兼容小键盘
SPEED_UP_KEYS = {61, 43, 107}     # = + keypad-plus
SPEED_DOWN_KEYS = {45, 95, 109}   # - _ keypad-minus
QUIT_KEYS = {113}                 # q

# evdev（Linux input 子系统）相关常量与键码表。
# 键码来自 <linux/input-event-codes.h>：如 103 上箭头、105 左箭头等
EV_KEY = 0x01
# Linux input 事件二进制布局：秒、微秒、类型、码、值（即 long long, long long, H, H, int）
EVENT_STRUCT = struct.Struct('llHHi')
EVDEV_LINEAR = {17: 1.0, 103: 1.0, 31: -1.0, 108: -1.0}    # w/up, s/down
EVDEV_ANGULAR = {30: 1.0, 105: 1.0, 32: -1.0, 106: -1.0}   # a/left, d/right
EVDEV_SPEED_UP = {13, 78}         # = keypad-plus
EVDEV_SPEED_DOWN = {12, 74}       # - keypad-minus
EVDEV_SPACE = 57
# 修饰键（Ctrl/Alt/Super/Shift）：组合键不应触发单键功能。
# 全局键盘模式下，Ctrl+C、Ctrl+M 等组合键会被拆成普通按键上报，
# 若不加区分，Ctrl+C（终端中断）会被误当成“相机开关”。
# 集合内容依次为：29 左Ctrl、97 右Ctrl、56 左Alt、100 右Alt、
# 125 左Super、126 右Super、42 左Shift、54 右Shift（左右都记，避免漏判导致误触发）
EVDEV_MODIFIERS = {29, 97, 56, 100, 125, 126, 42, 54}


def _has_bit(bitmask, code):
    """检查设备能力位图（按位打包）中第 code 位是否置位。"""
    return bitmask[code // 8] & (1 << (code % 8))


def _eviocgbit(ev, length):
    """按 Linux 惯用法拼出 EVIOCGBIT(ev, length) 的 ioctl 请求号。"""
    return (2 << 30) | (length << 16) | (ord('E') << 8) | (0x20 + ev)


# 直接调用 C 库 ioctl：Python 标准库没有暴露 EVIOCGBIT 这类请求，
# use_errno=True 保证失败后能取到准确的 errno 用于生成异常信息
LIBC = ctypes.CDLL('libc.so.6', use_errno=True)


def _ioctl_buffer(fd, request, size):
    """执行 ioctl 并把返回的定长字节缓冲读出来；失败抛 OSError。"""
    buf = ctypes.create_string_buffer(size)
    if LIBC.ioctl(fd, ctypes.c_ulong(request), buf) < 0:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err))
    return buf.raw


def find_keyboards():
    """扫描 /dev/input/event*，返回 {设备路径: 已打开的 fd} 供全局键盘模式使用。

    只认“真正的键盘”：设备既要支持按键事件（EV_KEY），又要同时具备
    w/a/s/d（码 17/30/31/32）四个常用键。这样可排除鼠标、电源键等
    会出现在 /dev/input 下、却无法用于遥控的设备。
    打不开（权限不足/已被占用）的设备直接跳过，不影响其余设备。
    """
    devices = {}
    for path in sorted(glob.glob('/dev/input/event*')):
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            continue
        try:
            types = _ioctl_buffer(fd, _eviocgbit(0, 8), 8)
            keys = _ioctl_buffer(fd, _eviocgbit(EV_KEY, 96), 96)
            if (types[0] & EV_KEY) and all(_has_bit(keys, code)
                                           for code in (17, 30, 31, 32)):
                devices[path] = fd
                continue
        except OSError:
            pass
        os.close(fd)
    return devices


def detect_repeat_settings():
    """检测系统键盘连发参数，返回 (首次延迟, 连发间隔)，失败返回 (None, None)"""
    # 兼容模式要靠系统的“按住→连发”行为来判定按键仍被按住，
    # 所以必须先问清连发节奏，才能算出合理的超时阈值（见 __init__ 里的 auto_* 计算）。
    # 首选 GNOME 的设置（单位 ms，除以 1000 换成秒）
    try:
        delay = int(subprocess.run(
            ['gsettings', 'get', 'org.gnome.desktop.peripherals.keyboard', 'delay'],
            capture_output=True, text=True, timeout=2).stdout.strip()) / 1000.0
        interval = int(subprocess.run(
            ['gsettings', 'get', 'org.gnome.desktop.peripherals.keyboard', 'repeat-interval'],
            capture_output=True, text=True, timeout=2).stdout.strip()) / 1000.0
        if delay > 0 and interval > 0:
            return delay, interval
    except Exception:
        # 非 GNOME 桌面或命令不存在时继续尝试下一种方式，不视为错误
        pass
    # 回退到 X11 通用的 xset q：delay 单位 ms，repeat rate 单位为“次/秒”需取倒数
    try:
        out = subprocess.run(['xset', 'q'], capture_output=True, text=True, timeout=2).stdout
        match = re.search(r'auto repeat delay:\s+(\d+)\s+repeat rate:\s+(\d+)', out)
        if match:
            return int(match.group(1)) / 1000.0, 1.0 / int(match.group(2))
    except Exception:
        pass
    # 两种方式都失败：返回 None，让调用方改用参数默认值
    return None, None


class MazeTeleop(Node):
    """键盘遥控节点：把三种后端解析出的按键事件翻译成速度/转角指令。

    核心是“两个独立通道”模型：linear（前进后退）与 angular（前轮转向），
    每个通道只保存一个方向符号，由 publish() 定时折算成 Twist 与转角指令。
    兼容模式与 kitty 模式用不同的状态容器保存通道方向：
        legacy —— 靠连发推断按键是否仍按住；
        held   —— 有明确的按下/抬起事件，按“后按的生效”排序。
    开关类按键（相机/SLAM/暂停）走话题与 GUI 双向同步，见 on_enable 等回调。
    """

    def __init__(self):
        super().__init__('maze_teleop')
        # 话题名与运动学参数全部声明为参数：launch 文件与命令行都能覆盖，
        # 默认值对应 teleop.launch.py 的默认参数
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('max_steering', 0.5)
        self.declare_parameter('steering_rate', 2.0)
        self.declare_parameter('wheelbase', 0.30)
        self.declare_parameter('steering_topic', '/steering_position')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('release_timeout', 0.25)
        self.declare_parameter('hold_delay', 0.7)
        self.declare_parameter('tap_pulse', 0.0)
        self.declare_parameter('debug_keys', False)
        self.declare_parameter('input_mode', 'auto')
        self.declare_parameter('arm_key', 88)
        self.declare_parameter('enable_topic', '/teleop_enable')
        self.declare_parameter('state_topic', '/teleop_state')
        self.declare_parameter('camera_enable_topic', '/camera_enable')
        self.declare_parameter('camera_state_topic', '/camera_state')
        self.declare_parameter('slam_enable_topic', '/slam_enable')
        self.declare_parameter('slam_state_topic', '/slam_state')

        # 拷贝常用参数到实例字段，避免每帧都走 get_parameter 查询
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.steering_rate = float(self.get_parameter('steering_rate').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        steering_topic = self.get_parameter('steering_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        topic = self.get_parameter('cmd_vel_topic').value
        self.debug_keys = bool(self.get_parameter('debug_keys').value)
        # arm_key 是 evdev 模式下“暂停/恢复遥控”的按键码（默认 88 = F12）
        self.arm_key = int(self.get_parameter('arm_key').value)
        # armed 为 True 才响应运动/开关按键；GUI 可通过 /teleop_enable 同步修改
        self.armed = True
        # 是否运行在 evdev 全局键盘模式（影响状态打印与退出提示）
        self.evdev = False
        # evdev 模式下当前按住的修饰键集合；组合键不触发单键功能（见 handle_evdev）
        self.modifiers = set()

        # 依系统连发节奏推导兼容模式的超时阈值。参数为 0 时表示“自动”：
        #   hold_delay —— 按下后允许多久没有连发仍认为有效，需覆盖首次连发延迟
        #   release_timeout —— 连发已经建立后，多久没有新字符判定为已松开；
        #     上限 0.25s 是为了让松开后的停车足够跟手
        self.delay, self.interval = detect_repeat_settings()
        auto_hold = self.delay + 0.12 if self.delay else 0.7
        auto_release = max(3.0 * self.interval, 0.08) if self.interval else 0.25
        auto_release = min(auto_release, 0.25)
        release_timeout = float(self.get_parameter('release_timeout').value)
        hold_delay = float(self.get_parameter('hold_delay').value)
        tap_pulse = float(self.get_parameter('tap_pulse').value)
        self.release_timeout = release_timeout if release_timeout > 0 else auto_release
        self.hold_delay = hold_delay if hold_delay > 0 else auto_hold
        # tap_pulse 为兼容模式下的“轻点脉冲”时长，默认 0 表示不启用
        self.tap_pulse = tap_pulse if tap_pulse > 0 else 0.0

        # 指令输出：Twist 给底盘控制器，Float64 给前轮转角控制器
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.steering_publisher = self.create_publisher(Float64, steering_topic, 10)
        # 周期上报 armed 状态，让后启动的 GUI 也能拿到当前值
        self.state_publisher = self.create_publisher(
            Bool, self.get_parameter('state_topic').value, 10)
        # 订阅 /teleop_enable：GUI 按钮或 E 键下发的暂停/恢复指令
        self.create_subscription(Bool, self.get_parameter('enable_topic').value,
                                 self.on_enable, 10)
        # 相机开关：下发指令到 /camera_enable，并订阅 /camera_state 回读真实状态
        self.camera_enable_publisher = self.create_publisher(
            Bool, self.get_parameter('camera_enable_topic').value, 10)
        self.create_subscription(
            Bool, self.get_parameter('camera_state_topic').value,
            self.on_camera_state, 10)
        # 初始按“开着”假设，随后由状态话题校正，避免刚启动就显示为关闭
        self.camera_enabled = True
        # SLAM 开关同理；建图默认关闭，需要用户按 m 才开启
        self.slam_enable_publisher = self.create_publisher(
            Bool, self.get_parameter('slam_enable_topic').value, 10)
        self.create_subscription(
            Bool, self.get_parameter('slam_state_topic').value,
            self.on_slam_state, 10)
        self.slam_enabled = False
        self.create_timer(1.0, self.publish_state)
        # 最近一次下发的指令与运动状态
        self.twist = Twist()
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self.steer = 0.0
        # 记录上次发布时刻，用于计算积分步长 dt（转角按速率限幅爬升）
        self.last_publish = time.monotonic()
        # 终端是否支持 kitty 键盘协议（由 enable_kitty_protocol 决定）
        self.kitty = False
        # 主循环退出条件：按 q / Ctrl+C / 收到退出信号都会置 False
        self.running = True
        # 终端输入解析缓冲区（一次 read 可能只读到半个转义序列）
        self.buf = b''

        # 兼容模式：按通道独立记录按键方向与时间戳
        self.legacy = {
            'linear': {'sign': 0.0, 'last': 0.0, 'repeat': False},
            'angular': {'sign': 0.0, 'last': 0.0, 'repeat': False},
        }
        # Kitty 模式：记录每个通道当前按住的键（有序，后按的生效）
        self.held = {'linear': {}, 'angular': {}}

        # 固定频率发布：保证没按键时也持续下发指令，控制器不会因话题静默而超时
        self.timer = self.create_timer(1.0 / publish_rate, self.publish)

    def publish(self):
        """定时把通道状态折算成速度/转角指令下发，并维护兼容模式的超时停车。"""
        now = time.monotonic()
        # 用单调时钟求两次发布的时间差作为积分步长，避免系统时间跳变影响转角爬升
        dt = max(now - self.last_publish, 0.0)
        self.last_publish = now
        # 兼容模式没有“抬起”事件，只能靠超时判断：太久没收到同方向连发就当作已松手。
        # 阈值分两段：首次连发尚未出现时用较宽松的 hold_delay，
        # 已进入连发状态后用较短的 release_timeout，让松手停车更跟手。
        for channel in ('linear', 'angular'):
            state = self.legacy[channel]
            if state['sign'] == 0.0:
                continue
            timeout = self.release_timeout if state['repeat'] else (
                self.tap_pulse if self.tap_pulse > 0 else self.hold_delay)
            if now - state['last'] > timeout:
                state['sign'] = 0.0
                state['repeat'] = False

        # 先取兼容模式结论，再用 kitty 模式的“当前按住的键”覆盖。
        # 两种模式互斥（kitty 模式不会写入 legacy），这样写可让 publish 无需分支。
        linear = self.legacy['linear']['sign']
        angular = self.legacy['angular']['sign']
        if self.held['linear']:
            linear = list(self.held['linear'].values())[-1]
        if self.held['angular']:
            angular = list(self.held['angular'].values())[-1]
        self.linear_cmd = linear
        self.angular_cmd = angular

        # 前轮转角按 steering_rate 限速逼近目标值，避免瞬间打死方向导致打滑/抖动
        target = angular * self.max_steering
        step = self.steering_rate * dt
        if self.steer < target:
            self.steer = min(self.steer + step, target)
        elif self.steer > target:
            self.steer = max(self.steer - step, target)

        # 先发布前轮目标转角（供关节控制器跟踪），再发布整车速度指令
        steering_msg = Float64()
        steering_msg.data = self.steer
        self.steering_publisher.publish(steering_msg)

        # 自行车模型：后轮驱动、前轮转向，转弯半径 R = wheelbase / tan(steer)，
        # 故角速度 omega = v / R = v * tan(steer) / wheelbase；
        # 转角为 0 时角速度自然也为 0（直行）
        velocity = linear * self.linear_speed
        self.twist = Twist()
        self.twist.linear.x = velocity
        self.twist.angular.z = velocity * math.tan(self.steer) / self.wheelbase
        self.publisher.publish(self.twist)

    def print_status(self, mode):
        """打印当前输入模式、连发时间参数与运动学参数，方便用户确认阈值是否合适。"""
        self.get_logger().info(f'键盘模式：{mode}')
        # 只有兼容模式才依赖系统连发参数，kitty/evdev 模式打印这行反而会误导
        if not self.kitty and not self.evdev:
            if self.delay:
                self.get_logger().info(
                    f'系统连发：延迟 {self.delay:.2f}s、间隔 {self.interval:.3f}s；'
                    f'松开判定 {self.release_timeout:.2f}s、按住宽限 {self.hold_delay:.2f}s')
            else:
                self.get_logger().info(
                    f'未检测到系统连发参数；松开判定 {self.release_timeout:.2f}s、'
                    f'按住宽限 {self.hold_delay:.2f}s')
        self.get_logger().info(
            f'线速度 {self.linear_speed:.2f} m/s，最大前轮转角 {self.max_steering:.2f} rad，'
            f'轴距 {self.wheelbase:.2f} m')

    def stop_all(self):
        """立即停止：清空两个方向通道。空格急停、暂停遥控、退出时都会调用。"""
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        # 必须同时清掉两种模式的容器，否则残留状态会在下一帧 publish 时“复活”
        for state in self.legacy.values():
            state['sign'] = 0.0
            state['repeat'] = False
        for channel in self.held:
            self.held[channel].clear()

    def scale_speed(self, factor):
        """按倍率调节线速度，并夹在 0.05~0.5 m/s 之间防止过慢或过快失控。"""
        self.linear_speed = min(max(self.linear_speed * factor, 0.05), 0.5)
        self.get_logger().info(
            f'线速度 {self.linear_speed:.2f} m/s，最大前轮转角 {self.max_steering:.2f} rad')

    def publish_state(self):
        """上报 armed 状态到 /teleop_state。"""
        msg = Bool()
        msg.data = self.armed
        self.state_publisher.publish(msg)

    def set_armed(self, armed):
        """切换遥控开关；暂停时立刻清零指令，避免“暂停后小车还在动”。"""
        self.armed = armed
        if not armed:
            self.stop_all()
        self.publish_state()
        if armed:
            self.get_logger().info('遥控已开启（F12 或 GUI 可再次暂停）')
        else:
            self.get_logger().info('遥控已暂停（F12 或 GUI 恢复）')

    def on_enable(self, msg):
        """收到 /teleop_enable：仅在与本地状态不一致时切换，避免消息回声导致反复打印。"""
        if bool(msg.data) != self.armed:
            self.set_armed(bool(msg.data))

    def on_camera_state(self, msg):
        """回读 /camera_state：以相机中继的实际状态为准，避免界面显示与实际不符。"""
        self.camera_enabled = bool(msg.data)

    def on_slam_state(self, msg):
        """回读 /slam_state：以 SLAM 管理器的实际状态为准。"""
        self.slam_enabled = bool(msg.data)

    def toggle_slam(self):
        """翻转建图开关：先本地翻转并复用为下发值，再发 /slam_enable 并打印提示。"""
        self.slam_enabled = not self.slam_enabled
        msg = Bool()
        msg.data = self.slam_enabled
        self.slam_enable_publisher.publish(msg)
        self.get_logger().info(
            'SLAM 建图已开启' if self.slam_enabled else 'SLAM 建图已关闭')

    def toggle_camera(self):
        """翻转相机开关：下发 /camera_enable，控制图像中继是否继续转发。"""
        self.camera_enabled = not self.camera_enabled
        msg = Bool()
        msg.data = self.camera_enabled
        self.camera_enable_publisher.publish(msg)
        self.get_logger().info(
            '相机已开启' if self.camera_enabled else '相机已关闭')

    # ---------- 事件处理 ----------

    def move_legacy(self, channel, sign):
        """兼容模式：记录方向与时刻；同一方向再次到达即视为“仍在按住”（连发）。"""
        state = self.legacy[channel]
        if state['sign'] == sign:
            # 相同方向的重复字符 = 系统连发，标记后方可用较短的 release_timeout 判松手
            state['repeat'] = True
        else:
            # 方向变了（含从停止变为移动），先按“刚按下”处理，给一次较长的宽限
            state['sign'] = sign
            state['repeat'] = False
        state['last'] = time.monotonic()

    def move_kitty(self, channel, keycode, sign, pressed):
        """Kitty 模式：维护该通道按住的键集合，并按“后按的生效”取当前方向。

        抬起时只移除对应键，因此同时按住 w 与 s（或 a 与 d）时，
        最后按下的那个决定方向，松开后自动回落到另一个仍按住的键，最终归零。
        """
        keys = self.held[channel]
        if pressed:
            # 先删再加，是为了让该键在 dict 中排到最后——即“最近按下”的语义
            keys.pop(keycode, None)
            keys[keycode] = sign
        else:
            keys.pop(keycode, None)
        if keys:
            sign = list(keys.values())[-1]
        else:
            sign = 0.0
        # 立即写入指令字段（下一帧 publish 也会重新求值），使状态即时可读
        if channel == 'linear':
            self.linear_cmd = sign
        else:
            self.angular_cmd = sign

    def handle_legacy_char(self, char):
        """兼容模式/普通字符路径：把单字符翻译成动作。

        每组按键都同时列出半角与全角（\\uffXX）两种写法，是为了兼容中文输入法
        开启时终端可能上报全角字符的情况；小写字母与大写字母同样并列。
        """
        # q 退出；\\x03 是 Ctrl+C（ETX），终端 raw 模式下不会自动产生 SIGINT，
        # 这里显式当作退出处理，让“Ctrl+C 退出”在两种模式下行为一致
        if char in ('q', 'Q', '\uff51', '\uff31', '\x03'):
            self.running = False
            return
        # 暂停状态下只响应退出键，其余按键全部忽略（但仍靠 publish 持续发零速度）
        if not self.armed:
            return
        # 全角 w/W（\uff57/\uff37）
        if char in ('w', 'W', '\uff57', '\uff37'):
            self.move_legacy('linear', 1.0)
        # 全角 s/S
        elif char in ('s', 'S', '\uff53', '\uff33'):
            self.move_legacy('linear', -1.0)
        # 全角 a/A
        elif char in ('a', 'A', '\uff41', '\uff21'):
            self.move_legacy('angular', 1.0)
        # 全角 d/D
        elif char in ('d', 'D', '\uff44', '\uff24'):
            self.move_legacy('angular', -1.0)
        # 全角 c/C
        elif char in ('c', 'C', '\uff43', '\uff23'):
            self.toggle_camera()
        # 全角 m/M
        elif char in ('m', 'M', '\uff4d', '\uff2d'):
            self.toggle_slam()
        # 半角空格与全角空格（\u3000）都算急停
        elif char in (' ', '\u3000'):
            self.stop_all()
        # +/= 与全角加号/等号，加速 25%
        elif char in ('+', '=', '\uff0b', '\uff1d'):
            self.scale_speed(1.25)
        # -/_ 与全角减号/下划线，减速 20%
        elif char in ('-', '_', '\uff0d', '\uff3f'):
            self.scale_speed(0.8)

    def handle_kitty_event(self, keycode, modifiers, event):
        """处理 kitty 协议上报的单次按键事件（形如 keycode;modifiers:event）。

        event：1=按下、2=重复（长按连发）、3=抬起。kitty 模式有真正的“抬起”事件，
        所以运动键必须把按下与抬起都交给 move_kitty 维护键集合，不能只处理按下。
        """
        # 长按产生的“重复”事件无需再触发一次动作（开关类按键尤其不能被连发反复翻转）
        if event == 2:
            return
        pressed = event == 1
        released = event == 3
        # q 退出：按下或重复时生效，抬起时不能再触发
        if keycode in QUIT_KEYS and not released:
            self.running = False
            return
        # Ctrl+C（或含 Ctrl/Super 的组合）退出。
        # kitty 的修饰位采用 1+bitmask：无修饰=1、Shift=2、Alt=4、Ctrl=5、Super=9，
        # 因此 modifiers >= 5 表示按下了 Ctrl 或 Super 类组合键。
        # 这里必须单独判断，否则被拆成普通 c 键后会被误当成“相机开关”。
        if keycode == 99 and modifiers >= 5 and not released:
            self.running = False
            return
        if not self.armed:
            return
        # 99/109 是 c/m 的 ASCII 码；开关类只在按下瞬间响应一次
        if keycode == 99 and pressed:
            self.toggle_camera()
        elif keycode == 109 and pressed:
            self.toggle_slam()
        # 运动键：按下与抬起都要处理（同一键码可能来自方向键/小键盘，故查表）
        elif keycode in LINEAR_KEYS:
            self.move_kitty('linear', keycode, LINEAR_KEYS[keycode], pressed)
        elif keycode in ANGULAR_KEYS:
            self.move_kitty('angular', keycode, ANGULAR_KEYS[keycode], pressed)
        # 32 是空格：急停，只在按下时触发
        elif keycode == 32 and pressed:
            self.stop_all()
        elif keycode in SPEED_UP_KEYS and pressed:
            self.scale_speed(1.25)
        elif keycode in SPEED_DOWN_KEYS and pressed:
            self.scale_speed(0.8)

    # ---------- 输入解析 ----------

    def handle_evdev(self, code, value):
        """处理全局键盘（evdev）的单条按键事件。

        与终端模式的根本差别：这里拿不到“终端自己的按键上下文”，
        Ctrl/Shift/Alt/Super 不会被终端合并进字符，而是作为独立按键上报，
        因此必须自己维护 self.modifiers 集合，并在触发单键功能时排除组合键。

        value：1=按下、2=自动连发、0=抬起。
        """
        # 系统的自动连发（value==2）对运动键没有额外信息（抬起事件已能判松手），
        # 对开关键则会造成连发误触发，因此统一忽略
        if value == 2:
            return
        pressed = value == 1
        # 先维护修饰键状态：按下加入集合、抬起移除，然后直接返回，
        # 避免修饰键本身被后续分支当作功能键处理
        if code in EVDEV_MODIFIERS:
            if pressed:
                self.modifiers.add(code)
            else:
                self.modifiers.discard(code)
            return
        # plain = 本键刚按下【且】当前没有任何修饰键按住。
        # 只有 plain 为真才响应 c/m/+/- 这类单键开关：
        # 否则用户在别处按 Ctrl+C（中断）或 Shift+C（大写 C）会被拆成普通 c 上报，
        # 从而被误判为“关闭相机”，导致画面莫名其妙消失。
        # 运动键与空格不受此限制——按住 Shift 与方向键同按也不该丢失移动能力。
        plain = pressed and not self.modifiers
        # F12 暂停/恢复优先于暂停判断：暂停状态下仍要能按 F12 恢复，
        # 且允许带修饰键（例如某些桌面把 F12 与功能键组合在一起）
        if code == self.arm_key and pressed:
            self.set_armed(not self.armed)
            return
        if not self.armed:
            return
        # 运动键复用 kitty 模式的 move_kitty：evdev 的按下/抬起语义与之完全一致
        if code in EVDEV_LINEAR:
            self.move_kitty('linear', code, EVDEV_LINEAR[code], pressed)
        elif code in EVDEV_ANGULAR:
            self.move_kitty('angular', code, EVDEV_ANGULAR[code], pressed)
        # 46/50 分别是按键码 KEY_C / KEY_M；只有“无修饰的按下”才切换开关
        elif code == 46 and plain:
            self.toggle_camera()
        elif code == 50 and plain:
            self.toggle_slam()
        # 空格（57）急停：即使带修饰键也允许，便于应急
        elif code == EVDEV_SPACE and pressed:
            self.stop_all()
        elif code in EVDEV_SPEED_UP and plain:
            self.scale_speed(1.25)
        elif code in EVDEV_SPEED_DOWN and plain:
            self.scale_speed(0.8)

    def feed(self, data):
        """把终端读到的原始字节喂进解析缓冲区，逐段识别为按键并派发。

        之所以要缓冲：一次 read 可能只拿到半个转义序列（CSI/SS3/多字节 UTF-8），
        必须要等剩余字节到达后再解析，否则会被拆成错误的按键。
        长度不足时直接 return 等待下次拼接，不丢数据。
        """
        if self.debug_keys and data:
            self.get_logger().info(f'DEBUG 原始输入: {data!r}')
        self.buf += data
        while self.buf:
            first = self.buf[0]
            # 非 ESC 开头：普通字符。按首字节推断 UTF-8 序列长度
            # （首字节高位 1 的个数即字节数），不足则等下一次输入
            if first != 0x1b:
                length = 1
                if first >= 0xf0:
                    length = 4
                elif first >= 0xe0:
                    length = 3
                elif first >= 0xc0:
                    length = 2
                if len(self.buf) < length:
                    return
                char = self.buf[:length].decode('utf-8', errors='ignore')
                self.buf = self.buf[length:]
                if self.debug_keys:
                    self.get_logger().info(f'DEBUG 普通按键: {char!r}')
                self.handle_legacy_char(char)
                continue
            # ESC 开头，优先尝试 CSI 序列：ESC [ 参数 结束符，
            # kitty 协议也复用 CSI（结束符 u），参数里含 keycode;modifiers:event。
            # 注意匹配允许参数为空（如 ESC [ A 这类方向键），故用 * 而非 +
            match = re.match(rb'\x1b\[([0-9;: ]*)([A-Za-z~])', self.buf)
            if match:
                params = match.group(1).decode().replace(' ', '')
                final = match.group(2).decode()
                self.buf = self.buf[match.end():]
                if self.debug_keys:
                    self.get_logger().info(f'DEBUG CSI 序列: 参数={params!r} 结束符={final!r}')
                self.on_sequence(params, final)
                continue
            # SS3 序列：ESC O X。部分终端的功能键/方向键走这条路径，
            # 结束符仍是 A/B/C/D，故复用 on_arrow
            if len(self.buf) >= 3 and self.buf[1] == 0x4f:
                final = self.buf[2:3].decode('utf-8', errors='ignore')
                self.buf = self.buf[3:]
                if self.debug_keys:
                    self.get_logger().info(f'DEBUG SS3 序列: {final!r}')
                if final in LETTER_KEYS:
                    self.on_arrow(final)
                continue
            # ESC + 单个可打印字符（如 Alt+某键）：不是已知的转义序列，
            # 丢弃 ESC 只按普通字符处理，避免整段输入卡死
            if (len(self.buf) >= 2 and self.buf[1] not in (0x1b, 0x5b, 0x4f)
                    and 0x20 <= self.buf[1] < 0x7f):
                char = self.buf[1:2].decode('utf-8', errors='ignore')
                self.buf = self.buf[2:]
                if self.debug_keys:
                    self.get_logger().info(f'DEBUG ESC+{char}（按普通按键处理）')
                self.handle_legacy_char(char)
                continue
            # 以上都不匹配且缓冲过长：扔掉一个字节防止无限累积（应对乱码/未知序列）
            if len(self.buf) > 64:
                self.buf = self.buf[1:]
                continue
            return

    def on_arrow(self, final):
        """把方向键的 A/B/C/D 结束符转成对应动作。

        kitty 模式下统一换算成 kitty 键码交给 handle_kitty_event（这里没有修饰键
        信息，按 kitty 约定用 1=无修饰、1=按下）；兼容模式则退回成 w/s/d/a 字符。
        """
        if self.kitty:
            self.handle_kitty_event(LETTER_KEYS[final], 1, 1)
        else:
            # C/D 是右/左方向键，因此映射为 d/a，不能按字母顺序误解
            self.handle_legacy_char({'A': 'w', 'B': 's', 'C': 'd', 'D': 'a'}[final])

    def on_sequence(self, params, final):
        """解析 CSI 序列的参数与结束符，分派到 kitty 事件或方向键处理。

        kitty 的参数形如 ``keycode;modifiers:event``（还可能带可选的备选键码/文本
        字段，用 : 或 ; 分隔）；这里只取前两个字段，缺省值按 kitty 规范为
        modifiers=1（无修饰）、event=1（按下），以便普通 CSI A~D 也能复用同一路径。
        """
        fields = params.split(';') if params else []
        keycode = 0
        modifiers = 1
        event = 1
        # 第一个字段是键码（可能形如 97:65，冒号后为备选键码，这里只用主键码）
        if fields and fields[0]:
            keycode = int(fields[0].split(':')[0] or 0)
        # 第二个字段是“修饰键:事件类型”，任一部分缺失都保持默认值
        if len(fields) > 1 and fields[1]:
            parts = fields[1].split(':')
            modifiers = int(parts[0] or 1)
            if len(parts) > 1 and parts[1]:
                event = int(parts[1])
        # 结束符 u = kitty 的增强按键上报，键码直接可用
        if final == 'u':
            if keycode:
                self.handle_kitty_event(keycode, modifiers, event)
        # 结束符 A~D = 方向键。kitty 模式下换算成 kitty 键码；
        # 兼容模式没有修饰/事件信息，只在“按下”时响应，避免重复触发
        elif final in LETTER_KEYS:
            if self.kitty:
                self.handle_kitty_event(LETTER_KEYS[final], modifiers, event)
            elif event == 1:
                self.on_arrow(final)


def enable_kitty_protocol(fd):
    """主动询问终端是否支持 kitty 键盘协议，支持则请求增强上报。

    流程：发 CSI ?u 查询 -> 在 0.3s 内等终端回复 CSI ?编号u -> 回复里带上能力位时
    再发  CSI > 能力位u 启用上报。返回 (是否支持, 查询期间读到的剩余原始字节)。
    这些剩余字节通常是用户在查询窗口期内敲下的按键（或终端回显），
    必须交回给调用方继续解析，否则会丢失开头几个按键。
    超时未回复就判定为不支持，退回兼容模式。
    """
    os.write(fd, b'\x1b[?u')
    deadline = time.monotonic() + 0.3
    buf = b''
    while time.monotonic() < deadline:
        ready, _, _ = select.select([fd], [], [], 0.05)
        if not ready:
            continue
        buf += os.read(fd, 256)
        match = re.search(rb'\x1b\[\?(\d+)u', buf)
        if match:
            leftover = buf[match.end():]
            os.write(fd, f'\x1b[>{KITTY_FLAGS}u'.encode())
            return True, leftover
    return False, buf


def open_terminal():
    """获取可读写的终端 fd 用于遥控；不是交互式终端时返回 None。

    需要可写的原因是：kitty 查询与退出时的协议还原、以及 termios 操作都要求
    同一 fd 可写。若 stdin 只是只读重定向（launch 里常见），就改用 /dev/tty，
    它始终指向当前控制终端，不受重定向影响。
    """
    fd = sys.stdin.fileno()
    if not sys.stdin.isatty():
        return None
    try:
        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        if (flags & os.O_ACCMODE) == os.O_RDONLY:
            return os.open('/dev/tty', os.O_RDWR)
    except OSError:
        return os.open('/dev/tty', os.O_RDWR)
    return fd


def tcgetattr_retry(fd):
    """读取终端属性；被信号打断（EINTR）时重试而非报错。

    退出信号会打断 termios 调用；若在此处抛出异常，后面的“恢复终端属性”就
    执行不到，终端会遗留在 raw 模式（看不到输入回显），所以必须重试。
    """
    while True:
        try:
            return termios.tcgetattr(fd)
        except termios.error as exc:
            if exc.args and exc.args[0] == errno.EINTR:
                continue
            raise


def tcsetattr_retry(fd, when, attrs):
    """写入终端属性，同样在 EINTR 时重试，保证设置一定生效。"""
    while True:
        try:
            termios.tcsetattr(fd, when, attrs)
            return
        except termios.error as exc:
            if exc.args and exc.args[0] == errno.EINTR:
                continue
            raise


def setcbreak_retry(fd):
    """把终端切成 cbreak/raw 模式：逐字符、无回显。

    关掉 ICANON 才能不用回车就立即拿到按键；关掉 ECHO 避免按键被终端回显
    （否则方向键的转义序列会被打印成一堆乱码）。
    VMIN=1/VTIME=0 表示 read 至少等到 1 个字节，配合外层的 select 即可在
    “有数据才读”与“不空转”之间取得平衡。
    """
    attrs = tcgetattr_retry(fd)
    attrs[3] &= ~(termios.ICANON | termios.ECHO)
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    tcsetattr_retry(fd, termios.TCSAFLUSH, attrs)


def finish(node):
    """统一收尾：先补发一帧零速度与前轮回正，再销毁节点、关闭 rclpy。

    额外补发的原因是退出瞬间定时器可能刚发出过运动指令，若不再发一帧零值，
    控制器会保持最后一次速度直到超时，表现为“松开后小车还往前冲一下”。
    """
    try:
        node.steering_publisher.publish(Float64())
        node.publisher.publish(Twist())
    except Exception:
        # 收尾阶段发布失败（如上下文已失效）不应影响后续销毁与退出
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print('已退出键盘遥控，速度指令已归零、前轮回正')


def run_terminal(node):
    """终端模式主循环：设置 raw，启用 kitty（可行时），非阻塞读取并驱动 ROS 回调。"""
    fd = open_terminal()
    if fd is None:
        # 非交互式终端拿不到键盘，直接报错退出，避免节点静默空转
        node.get_logger().error('无法读取键盘：请在交互式终端中运行该节点')
        finish(node)
        return

    # 先保存原始属性，退出时必须还原，否则用户的终端会一直处于无回显状态
    old_settings = tcgetattr_retry(fd)
    setcbreak_retry(fd)

    print(HELP)
    node.kitty, leftover = enable_kitty_protocol(fd)
    if node.kitty:
        node.print_status('完整按键事件（Kitty 协议），前进/转向通道完全独立，可同时按住')
    else:
        node.print_status('兼容模式（按键连发检测），松开后自动停止')
    # 查询 kitty 能力期间读到的按键在这里补喂，避免开头丢键
    if leftover:
        node.feed(leftover)

    try:
        while rclpy.ok() and node.running:
            # 用带超时的 spin_once 驱动 ROS 回调（订阅/定时器），不用后台线程，
            # 保证按键状态与发布都在同一线程内访问
            rclpy.spin_once(node, timeout_sec=0.05)
            # 超时为 0：只在确实有输入时读，避免阻塞主循环
            if select.select([fd], [], [], 0)[0]:
                node.feed(os.read(fd, 1024))
    except KeyboardInterrupt:
        # rclpy 已禁用自带信号处理，这里主要兜底 SIGINT；finally 仍会执行
        pass
    finally:
        # 关闭 kitty 增强上报，否则用户后续在终端里按方向键仍会看到增强序列
        if node.kitty:
            os.write(fd, b'\x1b[<u')
        # TCSADRAIN：等输出队列发完再改属性，避免退出提示与回显设置相互干扰
        tcsetattr_retry(fd, termios.TCSADRAIN, old_settings)
        # 只有自己打开的 /dev/tty 才需要关闭，stdin 不能代替别人关
        if fd != sys.stdin.fileno():
            os.close(fd)
        finish(node)


def run_evdev(node, keyboards):
    """全局键盘（evdev）模式主循环：直接读 /dev/input/event*，不依赖窗口焦点。

    与终端模式的差别：按键来自设备文件而非 stdin，所以切到 RViz/Gazebo 窗口后
    照样能遥控；同时也可以顺便监听 stdin，仅用于把终端输入读走、避免堆积。
    """
    node.evdev = True
    print(HELP)
    node.print_status(
        f'全局键盘（evdev，{len(keyboards)} 个设备，无需窗口焦点；'
        f'F12 暂停/恢复，Ctrl+C 退出）')

    # 若确实运行在交互式终端，也把它纳入监听：读到的内容直接丢弃，
    # 目的只是不让终端输入缓冲堆积。Ctrl+C 仍由 ISIG 产生 SIGINT 触发退出。
    tty_fd = None
    old_settings = None
    if sys.stdin.isatty():
        tty_fd = open_terminal()
        if tty_fd is not None:
            old_settings = tcgetattr_retry(tty_fd)
            setcbreak_retry(tty_fd)

    last_scan = time.monotonic()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.02)
            # 每次循环都重新构造监听列表，因为期间可能有设备被拔掉/新增
            watch = list(keyboards.values())
            if tty_fd is not None:
                watch.append(tty_fd)
            ready, _, _ = select.select(watch, [], [], 0.05)
            for fd in ready:
                if fd == tty_fd:
                    os.read(fd, 1024)
                    continue
                try:
                    # 一次最多读 64 个事件，减少系统调用次数
                    data = os.read(fd, EVENT_STRUCT.size * 64)
                except OSError:
                    # 设备被拔出/无权限等情况：当作空数据处理，走下面的下线逻辑
                    data = b''
                if not data:
                    # 读不到数据说明设备已不可用（多为拔出），关闭 fd 并移出列表
                    for path, dev_fd in list(keyboards.items()):
                        if dev_fd == fd:
                            os.close(fd)
                            del keyboards[path]
                    continue
                # 按事件结构体长度对齐遍历；末尾不足一个事件的字节不解析，
                # 避免把不完整数据解包成错误的按键
                for offset in range(0, len(data) - len(data) % EVENT_STRUCT.size,
                                    EVENT_STRUCT.size):
                    _, _, etype, code, value = EVENT_STRUCT.unpack_from(data, offset)
                    # 只关心按键事件，忽略同步/坐标等其他类型
                    if etype == EV_KEY:
                        node.handle_evdev(code, value)
            # 每 2 秒重新扫描一次设备列表以支持热插拔：
            # 已存在的设备要关掉这次多开的 fd（保持集合中每个设备只有一个 fd），
            # 新出现的设备才加入监听
            if time.monotonic() - last_scan > 2.0:
                last_scan = time.monotonic()
                for path, new_fd in find_keyboards().items():
                    if path in keyboards:
                        os.close(new_fd)
                    else:
                        keyboards[path] = new_fd
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭所有键盘 fd，并把终端属性还原（只还原自己动过的那一个）
        for fd in keyboards.values():
            os.close(fd)
        if tty_fd is not None and old_settings is not None:
            tcsetattr_retry(tty_fd, termios.TCSADRAIN, old_settings)
            if tty_fd != sys.stdin.fileno():
                os.close(tty_fd)
        finish(node)


def main(args=None):
    """入口：选定输入后端并进入对应主循环。

    选取顺序（input_mode=auto 时的默认行为）：
        能扫到全局键盘设备 -> evdev 全局键盘模式；
        否则退回终端模式（由 enable_kitty_protocol 再决定 kitty 还是兼容模式）。
    input_mode=evdev 时会强制要求全局键盘，找不到就报错退出；
    input_mode=terminal 则跳过扫描，直接走终端模式。
    """
    # 禁用 rclpy 自带的信号处理，改由下面自行接管：自带处理会直接抛异常打断
    # 主循环，导致终端属性来不及恢复
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MazeTeleop()

    def request_stop(signum, frame):
        # 只置标志位，让主循环自然退出并在 finally 里完成清理，不在信号处理函数里做重活
        node.running = False

    # SIGINT（Ctrl+C）与 SIGTERM（kill/launch 关闭）都走同一条优雅退出路径
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    mode = node.get_parameter('input_mode').value
    keyboards = {}
    # 只有 auto/evdev 才需要扫描设备；terminal 模式跳过可避免不必要的 ioctl
    if mode in ('auto', 'evdev'):
        keyboards = find_keyboards()

    # 有可用设备就直接进入 evdev 模式（auto 下优先它，因为它不依赖窗口焦点）
    if keyboards:
        run_evdev(node, keyboards)
        return

    # 强制 evdev 却找不到设备：明确报错退出，而不是静默降级，
    # 否则用户会以为遥控可用却怎么按都没反应
    if mode == 'evdev':
        node.get_logger().error(
            '未找到可读的键盘设备：请检查 /dev/input 权限（见 README 全局键盘模式）')
        finish(node)
        return

    run_terminal(node)


if __name__ == '__main__':
    main()
