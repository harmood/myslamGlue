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

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64

HELP = """
==================================================
  maze_bot 键盘遥控
--------------------------------------------------
  前后移动： w / ↑ 前进      s / ↓ 后退
  前轮转向： a / ← 左转      d / → 右转
  空格 立即停止          q 退出
  + / - 加快 / 减慢速度   c 相机开关
--------------------------------------------------"""

KITTY_FLAGS = 11  # disambiguate(1) | report event types(2) | report all keys(8)

LINEAR_KEYS = {119: 1.0, 57352: 1.0, 57419: 1.0,    # w, up, keypad up
               115: -1.0, 57353: -1.0, 57420: -1.0,  # s, down, keypad down
               87: 1.0, 83: -1.0}                    # W, S（兼容实现不完整的终端）
ANGULAR_KEYS = {97: 1.0, 57350: 1.0, 57417: 1.0,    # a, left, keypad left
                100: -1.0, 57351: -1.0, 57418: -1.0,  # d, right, keypad right
                65: 1.0, 68: -1.0}                    # A, D（兼容实现不完整的终端）
LETTER_KEYS = {'A': 57352, 'B': 57353, 'C': 57351, 'D': 57350}
SPEED_UP_KEYS = {61, 43, 107}     # = + keypad-plus
SPEED_DOWN_KEYS = {45, 95, 109}   # - _ keypad-minus
QUIT_KEYS = {113}                 # q

EV_KEY = 0x01
EVENT_STRUCT = struct.Struct('llHHi')
EVDEV_LINEAR = {17: 1.0, 103: 1.0, 31: -1.0, 108: -1.0}    # w/up, s/down
EVDEV_ANGULAR = {30: 1.0, 105: 1.0, 32: -1.0, 106: -1.0}   # a/left, d/right
EVDEV_SPEED_UP = {13, 78}         # = keypad-plus
EVDEV_SPEED_DOWN = {12, 74}       # - keypad-minus
EVDEV_SPACE = 57


def _has_bit(bitmask, code):
    return bitmask[code // 8] & (1 << (code % 8))


def _eviocgbit(ev, length):
    return (2 << 30) | (length << 16) | (ord('E') << 8) | (0x20 + ev)


LIBC = ctypes.CDLL('libc.so.6', use_errno=True)


def _ioctl_buffer(fd, request, size):
    buf = ctypes.create_string_buffer(size)
    if LIBC.ioctl(fd, ctypes.c_ulong(request), buf) < 0:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err))
    return buf.raw


def find_keyboards():
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
        pass
    try:
        out = subprocess.run(['xset', 'q'], capture_output=True, text=True, timeout=2).stdout
        match = re.search(r'auto repeat delay:\s+(\d+)\s+repeat rate:\s+(\d+)', out)
        if match:
            return int(match.group(1)) / 1000.0, 1.0 / int(match.group(2))
    except Exception:
        pass
    return None, None


class MazeTeleop(Node):
    def __init__(self):
        super().__init__('maze_teleop')
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

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.steering_rate = float(self.get_parameter('steering_rate').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        steering_topic = self.get_parameter('steering_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        topic = self.get_parameter('cmd_vel_topic').value
        self.debug_keys = bool(self.get_parameter('debug_keys').value)
        self.arm_key = int(self.get_parameter('arm_key').value)
        self.armed = True
        self.evdev = False

        self.delay, self.interval = detect_repeat_settings()
        auto_hold = self.delay + 0.12 if self.delay else 0.7
        auto_release = max(3.0 * self.interval, 0.08) if self.interval else 0.25
        auto_release = min(auto_release, 0.25)
        release_timeout = float(self.get_parameter('release_timeout').value)
        hold_delay = float(self.get_parameter('hold_delay').value)
        tap_pulse = float(self.get_parameter('tap_pulse').value)
        self.release_timeout = release_timeout if release_timeout > 0 else auto_release
        self.hold_delay = hold_delay if hold_delay > 0 else auto_hold
        self.tap_pulse = tap_pulse if tap_pulse > 0 else 0.0

        self.publisher = self.create_publisher(Twist, topic, 10)
        self.steering_publisher = self.create_publisher(Float64, steering_topic, 10)
        self.state_publisher = self.create_publisher(
            Bool, self.get_parameter('state_topic').value, 10)
        self.create_subscription(Bool, self.get_parameter('enable_topic').value,
                                 self.on_enable, 10)
        self.camera_enable_publisher = self.create_publisher(
            Bool, self.get_parameter('camera_enable_topic').value, 10)
        self.create_subscription(
            Bool, self.get_parameter('camera_state_topic').value,
            self.on_camera_state, 10)
        self.camera_enabled = True
        self.create_timer(1.0, self.publish_state)
        self.twist = Twist()
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self.steer = 0.0
        self.last_publish = time.monotonic()
        self.kitty = False
        self.running = True
        self.buf = b''

        # 兼容模式：按通道独立记录按键方向与时间戳
        self.legacy = {
            'linear': {'sign': 0.0, 'last': 0.0, 'repeat': False},
            'angular': {'sign': 0.0, 'last': 0.0, 'repeat': False},
        }
        # Kitty 模式：记录每个通道当前按住的键（有序，后按的生效）
        self.held = {'linear': {}, 'angular': {}}

        self.timer = self.create_timer(1.0 / publish_rate, self.publish)

    def publish(self):
        now = time.monotonic()
        dt = max(now - self.last_publish, 0.0)
        self.last_publish = now
        for channel in ('linear', 'angular'):
            state = self.legacy[channel]
            if state['sign'] == 0.0:
                continue
            timeout = self.release_timeout if state['repeat'] else (
                self.tap_pulse if self.tap_pulse > 0 else self.hold_delay)
            if now - state['last'] > timeout:
                state['sign'] = 0.0
                state['repeat'] = False

        linear = self.legacy['linear']['sign']
        angular = self.legacy['angular']['sign']
        if self.held['linear']:
            linear = list(self.held['linear'].values())[-1]
        if self.held['angular']:
            angular = list(self.held['angular'].values())[-1]
        self.linear_cmd = linear
        self.angular_cmd = angular

        target = angular * self.max_steering
        step = self.steering_rate * dt
        if self.steer < target:
            self.steer = min(self.steer + step, target)
        elif self.steer > target:
            self.steer = max(self.steer - step, target)

        steering_msg = Float64()
        steering_msg.data = self.steer
        self.steering_publisher.publish(steering_msg)

        velocity = linear * self.linear_speed
        self.twist = Twist()
        self.twist.linear.x = velocity
        self.twist.angular.z = velocity * math.tan(self.steer) / self.wheelbase
        self.publisher.publish(self.twist)

    def print_status(self, mode):
        self.get_logger().info(f'键盘模式：{mode}')
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
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        for state in self.legacy.values():
            state['sign'] = 0.0
            state['repeat'] = False
        for channel in self.held:
            self.held[channel].clear()

    def scale_speed(self, factor):
        self.linear_speed = min(max(self.linear_speed * factor, 0.05), 0.5)
        self.get_logger().info(
            f'线速度 {self.linear_speed:.2f} m/s，最大前轮转角 {self.max_steering:.2f} rad')

    def publish_state(self):
        msg = Bool()
        msg.data = self.armed
        self.state_publisher.publish(msg)

    def set_armed(self, armed):
        self.armed = armed
        if not armed:
            self.stop_all()
        self.publish_state()
        if armed:
            self.get_logger().info('遥控已开启（F12 或 GUI 可再次暂停）')
        else:
            self.get_logger().info('遥控已暂停（F12 或 GUI 恢复）')

    def on_enable(self, msg):
        if bool(msg.data) != self.armed:
            self.set_armed(bool(msg.data))

    def on_camera_state(self, msg):
        self.camera_enabled = bool(msg.data)

    def toggle_camera(self):
        self.camera_enabled = not self.camera_enabled
        msg = Bool()
        msg.data = self.camera_enabled
        self.camera_enable_publisher.publish(msg)
        self.get_logger().info(
            '相机已开启' if self.camera_enabled else '相机已关闭')

    # ---------- 事件处理 ----------

    def move_legacy(self, channel, sign):
        state = self.legacy[channel]
        if state['sign'] == sign:
            state['repeat'] = True
        else:
            state['sign'] = sign
            state['repeat'] = False
        state['last'] = time.monotonic()

    def move_kitty(self, channel, keycode, sign, pressed):
        keys = self.held[channel]
        if pressed:
            keys.pop(keycode, None)
            keys[keycode] = sign
        else:
            keys.pop(keycode, None)
        if keys:
            sign = list(keys.values())[-1]
        else:
            sign = 0.0
        if channel == 'linear':
            self.linear_cmd = sign
        else:
            self.angular_cmd = sign

    def handle_legacy_char(self, char):
        if char in ('q', 'Q', '\uff51', '\uff31', '\x03'):
            self.running = False
            return
        if not self.armed:
            return
        if char in ('w', 'W', '\uff57', '\uff37'):
            self.move_legacy('linear', 1.0)
        elif char in ('s', 'S', '\uff53', '\uff33'):
            self.move_legacy('linear', -1.0)
        elif char in ('a', 'A', '\uff41', '\uff21'):
            self.move_legacy('angular', 1.0)
        elif char in ('d', 'D', '\uff44', '\uff24'):
            self.move_legacy('angular', -1.0)
        elif char in ('c', 'C', '\uff43', '\uff23'):
            self.toggle_camera()
        elif char in (' ', '\u3000'):
            self.stop_all()
        elif char in ('+', '=', '\uff0b', '\uff1d'):
            self.scale_speed(1.25)
        elif char in ('-', '_', '\uff0d', '\uff3f'):
            self.scale_speed(0.8)

    def handle_kitty_event(self, keycode, modifiers, event):
        if event == 2:
            return
        pressed = event == 1
        released = event == 3
        if keycode in QUIT_KEYS and not released:
            self.running = False
            return
        if keycode == 99 and modifiers >= 5 and not released:
            self.running = False
            return
        if not self.armed:
            return
        if keycode == 99 and pressed:
            self.toggle_camera()
        elif keycode in LINEAR_KEYS:
            self.move_kitty('linear', keycode, LINEAR_KEYS[keycode], pressed)
        elif keycode in ANGULAR_KEYS:
            self.move_kitty('angular', keycode, ANGULAR_KEYS[keycode], pressed)
        elif keycode == 32 and pressed:
            self.stop_all()
        elif keycode in SPEED_UP_KEYS and pressed:
            self.scale_speed(1.25)
        elif keycode in SPEED_DOWN_KEYS and pressed:
            self.scale_speed(0.8)

    # ---------- 输入解析 ----------

    def handle_evdev(self, code, value):
        if value == 2:
            return
        pressed = value == 1
        if code == self.arm_key and pressed:
            self.set_armed(not self.armed)
            return
        if not self.armed:
            return
        if code in EVDEV_LINEAR:
            self.move_kitty('linear', code, EVDEV_LINEAR[code], pressed)
        elif code in EVDEV_ANGULAR:
            self.move_kitty('angular', code, EVDEV_ANGULAR[code], pressed)
        elif code == 46 and pressed:
            self.toggle_camera()
        elif code == EVDEV_SPACE and pressed:
            self.stop_all()
        elif code in EVDEV_SPEED_UP and pressed:
            self.scale_speed(1.25)
        elif code in EVDEV_SPEED_DOWN and pressed:
            self.scale_speed(0.8)

    def feed(self, data):
        if self.debug_keys and data:
            self.get_logger().info(f'DEBUG 原始输入: {data!r}')
        self.buf += data
        while self.buf:
            first = self.buf[0]
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
            match = re.match(rb'\x1b\[([0-9;: ]*)([A-Za-z~])', self.buf)
            if match:
                params = match.group(1).decode().replace(' ', '')
                final = match.group(2).decode()
                self.buf = self.buf[match.end():]
                if self.debug_keys:
                    self.get_logger().info(f'DEBUG CSI 序列: 参数={params!r} 结束符={final!r}')
                self.on_sequence(params, final)
                continue
            if len(self.buf) >= 3 and self.buf[1] == 0x4f:
                final = self.buf[2:3].decode('utf-8', errors='ignore')
                self.buf = self.buf[3:]
                if self.debug_keys:
                    self.get_logger().info(f'DEBUG SS3 序列: {final!r}')
                if final in LETTER_KEYS:
                    self.on_arrow(final)
                continue
            if (len(self.buf) >= 2 and self.buf[1] not in (0x1b, 0x5b, 0x4f)
                    and 0x20 <= self.buf[1] < 0x7f):
                char = self.buf[1:2].decode('utf-8', errors='ignore')
                self.buf = self.buf[2:]
                if self.debug_keys:
                    self.get_logger().info(f'DEBUG ESC+{char}（按普通按键处理）')
                self.handle_legacy_char(char)
                continue
            if len(self.buf) > 64:
                self.buf = self.buf[1:]
                continue
            return

    def on_arrow(self, final):
        if self.kitty:
            self.handle_kitty_event(LETTER_KEYS[final], 1, 1)
        else:
            self.handle_legacy_char({'A': 'w', 'B': 's', 'C': 'd', 'D': 'a'}[final])

    def on_sequence(self, params, final):
        fields = params.split(';') if params else []
        keycode = 0
        modifiers = 1
        event = 1
        if fields and fields[0]:
            keycode = int(fields[0].split(':')[0] or 0)
        if len(fields) > 1 and fields[1]:
            parts = fields[1].split(':')
            modifiers = int(parts[0] or 1)
            if len(parts) > 1 and parts[1]:
                event = int(parts[1])
        if final == 'u':
            if keycode:
                self.handle_kitty_event(keycode, modifiers, event)
        elif final in LETTER_KEYS:
            if self.kitty:
                self.handle_kitty_event(LETTER_KEYS[final], modifiers, event)
            elif event == 1:
                self.on_arrow(final)


def enable_kitty_protocol(fd):
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
    while True:
        try:
            return termios.tcgetattr(fd)
        except termios.error as exc:
            if exc.args and exc.args[0] == errno.EINTR:
                continue
            raise


def tcsetattr_retry(fd, when, attrs):
    while True:
        try:
            termios.tcsetattr(fd, when, attrs)
            return
        except termios.error as exc:
            if exc.args and exc.args[0] == errno.EINTR:
                continue
            raise


def setcbreak_retry(fd):
    attrs = tcgetattr_retry(fd)
    attrs[3] &= ~(termios.ICANON | termios.ECHO)
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    tcsetattr_retry(fd, termios.TCSAFLUSH, attrs)


def finish(node):
    try:
        node.steering_publisher.publish(Float64())
        node.publisher.publish(Twist())
    except Exception:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print('已退出键盘遥控，速度指令已归零、前轮回正')


def run_terminal(node):
    fd = open_terminal()
    if fd is None:
        node.get_logger().error('无法读取键盘：请在交互式终端中运行该节点')
        finish(node)
        return

    old_settings = tcgetattr_retry(fd)
    setcbreak_retry(fd)

    print(HELP)
    node.kitty, leftover = enable_kitty_protocol(fd)
    if node.kitty:
        node.print_status('完整按键事件（Kitty 协议），前进/转向通道完全独立，可同时按住')
    else:
        node.print_status('兼容模式（按键连发检测），松开后自动停止')
    if leftover:
        node.feed(leftover)

    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([fd], [], [], 0)[0]:
                node.feed(os.read(fd, 1024))
    except KeyboardInterrupt:
        pass
    finally:
        if node.kitty:
            os.write(fd, b'\x1b[<u')
        tcsetattr_retry(fd, termios.TCSADRAIN, old_settings)
        if fd != sys.stdin.fileno():
            os.close(fd)
        finish(node)


def run_evdev(node, keyboards):
    node.evdev = True
    print(HELP)
    node.print_status(
        f'全局键盘（evdev，{len(keyboards)} 个设备，无需窗口焦点；'
        f'F12 暂停/恢复，Ctrl+C 退出）')

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
            watch = list(keyboards.values())
            if tty_fd is not None:
                watch.append(tty_fd)
            ready, _, _ = select.select(watch, [], [], 0.05)
            for fd in ready:
                if fd == tty_fd:
                    os.read(fd, 1024)
                    continue
                try:
                    data = os.read(fd, EVENT_STRUCT.size * 64)
                except OSError:
                    data = b''
                if not data:
                    for path, dev_fd in list(keyboards.items()):
                        if dev_fd == fd:
                            os.close(fd)
                            del keyboards[path]
                    continue
                for offset in range(0, len(data) - len(data) % EVENT_STRUCT.size,
                                    EVENT_STRUCT.size):
                    _, _, etype, code, value = EVENT_STRUCT.unpack_from(data, offset)
                    if etype == EV_KEY:
                        node.handle_evdev(code, value)
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
        for fd in keyboards.values():
            os.close(fd)
        if tty_fd is not None and old_settings is not None:
            tcsetattr_retry(tty_fd, termios.TCSADRAIN, old_settings)
            if tty_fd != sys.stdin.fileno():
                os.close(tty_fd)
        finish(node)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MazeTeleop()

    def request_stop(signum, frame):
        node.running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    mode = node.get_parameter('input_mode').value
    keyboards = {}
    if mode in ('auto', 'evdev'):
        keyboards = find_keyboards()

    if keyboards:
        run_evdev(node, keyboards)
        return

    if mode == 'evdev':
        node.get_logger().error(
            '未找到可读的键盘设备：请检查 /dev/input 权限（见 README 全局键盘模式）')
        finish(node)
        return

    run_terminal(node)


if __name__ == '__main__':
    main()
