import ctypes
import math
import signal
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

WHEEL_RADIUS = 0.05
WHEEL_NAMES = {
    'front_left_wheel_joint': 'FL',
    'front_right_wheel_joint': 'FR',
    'rear_left_wheel_joint': 'RL',
    'rear_right_wheel_joint': 'RR',
}

CONTROLS = [
    ('w / ↑', '前进'),
    ('s / ↓', '后退'),
    ('a / ←', '前轮左转'),
    ('d / →', '前轮右转'),
    ('空格', '立即停止'),
    ('F12', '暂停 / 恢复遥控'),
    ('+ / -', '加快 / 减慢速度'),
    ('c', '相机开关'),
    ('m', 'SLAM 建图开关'),
    ('q', '退出遥控（终端）'),
    ('按钮 / E', '窗口内开启 / 暂停遥控'),
]

STATUS_ROWS = [
    ('linear', '线速度 (m/s)'),
    ('angular', '角速度 (rad/s)'),
    ('pose', '位置 (m)'),
    ('yaw', '航向 (°)'),
    ('steer', '前轮转角 (°)'),
    ('steer_cmd', '转角指令 (°)'),
    ('wheels', '轮速 FL/FR/RL/RR (m/s)'),
    ('cmd', '指令线速度 (m/s)'),
    ('cmd_omega', '指令角速度 (rad/s)'),
    ('camera', '相机'),
    ('slam', 'SLAM 建图'),
]

USAGE_LINES = [
    '启动全部功能：./quickstart.sh',
    '单独遥控：ros2 launch maze_teleop teleop.launch.py',
]


class RobotState(Node):
    def __init__(self):
        super().__init__('maze_gui')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.steer = 0.0
        self.wheel_speeds = {}
        self.cmd_linear = 0.0
        self.cmd_omega = 0.0
        self.cmd_steer = 0.0
        self.odom_stamp = 0.0
        self.teleop_armed = None
        self.teleop_stamp = 0.0
        self.camera_enabled = None
        self.camera_stamp = 0.0
        self.slam_enabled = None
        self.slam_stamp = 0.0
        self.running = True

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Float64, '/steering_position',
                                 self.on_steering_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Bool, '/teleop_state', self.on_teleop_state, 10)
        self.create_subscription(Bool, '/camera_state', self.on_camera_state, 10)
        self.create_subscription(Bool, '/slam_state', self.on_slam_state, 10)
        self.enable_pub = self.create_publisher(Bool, '/teleop_enable', 10)
        self.slam_pub = self.create_publisher(Bool, '/slam_enable', 10)

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z
        self.odom_stamp = time.monotonic()

    def on_joints(self, msg):
        joint = dict(zip(msg.name, msg.position))
        velocity = dict(zip(msg.name, msg.velocity))
        front = [joint.get('front_left_steering_joint', 0.0),
                 joint.get('front_right_steering_joint', 0.0)]
        self.steer = 0.5 * (front[0] + front[1])
        for name, label in WHEEL_NAMES.items():
            self.wheel_speeds[label] = abs(velocity.get(name, 0.0)) * WHEEL_RADIUS

    def on_steering_cmd(self, msg):
        self.cmd_steer = msg.data

    def on_cmd_vel(self, msg):
        self.cmd_linear = msg.linear.x
        self.cmd_omega = msg.angular.z

    def on_teleop_state(self, msg):
        self.teleop_armed = bool(msg.data)
        self.teleop_stamp = time.monotonic()

    def on_camera_state(self, msg):
        self.camera_enabled = bool(msg.data)
        self.camera_stamp = time.monotonic()

    def camera_connected(self):
        return (time.monotonic() - self.camera_stamp) < 2.0

    def on_slam_state(self, msg):
        self.slam_enabled = bool(msg.data)
        self.slam_stamp = time.monotonic()

    def slam_connected(self):
        return (time.monotonic() - self.slam_stamp) < 2.0

    def toggle_slam(self):
        target = True
        if self.slam_connected() and self.slam_enabled is not None:
            target = not self.slam_enabled
        msg = Bool()
        msg.data = target
        self.slam_pub.publish(msg)

    def connected(self):
        return (time.monotonic() - self.odom_stamp) < 1.0

    def teleop_connected(self):
        return (time.monotonic() - self.teleop_stamp) < 2.0

    def toggle_teleop(self):
        target = True
        if self.teleop_connected() and self.teleop_armed is not None:
            target = not self.teleop_armed
        msg = Bool()
        msg.data = target
        self.enable_pub.publish(msg)


class DashboardApp:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('maze_bot 控制台')
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        family = 'TkDefaultFont'
        families = set(tkfont.families(self.root))
        for candidate in ('Noto Sans CJK SC', 'WenQuanYi Micro Hei',
                          'AR PL UMing CN', 'Source Han Sans SC'):
            if candidate in families:
                family = candidate
                break
        self.ui_font = tkfont.Font(root=self.root, family=family, size=11)
        self.ui_font_bold = tkfont.Font(root=self.root, family=family, size=11,
                                        weight='bold')
        self.ui_font_small = tkfont.Font(root=self.root, family=family, size=9)

        self.rows = dict(STATUS_ROWS)
        self.build_controls()
        self.build_status()

        footer = ttk.Frame(self.root)
        footer.pack(side='bottom', fill='x', padx=12, pady=6)
        for line in USAGE_LINES:
            ttk.Label(footer, text=line, font=self.ui_font_small,
                      foreground='#555555').pack(anchor='w')

        self.root.bind('<KeyPress-e>', lambda event: self.node.toggle_teleop())
        self.root.bind('<KeyPress-E>', lambda event: self.node.toggle_teleop())

        self.fit_window()
        self.root.after(100, self.update)

    def fit_window(self):
        self.root.update_idletasks()
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        width = min(self.root.winfo_reqwidth(), screen_w - 60)
        height = min(self.root.winfo_reqheight(), screen_h - 100)
        x = max((screen_w - width) // 2, 0)
        y = max((screen_h - height) // 3, 0)
        self.root.geometry(f'{width}x{height}+{x}+{y}')

    def build_controls(self):
        frame = ttk.LabelFrame(self.root, text=' 操控方法 ', padding=8)
        frame.pack(fill='x', padx=12, pady=(12, 6))
        for row, (keys, action) in enumerate(CONTROLS):
            ttk.Label(frame, text=keys, font=self.ui_font_bold, width=7,
                      anchor='w').grid(row=row // 2, column=(row % 2) * 2,
                                       sticky='w', padx=(4, 2), pady=2)
            ttk.Label(frame, text=action, font=self.ui_font,
                      anchor='w').grid(row=row // 2, column=(row % 2) * 2 + 1,
                                       sticky='w', padx=(0, 16), pady=2)
        note = ttk.Label(frame, text='前进/后退由后轮驱动；转向键松开后前轮自动回正；'
                                     '遥控暂停时全部驾驶按键无效',
                         font=self.ui_font_small, foreground='#555555')
        note.grid(row=5, column=0, columnspan=4, sticky='w', padx=4, pady=(6, 0))

    def build_status(self):
        frame = ttk.LabelFrame(self.root, text=' 小车状态 ', padding=8)
        frame.pack(fill='both', expand=True, padx=12, pady=6)

        header = ttk.Frame(frame)
        header.pack(fill='x', pady=(0, 6))
        self.link_var = tk.StringVar(value='● 仿真未连接')
        self.link_label = ttk.Label(header, textvariable=self.link_var,
                                    font=self.ui_font_bold, foreground='#c03030')
        self.link_label.pack(side='left')
        self.teleop_var = tk.StringVar(value='遥控：未连接')
        self.teleop_label = ttk.Label(header, textvariable=self.teleop_var,
                                      font=self.ui_font_bold, foreground='#888888')
        self.teleop_label.pack(side='left', padx=(18, 8))
        self.toggle_button = ttk.Button(header, text='开启遥控',
                                        command=self.node.toggle_teleop)
        self.toggle_button.pack(side='right')
        self.slam_button = ttk.Button(header, text='开始建图',
                                      command=self.node.toggle_slam)
        self.slam_button.pack(side='right', padx=(0, 6))

        style = ttk.Style(self.root)
        row_height = self.ui_font.metrics('linespace') + 8
        style.configure('Dashboard.Treeview', font=self.ui_font,
                        rowheight=row_height)
        style.configure('Dashboard.Treeview.Heading', font=self.ui_font_bold)

        item_width = max(self.ui_font.measure(title)
                         for _, title in STATUS_ROWS) + 32
        value_width = max(self.ui_font.measure(sample) for sample in (
            'x = +0.000, y = +0.000', '0.00 / 0.00 / 0.00 / 0.00')) + 32

        self.table = ttk.Treeview(frame, columns=('item', 'value'),
                                  show='headings', height=len(STATUS_ROWS),
                                  style='Dashboard.Treeview')
        self.table.heading('item', text='项目')
        self.table.heading('value', text='数值')
        self.table.column('item', width=item_width, minwidth=item_width,
                          anchor='w', stretch=False)
        self.table.column('value', width=value_width, minwidth=value_width,
                          anchor='w', stretch=True)
        self.table.tag_configure('odd', background='#f3f3f3')
        for index, (key, title) in enumerate(STATUS_ROWS):
            tags = ('odd',) if index % 2 else ()
            self.table.insert('', 'end', iid=key, values=(title, '--'),
                              tags=tags)
        self.table.pack(fill='both', expand=True)

        note = ttk.Label(frame, text='按钮或 E 键可开启/暂停遥控；'
                                     'F12 全局暂停；退出遥控后小车自动停止并回正前轮',
                         font=self.ui_font_small, foreground='#555555')
        note.pack(anchor='w', pady=(8, 0))

    def set_row(self, key, value):
        self.table.item(key, values=(self.rows[key], value))

    def update(self):
        if not self.node.running:
            self.root.quit()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception:
            pass

        node = self.node
        if node.connected():
            self.link_var.set('● 仿真在线')
            self.link_label.configure(foreground='#2a8a2a')
        else:
            self.link_var.set('● 仿真未连接（等待 /odom）')
            self.link_label.configure(foreground='#c03030')

        if not node.teleop_connected():
            self.teleop_var.set('遥控：未连接')
            self.teleop_label.configure(foreground='#888888')
            self.toggle_button.configure(text='开启遥控')
        elif node.teleop_armed:
            self.teleop_var.set('遥控：已开启')
            self.teleop_label.configure(foreground='#2a8a2a')
            self.toggle_button.configure(text='暂停遥控')
        else:
            self.teleop_var.set('遥控：已暂停')
            self.teleop_label.configure(foreground='#c07000')
            self.toggle_button.configure(text='开启遥控')

        self.set_row('linear', f'{node.linear_vel:+.3f}')
        self.set_row('angular', f'{node.angular_vel:+.3f}')
        self.set_row('pose', f'x = {node.x:+.3f}, y = {node.y:+.3f}')
        self.set_row('yaw', f'{math.degrees(node.yaw):+.1f}')
        self.set_row('steer', f'{math.degrees(node.steer):+.1f}')
        self.set_row('steer_cmd', f'{math.degrees(node.cmd_steer):+.1f}')
        wheels = node.wheel_speeds
        self.set_row('wheels', ' / '.join(
            f'{wheels.get(name, 0.0):.2f}' for name in ('FL', 'FR', 'RL', 'RR')))
        self.set_row('cmd', f'{node.cmd_linear:+.3f}')
        self.set_row('cmd_omega', f'{node.cmd_omega:+.3f}')
        if not node.camera_connected():
            self.set_row('camera', '未连接')
        else:
            self.set_row('camera', '开启' if node.camera_enabled else '关闭')
        if not node.slam_connected():
            self.set_row('slam', '未连接')
            self.slam_button.configure(text='开始建图', state='disabled')
        else:
            self.set_row('slam', '建图中' if node.slam_enabled else '已停止')
            self.slam_button.configure(
                text='停止建图' if node.slam_enabled else '开始建图',
                state='normal')

        self.root.after(100, self.update)

    def on_close(self):
        self.node.running = False
        self.root.quit()

    def run(self):
        self.root.mainloop()


def main(args=None):
    ctypes.CDLL('libX11.so.6').XInitThreads()
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RobotState()

    def request_stop(signum, frame):
        node.running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        app = DashboardApp(node)
        app.run()
    except tk.TclError as exc:
        node.get_logger().error(f'无法创建 GUI 窗口（请检查 DISPLAY）: {exc}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()