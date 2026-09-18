"""maze_bot 控制台（Tkinter GUI）。

窗口分两部分：
    上半部分「操控方法」——按键速查表（纯文字说明，按键本身由 maze_teleop 处理）；
    下半部分「小车状态」——实时表格，显示里程计、轮速、转角以及相机/SLAM 开关状态，
    并提供「开启/暂停遥控」「开始/停止建图」两个按钮。

实现要点：这是一个把 ROS 2 节点与 Tkinter 主循环合在一起的进程。
    * ROS 2 回调不单独开线程，而是在 Tk 的 100 ms 定时器里用 spin_once() 驱动，
      避免多线程访问同一份状态；
    * 启动前调用 XInitThreads()，因为 Tk 与 ROS 2 的底层库都要用 X11；
    * 禁用 rclpy 自带的信号处理（SignalHandlerOptions.NO），改由自己处理
      SIGINT/SIGTERM，保证 Ctrl+C 时能退回 Tk 主循环正常退出。
"""
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

# 轮子半径，必须与 URDF 中的 wheel_radius 一致：
# 关节速度（rad/s）乘以它才能换算成轮面线速度（m/s）
WHEEL_RADIUS = 0.05
# 关节名 -> 表格里的显示缩写。键名必须与 URDF 中的关节名完全一致，
# 否则取不到数据（界面上会一直显示 0.00）
WHEEL_NAMES = {
    'front_left_wheel_joint': 'FL',
    'front_right_wheel_joint': 'FR',
    'rear_left_wheel_joint': 'RL',
    'rear_right_wheel_joint': 'RR',
}

# 操控方法速查表：(按键, 说明)。
# 这里只是说明文字，实际的按键监听在 maze_teleop（支持终端与全局键盘两种模式），
# 本窗口只额外响应 E 键用于快速开关遥控
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

# 状态表格的行定义：(内部 key, 显示标题)。
# key 与 update() 里 set_row('key', ...) 的调用一一对应，
# 表格行用 iid=key 插入，更新时按 key 定位
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

# 窗口底部的一行提示，告诉用户完整系统该怎么启动
USAGE_LINES = [
    '启动全部功能：./quickstart.sh',
    '单独遥控：ros2 launch maze_teleop teleop.launch.py',
]


class RobotState(Node):
    """纯数据节点：订阅各话题缓存最新状态，并提供两个开关的发布接口。

    本身不做任何界面绘制，界面只读取这里的字段，因此 UI 刷新与 ROS 回调解耦。
    """

    def __init__(self):
        super().__init__('maze_gui')
        # 里程计相关（来自 /odom）
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        # 关节相关（来自 /joint_states 与 /steering_position）
        self.steer = 0.0
        self.wheel_speeds = {}
        # 当前下发的速度指令（来自 /cmd_vel），用于与实测速度对比
        self.cmd_linear = 0.0
        self.cmd_omega = 0.0
        self.cmd_steer = 0.0
        # 以下 *_stamp 记录“最后一次收到消息的时刻”，配合 connected() 判断在线状态
        self.odom_stamp = 0.0
        self.teleop_armed = None
        self.teleop_stamp = 0.0
        self.camera_enabled = None
        self.camera_stamp = 0.0
        self.slam_enabled = None
        self.slam_stamp = 0.0
        # 由信号处理或关窗置为 False，主循环检测到后退出
        self.running = True

        # 状态类话题：订阅方只需“最近值”，用默认 QoS（Reliable）即可
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Float64, '/steering_position',
                                 self.on_steering_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        # 各节点的开关状态由对应节点周期上报（见 teleop/camera_relay/slam_manager）
        self.create_subscription(Bool, '/teleop_state', self.on_teleop_state, 10)
        self.create_subscription(Bool, '/camera_state', self.on_camera_state, 10)
        self.create_subscription(Bool, '/slam_state', self.on_slam_state, 10)
        # 下发给其他节点的开关指令
        self.enable_pub = self.create_publisher(Bool, '/teleop_enable', 10)
        self.slam_pub = self.create_publisher(Bool, '/slam_enable', 10)

    def on_odom(self, msg):
        """缓存里程计：位置、航向与实测速度。"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # 由四元数取绕 Z 轴的偏航角（车体在平面内运动，只关心 yaw）
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z
        # 用单调时钟而非系统时间：判断在线只看相对时间差，
        # 单调时钟不受系统时间调整影响
        self.odom_stamp = time.monotonic()

    def on_joints(self, msg):
        """从关节状态里取出前轮转角与四个轮子的转速。"""
        joint = dict(zip(msg.name, msg.position))
        velocity = dict(zip(msg.name, msg.velocity))
        # 左右前轮各有一个转向关节，取平均作为“前轮转角”显示
        front = [joint.get('front_left_steering_joint', 0.0),
                 joint.get('front_right_steering_joint', 0.0)]
        self.steer = 0.5 * (front[0] + front[1])
        # 关节速度单位是 rad/s，乘半径换算成轮面线速度（m/s）；
        # 取绝对值只关心快慢，不显示转向方向
        for name, label in WHEEL_NAMES.items():
            self.wheel_speeds[label] = abs(velocity.get(name, 0.0)) * WHEEL_RADIUS

    def on_steering_cmd(self, msg):
        """记录下发的目标转角，便于与实测转角对比（检查控制器是否跟上）。"""
        self.cmd_steer = msg.data

    def on_cmd_vel(self, msg):
        """记录下发的速度指令，便于与实测速度对比。"""
        self.cmd_linear = msg.linear.x
        self.cmd_omega = msg.angular.z

    def on_teleop_state(self, msg):
        """遥控节点上报的开关状态。"""
        self.teleop_armed = bool(msg.data)
        self.teleop_stamp = time.monotonic()

    def on_camera_state(self, msg):
        """相机中继上报的开关状态。"""
        self.camera_enabled = bool(msg.data)
        self.camera_stamp = time.monotonic()

    def camera_connected(self):
        """相机中继是否在线：2 秒内收到过状态上报。"""
        return (time.monotonic() - self.camera_stamp) < 2.0

    def on_slam_state(self, msg):
        """SLAM 管理器上报的开关状态。"""
        self.slam_enabled = bool(msg.data)
        self.slam_stamp = time.monotonic()

    def slam_connected(self):
        """SLAM 管理器是否在线：2 秒内收到过状态上报。"""
        return (time.monotonic() - self.slam_stamp) < 2.0

    def toggle_slam(self):
        """翻转建图开关；状态未知（未连上）时默认发“开启”。"""
        target = True
        if self.slam_connected() and self.slam_enabled is not None:
            target = not self.slam_enabled
        msg = Bool()
        msg.data = target
        self.slam_pub.publish(msg)

    def connected(self):
        """仿真是否在线：1 秒内收到过 /odom（比状态上报要求更严格）。"""
        return (time.monotonic() - self.odom_stamp) < 1.0

    def teleop_connected(self):
        """遥控节点是否在线：2 秒内收到过状态上报。"""
        return (time.monotonic() - self.teleop_stamp) < 2.0

    def toggle_teleop(self):
        """翻转遥控开关；状态未知（未连上）时默认发“开启”。"""
        target = True
        if self.teleop_connected() and self.teleop_armed is not None:
            target = not self.teleop_armed
        msg = Bool()
        msg.data = target
        self.enable_pub.publish(msg)


class DashboardApp:
    """Tkinter 界面：负责布局、绘制与定时刷新，数据全部取自 RobotState 节点。"""

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('maze_bot 控制台')
        # 点窗口关闭按钮时走统一退出流程（停主循环并释放节点）
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        # 中文显示依赖系统字体：按优先级挑一个已安装的 CJK 字体，
        # 都找不到就退回 Tk 默认字体（此时中文可能显示为方块）
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

        # 行 key -> 标题，供 set_row 回填数值时查标题
        self.rows = dict(STATUS_ROWS)
        self.build_controls()
        self.build_status()

        # 底部使用说明固定贴在最下方
        footer = ttk.Frame(self.root)
        footer.pack(side='bottom', fill='x', padx=12, pady=6)
        for line in USAGE_LINES:
            ttk.Label(footer, text=line, font=self.ui_font_small,
                      foreground='#555555').pack(anchor='w')

        # 窗口内也能用 E 键开关遥控（大小写都绑，避免 CapsLock/Shift 影响）
        self.root.bind('<KeyPress-e>', lambda event: self.node.toggle_teleop())
        self.root.bind('<KeyPress-E>', lambda event: self.node.toggle_teleop())

        # 先按内容算好尺寸再摆正窗口位置
        self.fit_window()
        # 启动刷新循环（100 ms 一次）
        self.root.after(100, self.update)

    def fit_window(self):
        """按控件实际需要的大小设置窗口尺寸，并居中偏上显示，避免超出屏幕。"""
        self.root.update_idletasks()
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        # 留出边距，小屏时也不会让窗口超出可视区域
        width = min(self.root.winfo_reqwidth(), screen_w - 60)
        height = min(self.root.winfo_reqheight(), screen_h - 100)
        x = max((screen_w - width) // 2, 0)
        # 垂直方向偏上（1/3 处），下方留出空间给其他窗口
        y = max((screen_h - height) // 3, 0)
        self.root.geometry(f'{width}x{height}+{x}+{y}')

    def build_controls(self):
        """构建「操控方法」区块：两列排布的按键速查表。"""
        frame = ttk.LabelFrame(self.root, text=' 操控方法 ', padding=8)
        frame.pack(fill='x', padx=12, pady=(12, 6))
        # 每行放两条，row//2 为行号，(row%2)*2 为起始列（按键列），+1 为说明列
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
        # 说明文字单独占一行（columnspan=4 跨两列键值对），放在表格下方
        note.grid(row=5, column=0, columnspan=4, sticky='w', padx=4, pady=(6, 0))

    def build_status(self):
        """构建「小车状态」区块：顶部连接/开关栏 + 状态表格。"""
        frame = ttk.LabelFrame(self.root, text=' 小车状态 ', padding=8)
        frame.pack(fill='both', expand=True, padx=12, pady=6)

        # 顶栏左侧是仿真连接与遥控状态指示，右侧是两个开关按钮
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
        # 按钮文案与可用性在 update() 中随状态变化
        self.toggle_button = ttk.Button(header, text='开启遥控',
                                        command=self.node.toggle_teleop)
        self.toggle_button.pack(side='right')
        self.slam_button = ttk.Button(header, text='开始建图',
                                      command=self.node.toggle_slam)
        self.slam_button.pack(side='right', padx=(0, 6))

        # 表格样式：行高按字体行距计算，避免中文被裁切
        style = ttk.Style(self.root)
        row_height = self.ui_font.metrics('linespace') + 8
        style.configure('Dashboard.Treeview', font=self.ui_font,
                        rowheight=row_height)
        style.configure('Dashboard.Treeview.Heading', font=self.ui_font_bold)

        # 按内容实测宽度定列宽（+32 留出内边距），保证数值不被截断
        item_width = max(self.ui_font.measure(title)
                         for _, title in STATUS_ROWS) + 32
        value_width = max(self.ui_font.measure(sample) for sample in (
            'x = +0.000, y = +0.000', '0.00 / 0.00 / 0.00 / 0.00')) + 32

        # show='headings' 表示不显示树状层级列，只显示自定义的两列
        self.table = ttk.Treeview(frame, columns=('item', 'value'),
                                  show='headings', height=len(STATUS_ROWS),
                                  style='Dashboard.Treeview')
        self.table.heading('item', text='项目')
        self.table.heading('value', text='数值')
        self.table.column('item', width=item_width, minwidth=item_width,
                          anchor='w', stretch=False)
        # 数值列允许拉伸，窗口变大时自动占满剩余宽度
        self.table.column('value', width=value_width, minwidth=value_width,
                          anchor='w', stretch=True)
        # 隔行浅灰底色，便于横向对齐阅读
        self.table.tag_configure('odd', background='#f3f3f3')
        # 预插入所有行（iid 用内部 key），初始值 '--'，之后仅更新数值
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
        """按行 key 更新表格数值（标题取自 STATUS_ROWS，保持不变）。"""
        self.table.item(key, values=(self.rows[key], value))

    def update(self):
        """定时刷新：先驱动 ROS 回调，再按最新状态重绘界面，最后排下一次刷新。"""
        # running 为 False（收到信号或用户关窗）时退出主循环，
        # 不能在这里直接 return，否则窗口会卡住不响应
        if not self.node.running:
            self.root.quit()
            return
        # timeout_sec=0.0 表示只处理已就绪的回调，立刻返回，不阻塞界面
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception:
            pass

        node = self.node
        # 连接指示：颜色与文字同步变化（绿=在线，红=未连接）
        if node.connected():
            self.link_var.set('● 仿真在线')
            self.link_label.configure(foreground='#2a8a2a')
        else:
            self.link_var.set('● 仿真未连接（等待 /odom）')
            self.link_label.configure(foreground='#c03030')

        # 遥控按钮有三种形态：未连接 / 已开启 / 已暂停
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

        # 带符号格式化便于观察方向；角度类统一换算成度
        self.set_row('linear', f'{node.linear_vel:+.3f}')
        self.set_row('angular', f'{node.angular_vel:+.3f}')
        self.set_row('pose', f'x = {node.x:+.3f}, y = {node.y:+.3f}')
        self.set_row('yaw', f'{math.degrees(node.yaw):+.1f}')
        self.set_row('steer', f'{math.degrees(node.steer):+.1f}')
        self.set_row('steer_cmd', f'{math.degrees(node.cmd_steer):+.1f}')
        wheels = node.wheel_speeds
        # 固定按 FL/FR/RL/RR 顺序展示，缺数据时补 0
        self.set_row('wheels', ' / '.join(
            f'{wheels.get(name, 0.0):.2f}' for name in ('FL', 'FR', 'RL', 'RR')))
        self.set_row('cmd', f'{node.cmd_linear:+.3f}')
        self.set_row('cmd_omega', f'{node.cmd_omega:+.3f}')
        if not node.camera_connected():
            self.set_row('camera', '未连接')
        else:
            self.set_row('camera', '开启' if node.camera_enabled else '关闭')
        # SLAM 按钮在管理器未上线时置灰，避免点了没有响应；
        # 文案随建图状态在「开始建图 / 停止建图」之间切换
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
        """点关闭按钮：通知主循环退出，并让 Tk 收尾。"""
        self.node.running = False
        self.root.quit()

    def run(self):
        """进入 Tk 主事件循环（阻塞，直到 quit() 被调用）。"""
        self.root.mainloop()


def main(args=None):
    # Tk 与 ROS 2 底层都会使用 X11，先初始化 X 的多线程支持，
    # 否则在部分环境下会出现随机崩溃或卡死
    ctypes.CDLL('libX11.so.6').XInitThreads()
    # 自己接管信号：默认的 rclpy 信号处理会直接结束进程，
    # 那样 Tk 窗口来不及正常关闭
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RobotState()

    # signum/frame 是信号处理函数的固定形参，必须保留（即使这里用不到）
    def request_stop(signum, frame):
        # 只置标志位，真正的退出在 update() 里完成（保证在 Tk 线程中收尾）
        node.running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        app = DashboardApp(node)
        app.run()
    except tk.TclError as exc:
        # 典型原因：无显示环境（未设置 DISPLAY、通过 SSH 运行等）
        node.get_logger().error(f'无法创建 GUI 窗口（请检查 DISPLAY）: {exc}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()