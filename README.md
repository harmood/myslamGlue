# myslam_ws

基于 ROS 2 Jazzy + Gazebo（gz-sim 8 / Harmonic）的迷宫仿真项目，用于机器人导航 / SLAM 等仿真实验，包含两个功能包：

- `maze_world`：迷宫世界（正方形地图、1.2 m 墙高、1.0 m 通道）
- `maze_bot`：四轮小车描述与仿真（前轮转向 + 后轮驱动、里程计、TF、ROS-Gazebo 话题桥接）
- `maze_teleop`：键盘遥控节点（全局键盘模式 / 终端模式）
- `maze_gui`：GUI 控制台（操控说明 + 小车实时状态）

## 地图规格

| 项目 | 数值 |
| --- | --- |
| 地图形状 | 正方形 11.0 m × 11.0 m |
| 迷宫规模 | 9 × 9 单元格 |
| 通道净宽 | 1.0 m |
| 墙壁高度 | 1.2 m |
| 墙壁厚度 | 0.2 m |
| 起点 | (-4.80, -4.80)，绿色圆标 |
| 终点 | (4.80, 4.80)，红色圆标 |
| 坐标原点 | 地图中心 |

迷宫由递归回溯算法生成，再随机打通 8% 的墙体形成环路；生成脚本内置 BFS 校验，保证起点到终点至少存在一条通路。

## 目录结构

```
myslam_ws/
├── quickstart.sh                    # 一键构建并启动（迷宫 + 小车）
├── teleop_kitty.sh                  # kitty 遥控窗口（完整按键事件模式）
└── src/
    ├── maze_world/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   └── maze.launch.py       # 启动 gz sim + RViz
    │   ├── rviz/
    │   │   └── maze.rviz            # 预留的 SLAM 可视化配置
    │   ├── scripts/
    │   │   └── generate_maze.py     # 迷宫 / 世界生成脚本
    │   └── worlds/
    │       └── maze.world           # 生成的 SDF 世界文件
    ├── maze_bot/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   └── ros_gz_bridge.yaml   # ROS <-> Gazebo 话题桥接配置
        ├── launch/
        │   ├── display.launch.py    # 仅 RViz 显示模型
        │   └── spawn_maze.launch.py # 迷宫世界 + 生成小车（默认）
        ├── rviz/
        │   └── maze_bot.rviz
        ├── scripts/
        │   └── camera_relay.py      # 相机中继（开关控制）
        └── urdf/
            └── maze_bot.urdf.xacro
    └── maze_teleop/
        ├── package.xml
        ├── setup.py
        ├── launch/
        │   └── teleop.launch.py     # 键盘遥控启动文件
        └── maze_teleop/
            └── teleop_keyboard.py   # 键盘遥控节点
    └── maze_gui/
        ├── package.xml
        ├── setup.py
        ├── launch/
        │   └── dashboard.launch.py  # GUI 控制台启动文件
        └── maze_gui/
            └── dashboard.py         # 操控说明 + 小车状态窗口
```

## 环境要求

- ROS 2 Jazzy
- Gazebo Harmonic（gz-sim 8）
- `ros_gz_sim`、`ros_gz_bridge`、`ros-jazzy-gz-sim-vendor`

## 快速开始

```bash
./quickstart.sh
```

一条命令启动全部功能：自动 source ROS 环境、编译四个功能包，然后启动迷宫世界、生成小车、打开 RViz 和话题桥接，并启动键盘遥控节点（全局键盘模式，无需聚焦终端）与 GUI 状态窗口。操控方式：`w/s` 前后、`a/d` 前轮转向、空格急停、`F12` 暂停/恢复，`Ctrl+C` 一键优雅关闭全部（遥控会先归零并回正）。

可选参数：

```bash
./quickstart.sh --no-build       # 跳过编译，直接启动
./quickstart.sh --no-teleop      # 不启动键盘遥控
./quickstart.sh --no-gui         # 不启动 GUI 状态窗口
./quickstart.sh --world-only     # 只启动迷宫世界（不生成小车、不启动遥控/GUI）
./quickstart.sh --help           # 查看帮助
```

手动分步执行等价命令：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select maze_world maze_bot maze_teleop maze_gui --symlink-install
source install/setup.bash
ros2 launch maze_bot spawn_maze.launch.py        # 终端 1：仿真
ros2 launch maze_teleop teleop.launch.py         # 终端 2：遥控
ros2 launch maze_gui dashboard.launch.py         # 终端 3：GUI 控制台（可选）
```

## 小车（maze_bot）

四轮小车，前轮带转向关节（转向柱 + 转向关节 + 轮子自转关节），后轮差速驱动，模型规格：

| 项目 | 数值 |
| --- | --- |
| 车身尺寸 | 0.40 m × 0.25 m × 0.10 m（1.0 kg） |
| 轮子 | 半径 0.05 m、宽 0.04 m，共 4 个（0.05 kg/个） |
| 轮距 / 轴距 | 0.28 m / 0.30 m |
| 前轮最大转角 | ±0.6 rad（遥控默认限幅 0.5 rad） |
| 车头相机 | 640×480 @ 15 Hz，水平 FOV 60°，安装于车头 (0.19, 0, 0.02) |
| 生成位置 | 起点 (-4.80, -4.80)，朝向 +x |
| 后轮驱动插件 | `gz-sim-diff-drive-system`（仅后轮） |
| 前轮转向插件 | `gz-sim-joint-position-controller-system` |

ROS 话题（经 `config/ros_gz_bridge.yaml` 桥接）：

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo | 后轮速度指令（angular.z 为后轮差速转向） |
| `/steering_position` | `std_msgs/msg/Float64` | ROS → Gazebo | 前轮转角指令 [rad]，同时作用于左右转向关节 |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS | 轮式里程计（50 Hz） |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo → ROS | odom → base_footprint |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS | 关节状态（含转向关节，RViz 模型显示用） |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Gazebo → ROS | 车头相机原始图像（640×480 rgb8，15 Hz） |
| `/camera/image` | `sensor_msgs/msg/Image` | 中继转发 | 相机中继输出（受相机开关控制），RViz 显示用 |
| `/camera_enable` | `std_msgs/msg/Bool` | ROS 内部 | 相机开关指令（true 开 / false 关） |
| `/camera_state` | `std_msgs/msg/Bool` | ROS 内部 | 相机开关状态（每秒上报） |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS | 仿真时钟 |

驱动示例：

```bash
# 前进 0.3 m/s
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
# 前轮左转 0.4 rad（松开回正为 0）
ros2 topic pub --once /steering_position std_msgs/msg/Float64 "{data: 0.4}"
```

说明：机器人转弯由"前轮转向角 + 后轮差速"共同实现，遥控节点会根据转向几何自动换算
`angular.z = v · tan(δ) / L`，因此直接使用 `/cmd_vel` 时只控制后轮，前轮转角需另行给出
（或使用键盘遥控的 a/d）。

单独显示模型（仅 RViz，不启动 Gazebo）：

```bash
ros2 launch maze_bot display.launch.py
```

## 键盘遥控（maze_teleop）

先在一个终端启动迷宫与小车：

```bash
./quickstart.sh
```

`quickstart.sh` 已自动启动遥控；如需单独运行遥控节点（仿真已在运行时）：

```bash
source install/setup.bash
ros2 launch maze_teleop teleop.launch.py
```

### 全局键盘模式（推荐，无需聚焦终端）

遥控节点默认自动使用全局键盘模式：直接读取 Linux 输入设备（evdev），**焦点即使停留在 Gazebo/RViz 窗口也照样操控**，并且使用真实的按下/松开事件，无兼容模式的粘滞问题。启动日志会显示：

```
键盘模式：全局键盘（evdev，N 个设备，无需窗口焦点；F12 暂停/恢复，Ctrl+C 退出）
```

- `F12`：暂停/恢复全局控制（暂停时所有按键不生效）
- 遥控开关话题：`/teleop_enable`（`std_msgs/msg/Bool`，`true` 开启、`false` 暂停），遥控节点通过 `/teleop_state` 每秒上报状态；GUI 控制台的按钮/E 键同样使用这两个话题
- 退出：在运行遥控的终端按 `Ctrl+C`（全局模式下 `q` 不再退出，避免误触）
- 注意：启用时按键是全局生效的，在其他窗口输入 `w/a/s/d` 也会驱动小车，不用时可先按 F12 暂停

其他机器启用该模式（一次性配置，无需重新登录）：

```bash
sudo sh -c 'echo "KERNEL==\"event*\", SUBSYSTEM==\"input\", TAG+=\"uaccess\"" > /etc/udev/rules.d/60-maze-teleop-input.rules'
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=input
```

如只想在终端窗口聚焦时控制，可强制终端模式（配合 kitty 终端获得完整按键事件）：

```bash
ros2 launch maze_teleop teleop.launch.py input_mode:=terminal
./teleop_kitty.sh        # 自动打开 kitty 窗口并启动遥控
```

其他支持 Kitty 协议的终端：foot、wezterm、新版 VS Code 集成终端。GNOME Terminal（VTE 0.76）不支持，会退化为兼容模式。

按键说明（两键前轮转向、两键前后，两条通道相互独立）：

| 按键 | 功能 | 按键 | 功能 |
| --- | --- | --- | --- |
| `w` / ↑ | 前进（后轮驱动） | `s` / ↓ | 后退（后轮驱动） |
| `a` / ← | 前轮左转向 | `d` / → | 前轮右转向 |
| 空格 | 立即停止 | `q` | 退出 |
| `+` / `-` | 加快 / 减慢速度 | `c` | 相机开关 |

`a`/`d` 直接控制前轮转向关节：按住时转角平滑转到最大角（默认 0.5 rad，速率 2.0 rad/s），松开自动回正；与 `w`/`s` 组合即为弧线行驶。转向角与后轮差速的换算由遥控节点按轴距自动完成。

前进/后退与转向是**独立双通道**：可以同时按住（例如 `w` + `a`）实现边走边转，松开哪个键哪个通道立即处理（前进停止、前轮回正）。节点启动时会自动检测终端能力并打印当前模式：

- **完整按键事件模式（Kitty 键盘协议）**：终端直报按键的按下/松开事件，两通道完全独立、松开瞬间停止（支持该协议的终端如 kitty、foot、新版 VS Code 终端等）
- **兼容模式（按键连发检测）**：终端只上报字符流时，按住期间靠自动连发维持；节点会自动读取系统键盘连发参数（GNOME `gsettings` 或 X11 `xset`）计算判停时间，松开判定通常约 0.1 s、按住宽限约为连发延迟 + 0.12 s，启动日志会打印检测结果

两个时间参数默认 `0`（自动检测），也可手动覆盖：

```bash
ros2 launch maze_teleop teleop.launch.py release_timeout:=0.15 hold_delay:=0.5
```

兼容模式下终端无法上报"松开"事件，点按会持续到 `hold_delay` 结束，快速点按会显得粘滞。可用 `tap_pulse` 改善（点按后只脉冲运动该时长，代价是按住时在首次连发到来前会短暂停顿）：

```bash
ros2 launch maze_teleop teleop.launch.py tap_pulse:=0.15
```

如果按键行为异常，可打开按键调试日志查看终端实际发送的序列：

```bash
ros2 launch maze_teleop teleop.launch.py debug_keys:=true
```

初始线速度 0.2 m/s、最大前轮转角 0.5 rad，可通过参数调整：

```bash
ros2 launch maze_teleop teleop.launch.py linear_speed:=0.3 max_steering:=0.45 steering_rate:=3.0
```

说明：`ros2 launch` 默认把子进程 stdin 设为管道，launch 文件内部通过 `/dev/tty` 读取键盘，因此必须在真实终端中运行，不要用管道重定向。退出（`q` 或 Ctrl+C）时节点会自动发布零速度、前轮回正、恢复终端设置并还原键盘协议状态。

## GUI 控制台（maze_gui）

`quickstart.sh` 会自动打开 GUI 控制台窗口（用 `--no-gui` 关闭），也可以单独启动：

```bash
ros2 launch maze_gui dashboard.launch.py
```

窗口包含三部分：

- **操控方法**：完整按键说明（前后、转向、急停、F12 暂停、调速、退出、GUI 遥控开关）与常用启动命令
- **遥控开关**：右上角按钮（或窗口内按 `E` 键）可开启 / 暂停遥控，状态显示"已开启 / 已暂停 / 未连接"，也可用 `F12` 全局切换。开关通过 `/teleop_enable`（`std_msgs/msg/Bool`）下发，遥控节点以 `/teleop_state` 每秒上报当前状态
- **小车状态**（约 10 Hz 刷新，以表格形式展示；窗口启动时按内容自动定尺并居中，无需手动调整大小）：
  - 表格行：线速度、角速度、位置 (x, y)、航向角、前轮转角、转角指令、四轮轮速、指令线速度、指令角速度、相机开关状态
  - 表格上方状态：仿真连接状态（依据 `/odom` 是否在 1 秒内更新）、遥控状态与开关按钮

订阅话题：`/odom`、`/joint_states`、`/steering_position`、`/cmd_vel`、`/teleop_state`；发布 `/teleop_enable`。

## RViz 可视化

- 主场景（`spawn_maze.launch.py`）会同时启动 RViz，使用 `maze_world/rviz/maze.rviz`；单独查看小车模型用 `maze_bot display.launch.py`（`maze_bot.rviz`，固定坐标系 `base_footprint`）
- 主场景 RViz 的固定坐标系为 `maze_bot/odom`（当前 TF 树中实际存在的坐标系）
- Grid / TF / RobotModel / Camera 默认启用（Camera 为车头相机实时画面，话题 `/camera/image`）；LaserScan、PointCloud2、Map、Path 已预配置好话题
- 相机渲染在启动后需要数十秒初始化（本机为软件渲染），期间 Camera 显示暂无画面，稍候即可（`/scan`、`/cloud_registered`、`/map`、`/path`）但**默认禁用**，避免数据源未就绪时报错；接入传感器或 SLAM 后在 Displays 面板勾选启用即可
- 接入 SLAM 后可将 Fixed Frame 切换为 `map`

## 关闭方式

脚本对启动的 ros2 launch 与 Gazebo 进程做了完整的生命周期管理：

- **Ctrl+C**：先发送 SIGINT 优雅关闭 Gazebo（server/gui），等待 10 秒；未退出则依次升级为 SIGTERM、SIGKILL
- **连续按两次 Ctrl+C**：立即强制结束所有相关进程
- **关闭 Gazebo 窗口**：`on_exit_shutdown` 会联动退出 ros2 launch 与脚本
- **终端窗口关闭（SIGHUP）/ 进程异常退出**：脚本会按会话清理残留的 gz sim server/gui 进程

无论哪种方式退出，脚本结束前都会回收本项目的全部残留进程，Ctrl+C 属于正常退出。

## 重新生成迷宫

```bash
python3 src/maze_world/scripts/generate_maze.py \
  --size 9 --seed 20260915 --loops 0.08 \
  --output src/maze_world/worlds/maze.world
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--size` | 9 | 每边单元格数，地图边长 = (size-1)×1.2 + 1.4 m |
| `--seed` | 20260915 | 随机种子，固定种子可复现同一迷宫 |
| `--loops` | 0.08 | 额外打通墙体的比例（0 为完美迷宫，无环路） |
| `--output` | maze.world | 输出文件路径 |

包以 `--symlink-install` 方式编译，重新生成世界后无需再次编译，直接重新启动即可生效。
