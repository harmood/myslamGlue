#!/usr/bin/env python3
"""由固定布局生成迷宫世界文件（worlds/maze.world 的生成脚本）。

设计取舍：迷宫不采用随机/递归回溯等生成算法，而是把布局显式写成源码里的
FIXED_LAYOUT 字符串——这样每次生成的结果完全一致，便于复现实验与调试；
需要改迷宫结构时只改这个字符串并重新生成，而不是去改生成物。

FIXED_LAYOUT 图例（19 行 = 2n+1，即 9 个格子组成的 9x9 迷宫）：
    '+' : 四面墙的交汇点，生成为方形柱子
    '-' : 东西向的墙（同一列相邻格子的上下之间）
    '|' : 南北向的墙（同一行相邻格子的左右之间）
    ' ' : 通道（无墙）
    'S' : 起点，位于布局左下角，对应格子 (0, 0)
    'G' : 终点，位于布局右上角，对应格子 (n-1, n-1)

生成结果由 worlds/maze.world 直接使用（脚本默认输出名就是 maze.world）。
注意：重新生成会整体覆盖该文件，因此不要在 worlds/maze.world 里手工改布局。

用法：
    python3 generate_maze.py --output src/maze_world/worlds/maze.world
"""
import argparse

# 固定迷宫布局（9x9，显式定义，不使用任何生成算法）
# 字符含义：+ - | 为墙壁，空格为通道，S 起点，G 终点
FIXED_LAYOUT = """
+-+-+-+-+-+-+-+-+-+
|   |     |     |G|
+ + +-+ + + +-+ + +
| |     |   | |   |
+ +-+-+-+-+-+ +-+ +
|   |           | |
+ + + +-+-+-+ +-+ +
| | | |     |     |
+-+ + + + + +-+-+-+
|   |   |   |     |
+ +-+-+-+ + + + +-+
|   |     | | |   |
+-+ + + + + +-+-+ +
|     |   |       |
+ +-+ + +-+ + +-+-+
|     | | | |     |
+-+-+-+ + + +-+-+ +
|S      |         |
+-+-+-+-+-+-+-+-+-+
"""


def parse_layout(layout):
    """把 ASCII 布局解析成两张布尔墙表，供后续几何生成使用。

    返回值 (n, right, up)：
        n         : 每边的格子数（布局为 2n+1 行、2n+1 列，n=9）
        right[i][j]: 格子 (i, j) 的“东侧”是否有墙（即该列右边的 '|'）
        up[i][j]   : 格子 (i, j) 的“北侧”是否有墙（即该行上方的 '-'）

    坐标约定：ASCII 图从上到下画的是“从北到南”，而仿真坐标 Y 向北为正，
    所以读取时要把行号翻转（row = 2*(n-1-j)+...）。这里显式定义布局，
    不依赖任何随机或生成算法，因此解析结果每次完全一致。
    """
    # 去掉首尾换行后按行拆开，忽略空行；要求布局为正方形且边长为奇数
    art = [line for line in layout.strip('\n').splitlines() if line]
    size = len(art)
    if size % 2 == 0:
        raise ValueError('固定布局必须是 (2n+1) x (2n+1) 的奇数尺寸')
    if any(len(line) != size for line in art):
        raise ValueError('固定布局必须是正方形')
    n = (size - 1) // 2
    # 两张表先默认全为 True（有墙），再按布局中实际出现的墙覆盖为 False/True
    right = [[True] * n for _ in range(n)]
    up = [[True] * n for _ in range(n)]
    # 竖直墙（'|'）位于 art 的奇数列 2*i+2；行号需按 Y 轴翻转
    for i in range(n - 1):
        for j in range(n):
            row = 2 * (n - 1 - j) + 1
            right[i][j] = art[row][2 * i + 2] == '|'
    # 水平墙（'-'）位于 art 的奇数行 2*(n-1-j)
    for i in range(n):
        for j in range(n - 1):
            row = 2 * (n - 1 - j)
            up[i][j] = art[row][2 * i + 1] == '-'
    return n, right, up


def path_exists(right, up, n, start, goal):
    """用深度优先搜索（栈）判断起点到终点是否连通。

    显式布局也可能笔误画出“死区”，这里做一次连通性自检：
    不连通就直接报错，避免生成一个机器人根本走不到终点的迷宫。
    """
    seen = {start}
    frontier = [start]
    while frontier:
        i, j = frontier.pop()
        if (i, j) == goal:
            return True
        # 依次尝试四个方向的邻居；只有当对应方向没有墙时才能通过
        if i < n - 1 and not right[i][j] and (i + 1, j) not in seen:
            seen.add((i + 1, j))
            frontier.append((i + 1, j))
        if i > 0 and not right[i - 1][j] and (i - 1, j) not in seen:
            seen.add((i - 1, j))
            frontier.append((i - 1, j))
        if j < n - 1 and not up[i][j] and (i, j + 1) not in seen:
            seen.add((i, j + 1))
            frontier.append((i, j + 1))
        if j > 0 and not up[i][j - 1] and (i, j - 1) not in seen:
            seen.add((i, j - 1))
            frontier.append((i, j - 1))
    return False


# 几何常量（单位：米）
#   P：相邻格子中心的间距（cell pitch），也等于通道宽度 + 墙厚
#   T：墙体厚度
#   H：墙体高度
# 通道净宽 = P - T = 1.0 m，与 package.xml 里“1m corridors”的描述一致
P = 1.2
T = 0.2
H = 1.2


def box_xml(name, x, y, sx, sy, diffuse):
    """生成一段墙体的 collision + visual 条目（长方体）。

    两部分几何完全相同，只是 collision 参与物理、visual 负责显示；
    pose 的 z 取 H/2，让高为 H 的盒子底面正好落在地面 z=0 上。
    """
    return f"""
        <collision name='{name}_collision'>
          <pose>{x:.3f} {y:.3f} {H / 2:.3f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{sx:.3f} {sy:.3f} {H:.3f}</size>
            </box>
          </geometry>
        </collision>
        <visual name='{name}_visual'>
          <pose>{x:.3f} {y:.3f} {H / 2:.3f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{sx:.3f} {sy:.3f} {H:.3f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.55 0.55 0.55 1</ambient>
            <diffuse>{diffuse}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>"""


def build_walls(n, right, up):
    """由墙表生成全部墙体的几何列表 (name, x, y, sx, sy)。

    坐标以“格子中心 = i*P, j*P”为基准（(0,0) 格子中心即局部原点）：
        * outer_*      四面外墙，尺寸含外扩量，把迷宫四周包起来
        * pillar_{i}_{j} 每个墙交汇点放一个 T x T 的方柱（共 (n-1)^2 个）
        * east_{i}_{j} 格子 (i,j) 东侧的墙段，沿 Y 方向长 P-T
        * north_{i}_{j} 格子 (i,j) 北侧的墙段，沿 X 方向长 P-T
    本函数属于机器生成逻辑，改迷宫布局应改 FIXED_LAYOUT，而不是这里。
    """
    walls = []
    span = (n - 1) * P
    # 外墙：向外偏移半个墙厚再加大 0.5 m，确保把边界格子的房间完整围住
    walls.append(("outer_south", span / 2, -0.5 - T / 2, span + 1.0 + 2 * T, T))
    walls.append(("outer_north", span / 2, span + 0.5 + T / 2, span + 1.0 + 2 * T, T))
    walls.append(("outer_west", -0.5 - T / 2, span / 2, T, span + 1.0))
    walls.append(("outer_east", span + 0.5 + T / 2, span / 2, T, span + 1.0))
    # 柱子：放在相邻四个格子的交汇处，即 (i+0.5)*P
    for i in range(n - 1):
        for j in range(n - 1):
            walls.append((f"pillar_{i}_{j}", (i + 0.5) * P, (j + 0.5) * P, T, T))
    # 竖直墙段：位于格子右侧的网格线上，长度扣掉两端柱子占用的厚度
    for i in range(n - 1):
        for j in range(n):
            if right[i][j]:
                walls.append((f"east_{i}_{j}", (i + 0.5) * P, j * P, T, P - T))
    # 水平墙段：位于格子上方的网格线上，同样扣掉柱子厚度
    for i in range(n):
        for j in range(n - 1):
            if up[i][j]:
                walls.append((f"north_{i}_{j}", i * P, (j + 0.5) * P, P - T, T))
    return walls


def marker_xml(name, x, y, r, g, b):
    """生成起点/终点标记模型：贴地的一个小圆盘（薄圆柱）。

    pose 抬到 z=0.021，避免圆盘与地面共面产生 Z-fighting（闪烁）；
    颜色由调用方传入（起点绿、终点红）。
    """
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <pose>0 0 0.021 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>0.02</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def generate(n, right, up, output):
    """拼装完整 SDF 世界并写入 output 文件。

    整体由四部分组成：物理/系统插件与 GUI（可视化配置）、地面、
    迷宫墙体模型 maze_walls、以及起点/终点标记模型。
    坐标系：格子 (0,0) 中心被平移到世界 (-offset, -offset)，使迷宫中心
    位于世界原点；起点固定左下角 (0,0)、终点右上角 (n-1,n-1)。
    """
    start = (0, 0)
    goal = (n - 1, n - 1)
    if not path_exists(right, up, n, start, goal):
        raise RuntimeError("no path from start to goal")

    walls = build_walls(n, right, up)
    # offset 为迷宫中心到 (0,0) 格中心的距离，同时用于 maze_walls 的 pose 平移
    offset = (n - 1) * P / 2

    # 所有墙体条目拼成一个大字符串，嵌入 maze_walls 的 <link name="walls">
    wall_xml = "".join(
        box_xml(name, x, y, sx, sy, "0.80 0.80 0.80 1") for name, x, y, sx, sy in walls
    )

    # 起点/终点标记的世界坐标 = 格子坐标 * 间距 - offset
    start_x = start[0] * P - offset
    start_y = start[1] * P - offset
    goal_x = goal[0] * P - offset
    goal_y = goal[1] * P - offset
    # 地面可视方块边长：迷宫跨度 + 两侧各半个格子 + 两侧外墙厚度
    side = (n - 1) * P + 1.0 + 2 * T

    # SDF 模板拼装顺序：物理/系统插件 -> GUI 可视化插件 -> 地面 ->
    # 迷宫墙体模型 maze_walls -> 起点/终点标记模型。
    # 模板中写出的都是世界坐标；maze_walls 通过 pose 的 -offset 把
    # “格子局部坐标”整体平移到以世界原点为中心的区域内。
    world = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="maze">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gui fullscreen="0">

      <plugin filename="MinimalScene" name="3D View">
          <gz-gui>
              <title>3D View</title>
              <property type="bool" key="showTitleBar">false</property>
              <property type="string" key="state">docked</property>
          </gz-gui>
          <engine>ogre2</engine>
          <scene>scene</scene>
          <ambient_light>0.4 0.4 0.4</ambient_light>
          <background_color>0.8 0.8 0.8</background_color>
          <camera_pose>-6 0 6 0 0.5 0</camera_pose>
      </plugin>

      <plugin filename="EntityContextMenuPlugin" name="Entity context menu">
          <gz-gui>
              <property key="state" type="string">floating</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="showTitleBar" type="bool">false</property>
          </gz-gui>
      </plugin>

      <plugin filename="GzSceneManager" name="Scene Manager">
          <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
          </gz-gui>
      </plugin>

      <plugin filename="InteractiveViewControl" name="Interactive view control">
          <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
          </gz-gui>
      </plugin>

      <plugin filename="CameraTracking" name="Camera Tracking">
          <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
          </gz-gui>
      </plugin>

      <plugin filename="MarkerManager" name="Marker manager">
          <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
          </gz-gui>
      </plugin>

      <plugin filename="SelectEntities" name="Select Entities">
          <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
          </gz-gui>
      </plugin>

      <plugin filename="Spawn" name="Spawn Entities">
          <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
          </gz-gui>
      </plugin>

      <plugin filename="VisualizationCapabilities" name="Visualization Capabilities">
          <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="width" type="double">5</property>
              <property key="height" type="double">5</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
          </gz-gui>
      </plugin>

      <plugin filename="WorldControl" name="World control">
          <gz-gui>
              <title>World control</title>
              <property type="bool" key="showTitleBar">false</property>
              <property type="bool" key="resizable">false</property>
              <property type="double" key="height">72</property>
              <property type="double" key="z">1</property>
              <property type="string" key="state">floating</property>
              <anchors target="3D View">
                  <line own="left" target="left"/>
                  <line own="bottom" target="bottom"/>
              </anchors>
          </gz-gui>
          <play_pause>true</play_pause>
          <step>true</step>
          <start_paused>true</start_paused>
          <use_event>true</use_event>
      </plugin>

      <plugin filename="WorldStats" name="World stats">
          <gz-gui>
              <title>World stats</title>
              <property type="bool" key="showTitleBar">false</property>
              <property type="bool" key="resizable">false</property>
              <property type="double" key="height">110</property>
              <property type="double" key="width">290</property>
              <property type="double" key="z">1</property>
              <property type="string" key="state">floating</property>
              <anchors target="3D View">
                  <line own="right" target="right"/>
                  <line own="bottom" target="bottom"/>
              </anchors>
          </gz-gui>
          <sim_time>true</sim_time>
          <real_time>true</real_time>
          <real_time_factor>true</real_time_factor>
          <iterations>true</iterations>
      </plugin>

      <plugin filename="Shapes" name="Shapes">
          <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="x" type="double">0</property>
              <property key="y" type="double">0</property>
              <property key="width" type="double">300</property>
              <property key="height" type="double">50</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
              <property key="cardBackground" type="string">#666666</property>
          </gz-gui>
      </plugin>

      <plugin filename="Lights" name="Lights">
          <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="x" type="double">300</property>
              <property key="y" type="double">0</property>
              <property key="width" type="double">150</property>
              <property key="height" type="double">50</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
              <property key="cardBackground" type="string">#666666</property>
          </gz-gui>
      </plugin>

      <plugin filename="TransformControl" name="Transform control">
          <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="x" type="double">0</property>
              <property key="y" type="double">50</property>
              <property key="width" type="double">250</property>
              <property key="height" type="double">50</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
              <property key="cardBackground" type="string">#777777</property>
          </gz-gui>
      </plugin>

      <plugin filename="Screenshot" name="Screenshot">
          <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="x" type="double">250</property>
              <property key="y" type="double">50</property>
              <property key="width" type="double">50</property>
              <property key="height" type="double">50</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
              <property key="cardBackground" type="string">#777777</property>
          </gz-gui>
      </plugin>

      <plugin filename="CopyPaste" name="CopyPaste">
          <gz-gui>
              <property key="resizable" type="bool">false</property>
              <property key="x" type="double">300</property>
              <property key="y" type="double">50</property>
              <property key="width" type="double">100</property>
              <property key="height" type="double">50</property>
              <property key="state" type="string">floating</property>
              <property key="showTitleBar" type="bool">false</property>
              <property key="cardBackground" type="string">#777777</property>
          </gz-gui>
      </plugin>

      <plugin filename="ComponentInspector" name="Component inspector">
          <gz-gui>
              <property type="bool" key="showTitleBar">false</property>
              <property type="string" key="state">docked</property>
          </gz-gui>
      </plugin>

      <plugin filename="EntityTree" name="Entity tree">
          <gz-gui>
              <property type="bool" key="showTitleBar">false</property>
              <property type="string" key="state">docked</property>
          </gz-gui>
      </plugin>

      <plugin filename="VisualizeLidar" name="Visualize Lidar">
          <gz-gui>
              <title>Visualize Lidar</title>
              <property type="bool" key="showTitleBar">true</property>
              <property type="string" key="state">docked</property>
          </gz-gui>
      </plugin>
    </gui>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <pose>0 0 -0.05 0 0 0</pose>
          <geometry>
            <box>
              <size>{side:.3f} {side:.3f} 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.45 0.47 0.45 1</ambient>
            <diffuse>0.45 0.47 0.45 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="maze_walls">
      <static>true</static>
      <pose>-{offset:.3f} -{offset:.3f} 0 0 0 0</pose>
      <link name="walls">{wall_xml}
      </link>
    </model>{marker_xml("start_marker", start_x, start_y, 0.1, 0.85, 0.15)}{marker_xml("goal_marker", goal_x, goal_y, 0.9, 0.15, 0.1)}
  </world>
</sdf>
"""
    # 直接以文本方式写出：SDF/XML 不需要序列化，模板已完全拼好
    with open(output, "w") as f:
        f.write(world)
    # 打印生成摘要（迷宫规模、间距、墙厚高、通道净宽、起终点、墙体数量），
    # 方便与 worlds/maze.world 的实际内容核对
    print(f"world written: {output}")
    print(f"fixed maze: {n}x{n} cells, pitch {P} m, wall {T} m thick / {H} m high")
    print(f"corridor width: {P - T:.2f} m, map size: {side:.2f} x {side:.2f} m")
    print(f"start: ({start_x:.2f}, {start_y:.2f})  goal: ({goal_x:.2f}, {goal_y:.2f})")
    print(f"wall boxes: {len(walls)}")


if __name__ == "__main__":
    # --output 默认写当前目录下的 maze.world，可指定为 worlds/maze.world 来覆盖仓库内文件
    parser = argparse.ArgumentParser(description='由固定布局生成迷宫世界文件')
    parser.add_argument("--output", default="maze.world")
    args = parser.parse_args()
    # 解析固定布局 -> 生成世界文件；布局或通道不通会在上面的函数里直接报错
    size, right, up = parse_layout(FIXED_LAYOUT)
    generate(size, right, up, args.output)