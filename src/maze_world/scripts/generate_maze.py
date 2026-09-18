#!/usr/bin/env python3
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
    art = [line for line in layout.strip('\n').splitlines() if line]
    size = len(art)
    if size % 2 == 0:
        raise ValueError('固定布局必须是 (2n+1) x (2n+1) 的奇数尺寸')
    if any(len(line) != size for line in art):
        raise ValueError('固定布局必须是正方形')
    n = (size - 1) // 2
    right = [[True] * n for _ in range(n)]
    up = [[True] * n for _ in range(n)]
    for i in range(n - 1):
        for j in range(n):
            row = 2 * (n - 1 - j) + 1
            right[i][j] = art[row][2 * i + 2] == '|'
    for i in range(n):
        for j in range(n - 1):
            row = 2 * (n - 1 - j)
            up[i][j] = art[row][2 * i + 1] == '-'
    return n, right, up


def path_exists(right, up, n, start, goal):
    seen = {start}
    frontier = [start]
    while frontier:
        i, j = frontier.pop()
        if (i, j) == goal:
            return True
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


P = 1.2
T = 0.2
H = 1.2


def box_xml(name, x, y, sx, sy, diffuse):
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
    walls = []
    span = (n - 1) * P
    walls.append(("outer_south", span / 2, -0.5 - T / 2, span + 1.0 + 2 * T, T))
    walls.append(("outer_north", span / 2, span + 0.5 + T / 2, span + 1.0 + 2 * T, T))
    walls.append(("outer_west", -0.5 - T / 2, span / 2, T, span + 1.0))
    walls.append(("outer_east", span + 0.5 + T / 2, span / 2, T, span + 1.0))
    for i in range(n - 1):
        for j in range(n - 1):
            walls.append((f"pillar_{i}_{j}", (i + 0.5) * P, (j + 0.5) * P, T, T))
    for i in range(n - 1):
        for j in range(n):
            if right[i][j]:
                walls.append((f"east_{i}_{j}", (i + 0.5) * P, j * P, T, P - T))
    for i in range(n):
        for j in range(n - 1):
            if up[i][j]:
                walls.append((f"north_{i}_{j}", i * P, (j + 0.5) * P, P - T, T))
    return walls


def marker_xml(name, x, y, r, g, b):
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
    start = (0, 0)
    goal = (n - 1, n - 1)
    if not path_exists(right, up, n, start, goal):
        raise RuntimeError("no path from start to goal")

    walls = build_walls(n, right, up)
    offset = (n - 1) * P / 2

    wall_xml = "".join(
        box_xml(name, x, y, sx, sy, "0.80 0.80 0.80 1") for name, x, y, sx, sy in walls
    )

    start_x = start[0] * P - offset
    start_y = start[1] * P - offset
    goal_x = goal[0] * P - offset
    goal_y = goal[1] * P - offset
    side = (n - 1) * P + 1.0 + 2 * T

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
    with open(output, "w") as f:
        f.write(world)
    print(f"world written: {output}")
    print(f"fixed maze: {n}x{n} cells, pitch {P} m, wall {T} m thick / {H} m high")
    print(f"corridor width: {P - T:.2f} m, map size: {side:.2f} x {side:.2f} m")
    print(f"start: ({start_x:.2f}, {start_y:.2f})  goal: ({goal_x:.2f}, {goal_y:.2f})")
    print(f"wall boxes: {len(walls)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='由固定布局生成迷宫世界文件')
    parser.add_argument("--output", default="maze.world")
    args = parser.parse_args()
    size, right, up = parse_layout(FIXED_LAYOUT)
    generate(size, right, up, args.output)