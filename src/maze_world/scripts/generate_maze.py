#!/usr/bin/env python3
import argparse
import random


def carve_maze(n, rng):
    right = [[True] * n for _ in range(n)]
    up = [[True] * n for _ in range(n)]
    seen = [[False] * n for _ in range(n)]
    stack = [(0, 0)]
    seen[0][0] = True
    while stack:
        i, j = stack[-1]
        nbrs = []
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < n and 0 <= b < n and not seen[a][b]:
                nbrs.append((a, b))
        if not nbrs:
            stack.pop()
            continue
        a, b = rng.choice(nbrs)
        if a > i:
            right[i][j] = False
        elif a < i:
            right[a][b] = False
        elif b > j:
            up[i][j] = False
        else:
            up[a][b] = False
        seen[a][b] = True
        stack.append((a, b))
    return right, up


def add_loops(right, up, n, rng, p):
    for i in range(n - 1):
        for j in range(n):
            if right[i][j] and rng.random() < p:
                right[i][j] = False
    for i in range(n):
        for j in range(n - 1):
            if up[i][j] and rng.random() < p:
                up[i][j] = False


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


def generate(n, seed, loop_ratio, output):
    rng = random.Random(seed)
    right, up = carve_maze(n, rng)
    add_loops(right, up, n, rng, loop_ratio)
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
    print(f"maze: {n}x{n} cells, pitch {P} m, wall {T} m thick / {H} m high")
    print(f"corridor width: {P - T:.2f} m, map size: {side:.2f} x {side:.2f} m")
    print(f"start: ({start_x:.2f}, {start_y:.2f})  goal: ({goal_x:.2f}, {goal_y:.2f})")
    print(f"wall boxes: {len(walls)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--size", type=int, default=9)
    parser.add_argument("--seed", type=int, default=20260915)
    parser.add_argument("--loops", type=float, default=0.08)
    parser.add_argument("--output", default="maze.world")
    args = parser.parse_args()
    generate(args.size, args.seed, args.loops, args.output)