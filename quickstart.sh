#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

BUILD=1
TELEOP=1
GUI=1
SLAM=1
LOG=1
GRACE=10
AUX_GRACE=3
LAUNCH_ARGS=()
CHILD_PID=""
TELEOP_PID=""
GUI_PID=""
SLAM_PID=""
CLEANING=0
LAUNCH_PKG="maze_bot"
LAUNCH_FILE="spawn_maze.launch.py"

usage() {
  echo "用法: ./quickstart.sh [--no-build] [--no-teleop] [--no-gui] [--world-only] [ros2 launch 参数...]"
  echo
  echo "  --no-build    跳过 colcon build，直接启动"
  echo "  --no-teleop   不启动键盘遥控"
  echo "  --no-gui      不启动 GUI 状态窗口"
  echo "  --no-slam     不启动视觉 SLAM 建图"
  echo "  --no-log      不采集 ERROR 运行日志"
  echo "  --world-only  只启动迷宫世界（隐含 --no-teleop --no-gui），不生成小车"
  echo "  -h, --help    显示本帮助"
  echo
  echo "默认启动：迷宫世界 + 小车 + ROS 话题桥接 + 键盘遥控 + GUI 状态窗口 + 视觉 SLAM。"
  echo "遥控为全局键盘模式：无需聚焦终端，焦点在 Gazebo/RViz 也可操控。"
  echo "  w/s 前后、a/d 前轮转向、空格急停、F12 暂停/恢复。"
  echo "按 Ctrl+C 可优雅关闭整个项目（遥控会先归零并回正前轮）。"
  echo "每次运行的 ERROR 日志保存到 resources/logs/quickstart_<时间戳>.log。"
}

for arg in "$@"; do
  case "$arg" in
    --no-build) BUILD=0 ;;
    --no-teleop) TELEOP=0 ;;
    --no-gui) GUI=0 ;;
    --no-slam) SLAM=0 ;;
    --no-log) LOG=0 ;;
    --world-only) LAUNCH_PKG="maze_world"; LAUNCH_FILE="maze.launch.py"; TELEOP=0; GUI=0; SLAM=0 ;;
    -h|--help) usage; exit 0 ;;
    *) LAUNCH_ARGS+=("$arg") ;;
  esac
done

LOG_DIR="$SCRIPT_DIR/resources/logs"
LOG_FILE="$LOG_DIR/quickstart_$(date +%Y%m%d_%H%M%S).log"
LOG_FILTER="$SCRIPT_DIR/src/maze_tools/maze_tools/log_filter.py"
if [ "$LOG" -eq 1 ] && [ -f "$LOG_FILTER" ] && command -v python3 >/dev/null 2>&1; then
  mkdir -p "$LOG_DIR"
  exec > >(python3 "$LOG_FILTER" "$LOG_FILE" "$0 $*") 2>&1
  echo "==> 本次运行的 ERROR 日志将保存到: $LOG_FILE"
fi

if [ -z "${ROS_DISTRO:-}" ]; then
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
  else
    echo "未检测到 ROS 2 环境，请先 source /opt/ros/<distro>/setup.bash" >&2
    exit 1
  fi
fi

if [ "$GUI" -eq 1 ] && [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
  echo "==> 未检测到图形显示，跳过 GUI 窗口"
  GUI=0
fi

if [ "$BUILD" -eq 1 ] || [ ! -f install/setup.bash ]; then
  echo "==> colcon build --packages-select maze_world maze_bot maze_teleop maze_gui maze_slam maze_tools --symlink-install"
  colcon build --packages-select maze_world maze_bot maze_teleop maze_gui maze_slam maze_tools --symlink-install
fi

set +u
source install/setup.bash
set -u

session_pids() {
  ps -eo pid=,sid= | awk -v s="$CHILD_PID" -v me="$$" -v ch="$CHILD_PID" \
    '$1 != me && $1 != ch && $2 == s { print $1 }'
}

force_kill() {
  kill -KILL -"$CHILD_PID" 2>/dev/null || true
  local pids
  pids="$(session_pids)"
  if [ -n "$pids" ]; then
    kill -KILL $pids 2>/dev/null || true
  fi
}

sweep() {
  local pids
  pids="$(session_pids)"
  if [ -n "$pids" ]; then
    echo "==> 清理残留进程: $pids"
    kill -TERM $pids 2>/dev/null || true
    sleep 1
    pids="$(session_pids)"
    if [ -n "$pids" ]; then
      kill -KILL $pids 2>/dev/null || true
    fi
  fi
}

stop_group() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  if kill -0 -"$pid" 2>/dev/null; then
    kill -INT -"$pid" 2>/dev/null || true
    local i
    for i in $(seq 1 $((AUX_GRACE * 10))); do
      kill -0 -"$pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 -"$pid" 2>/dev/null; then
      kill -TERM -"$pid" 2>/dev/null || true
      sleep 1
      kill -KILL -"$pid" 2>/dev/null || true
    fi
  fi
  wait "$pid" 2>/dev/null || true
}

stop_aux() {
  stop_group "$TELEOP_PID"
  TELEOP_PID=""
  stop_group "$GUI_PID"
  GUI_PID=""
  stop_group "$SLAM_PID"
  SLAM_PID=""
}

on_signal() {
  if [ "$CLEANING" -eq 1 ]; then
    echo
    echo "==> 再次收到中断，强制结束"
    stop_aux
    force_kill
    return
  fi
  CLEANING=1
  echo
  echo "==> 正在优雅关闭遥控、GUI、SLAM 与 Gazebo ..."
  stop_aux
  kill -INT -"$CHILD_PID" 2>/dev/null || true
  local i
  for i in $(seq 1 $((GRACE * 10))); do
    kill -0 -"$CHILD_PID" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 -"$CHILD_PID" 2>/dev/null; then
    echo "==> 关闭超时，发送 SIGTERM"
    kill -TERM -"$CHILD_PID" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 -"$CHILD_PID" 2>/dev/null; then
    echo "==> 进程未退出，强制结束"
    force_kill
  fi
}

cleanup_exit() {
  stop_aux
  sweep
  sleep 0.5
}

echo "==> ros2 launch ${LAUNCH_PKG} ${LAUNCH_FILE} ${LAUNCH_ARGS[*]:-}"
setsid ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" "${LAUNCH_ARGS[@]}" &
CHILD_PID=$!

if [ "$TELEOP" -eq 1 ]; then
  echo "==> ros2 run maze_teleop teleop_keyboard（全局键盘：w/s 前后、a/d 转向、F12 暂停）"
  setsid ros2 run maze_teleop teleop_keyboard &
  TELEOP_PID=$!
fi

if [ "$GUI" -eq 1 ]; then
  echo "==> ros2 run maze_gui maze_dashboard（控制说明与小车状态窗口）"
  setsid ros2 run maze_gui maze_dashboard &
  GUI_PID=$!
fi

if [ "$SLAM" -eq 1 ]; then
  echo "==> ros2 launch maze_slam slam.launch.py（视觉 SLAM，默认停止，按 m 或 GUI 按钮开始建图）"
  setsid ros2 launch maze_slam slam.launch.py &
  SLAM_PID=$!
fi

trap on_signal INT TERM HUP
trap cleanup_exit EXIT

STATUS=0
wait "$CHILD_PID" || STATUS=$?

trap - INT TERM HUP
stop_aux

if [ "$STATUS" -eq 0 ]; then
  echo "==> 已全部关闭"
else
  echo "==> 已全部关闭 (退出码 $STATUS)"
fi
exit "$STATUS"