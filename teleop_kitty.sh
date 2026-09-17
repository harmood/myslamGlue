#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v kitty >/dev/null 2>&1; then
  echo "未找到 kitty，请先安装：sudo apt install kitty" >&2
  exit 1
fi

if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
  echo "工作区尚未构建，请先运行 ./quickstart.sh" >&2
  exit 1
fi

exec kitty --title "maze_bot 键盘遥控" --working-directory "$SCRIPT_DIR" \
  bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch maze_teleop teleop.launch.py $*"