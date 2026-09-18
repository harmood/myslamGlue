"""项目资源目录（地图、数据库等）定位。

原先用 ``Path(get_package_share_directory(...)).parents[3]`` 反推工作空间根
目录，这假设 share 路径恒为 ``<ws>/install/<pkg>/share/<pkg>``。在
``colcon build --merge-install``（``<ws>/install/share/<pkg>``）或安装到
``/opt/ros`` 等结构下会算错，落到 ``/resources/maps`` 之类的位置。

改为按优先级解析：
1. 环境变量 ``MAZE_RESOURCES_DIR``：显式指定资源目录，最可靠；
2. 从安装路径向上寻找同时含 ``src/`` 与 ``install/`` 的工作空间根目录；
3. 兜底 ``~/.ros/maze_maps``。
"""
import os
from pathlib import Path


def workspace_root():
    """返回工作空间根目录（同时含 src/ 与 install/），找不到返回 None。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('maze_slam'))
    except Exception:
        return None
    for parent in share.parents:
        if (parent / 'src').is_dir() and (parent / 'install').is_dir():
            return parent
    return None


def default_map_dir():
    """地图与 rtabmap.db 的默认保存目录。"""
    override = os.environ.get('MAZE_RESOURCES_DIR')
    if override:
        return str(Path(override).expanduser() / 'maps')
    root = workspace_root()
    if root is not None:
        return str(root / 'resources' / 'maps')
    return os.path.expanduser('~/.ros/maze_maps')