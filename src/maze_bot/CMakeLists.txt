# maze_bot：四轮小车模型与仿真启动所需的资源包
# 本包没有需要编译的 C++ 代码，只用 ament_cmake 把模型/配置/脚本安装到
# 正确位置，因此属于“纯资源包”。
cmake_minimum_required(VERSION 3.8)
project(maze_bot)

# ament_cmake 是 ROS 2 的构建系统基础包，ROS 2 包必须声明
find_package(ament_cmake REQUIRED)

# 把模型与配置目录整体安装到 share/maze_bot/ 下：
#   urdf     : 机器人模型（maze_bot.urdf.xacro）
#   launch   : 启动文件（display.launch.py、spawn_maze.launch.py）
#   rviz     : RViz 配置（maze_bot.rviz）
#   resource : ament 资源索引标记文件
#   config   : ros_gz_bridge 桥接配置（ros_gz_bridge.yaml）
# 这些文件运行期由 launch / get_package_share_directory 按路径读取，
# 所以必须安装，不能只留在源码目录里。
install(
  DIRECTORY urdf launch rviz resource config
  DESTINATION share/${PROJECT_NAME}
)

# 把 Python 脚本安装到 lib/maze_bot/ 并赋予可执行权限。
# 用 PROGRAMS（而非 FILES）才能保留执行位，launch 里可直接以
# executable='camera_relay.py' 的方式启动它们。
install(
  PROGRAMS scripts/camera_relay.py scripts/lidar_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# 生成 ament 索引并收尾包配置，必须是最后一条命令
ament_package()