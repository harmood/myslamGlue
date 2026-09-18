# maze_world：迷宫 Gazebo 世界资源包
# 本包只有世界文件、启动文件、RViz 配置与一个生成脚本，没有需要编译的代码，
# 因此属于“纯资源包”，只用 ament_cmake 把资源安装到正确位置。
cmake_minimum_required(VERSION 3.8)
project(maze_world)

# ament_cmake 是 ROS 2 的构建系统基础包，ROS 2 包必须声明
find_package(ament_cmake REQUIRED)

# 把这些目录整体安装到 share/maze_world/ 下：
#   worlds  : Gazebo 世界文件（maze.world，由 scripts/generate_maze.py 生成）
#   launch  : 启动文件（maze.launch.py）
#   scripts : 迷宫生成脚本（generate_maze.py）
#   resource: ament 资源索引标记文件
#   rviz    : RViz 配置（maze.rviz）
# 这些文件在运行期由 launch / get_package_share_directory 按路径读取，
# 所以必须安装，不能只留在源码目录里。
install(
  DIRECTORY worlds launch scripts resource rviz
  DESTINATION share/${PROJECT_NAME}
)

# 生成 ament 索引并收尾包配置，必须是最后一条命令
ament_package()
