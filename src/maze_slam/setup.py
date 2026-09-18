"""ament_python 包的安装脚本：声明包结构、随包安装的资源文件和可执行入口。"""
from setuptools import find_packages, setup

package_name = 'maze_slam'

setup(
    name=package_name,
    version='0.1.0',
    # 自动收集 maze_slam/ 下的所有 Python 模块
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 资源索引标记：ROS 2 靠它发现本包（内容为包名）
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 包清单，运行时 get_package_share_directory 需要它
        ('share/' + package_name, ['package.xml']),
        # launch 文件必须显式安装，否则 ros2 launch 找不到
        ('share/' + package_name + '/launch', ['launch/slam.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghb',
    maintainer_email='ghb@todo.todo',
    description='Visual SLAM (RTAB-Map) for maze_bot with mapping switch and map saving',
    license='Apache-2.0',
    entry_points={
        # 生成名为 slam_manager 的可执行命令，调用 maze_slam/slam_manager.py 的 main()；
        # launch 文件中的 executable='slam_manager' 就指向这里
        'console_scripts': [
            'slam_manager = maze_slam.slam_manager:main',
        ],
    },
)