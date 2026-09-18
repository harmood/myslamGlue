"""ament_python 包的安装脚本：声明包结构、随包安装的文件和可执行入口。"""
from setuptools import find_packages, setup

package_name = 'maze_tools'

setup(
    name=package_name,
    version='0.1.0',
    # 自动收集 maze_tools/ 下的所有 Python 模块
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 资源索引标记：ROS 2 靠它发现本包（文件内容为包名）
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 包清单，运行时 get_package_share_directory 需要它
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghb',
    maintainer_email='ghb@todo.todo',
    description='Tooling for the maze project (quickstart run log collection)',
    license='Apache-2.0',
    entry_points={
        # 生成 quickstart_log_filter 命令，调用 maze_tools/log_filter.py 的 main()。
        # quickstart.sh 就是通过这个命令把运行输出接进日志过滤器的
        'console_scripts': [
            'quickstart_log_filter = maze_tools.log_filter:main',
        ],
    },
)