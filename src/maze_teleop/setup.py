"""maze_teleop 的安装脚本（ament_python 包）。

安装内容分三类：
    1. ament 资源标记文件，让 ros2 能索引到本包；
    2. package.xml 与 launch 文件，装到 share/maze_teleop 下供 launch 使用；
    3. console_scripts 入口，注册出 ``ros2 run maze_teleop teleop_keyboard``。
"""
from setuptools import find_packages, setup

package_name = 'maze_teleop'

setup(
    name=package_name,
    version='0.1.0',
    # 只安装真正的 Python 包目录，排除测试目录
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 资源标记（resource/maze_teleop，内容即包名）
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 包清单，运行时 ament 会读取
        ('share/' + package_name, ['package.xml']),
        # launch 文件必须显式安装，否则 ros2 launch 找不到
        ('share/' + package_name + '/launch', ['launch/teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghb',
    maintainer_email='ghb@todo.todo',
    description='Keyboard teleoperation for maze_bot in the maze world',
    license='Apache-2.0',
    entry_points={
        # 可执行名 teleop_keyboard -> maze_teleop/teleop_keyboard.py 的 main()
        'console_scripts': [
            'teleop_keyboard = maze_teleop.teleop_keyboard:main',
        ],
    },
)