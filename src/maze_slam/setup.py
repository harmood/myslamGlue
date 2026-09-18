from setuptools import find_packages, setup

package_name = 'maze_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghb',
    maintainer_email='ghb@todo.todo',
    description='Visual SLAM (RTAB-Map) for maze_bot with mapping switch and map saving',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'slam_manager = maze_slam.slam_manager:main',
        ],
    },
)