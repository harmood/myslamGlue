from setuptools import find_packages, setup

package_name = 'maze_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghb',
    maintainer_email='ghb@todo.todo',
    description='Tooling for the maze project (quickstart run log collection)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'quickstart_log_filter = maze_tools.log_filter:main',
        ],
    },
)