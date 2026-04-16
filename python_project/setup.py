from setuptools import setup
import os
from glob import glob

package_name = 'python_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='empty',
    maintainer_email='empty@empty.com',
    description='',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = ros_project_scene.control_node:main',
            'lidar_data_proc = ros_project_scene.lidar_data_proc:main',
            'sensor_data_proc = ros_project_scene.sensor_data_proc:main',
            'lidar_logger = ros_project_scene.lidar_logger:main'
        ],
    },
)