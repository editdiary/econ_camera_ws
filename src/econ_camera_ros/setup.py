import os
from glob import glob

from setuptools import setup

package_name = 'econ_camera_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DoHyeon_Lee',
    maintainer_email='dhlee@sju.ac.kr',
    description='e-con AR0234 4-camera 연속 동기 수집 ROS2 노드',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture = econ_camera_ros.capture_node:main',
            'monitor = econ_camera_ros.web_monitor_node:main',
            'bag_extract = econ_camera_ros.bag_extract:main',
        ],
    },
)
