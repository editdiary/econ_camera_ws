"""카메라 4대 + Unitree L2 LiDAR를 함께 기동하고 단일 bag(mcap)으로 녹화.

실행: ros2 launch econ_camera_ros record_all.launch.py
구성: capture 노드 + LiDAR(unitree_lidar_ros2 launch_no_rviz.py 재사용) + bag record.
녹화 토픽(8): /camera{0..3}/image_raw/compressed, /unilidar/cloud, /unilidar/imu, /tf, /tf_static.
사전조건: 호스트 NIC 192.168.1.2/24, ping 192.168.1.62 OK (LiDAR 이더넷).
Ctrl-C 시 카메라 노드·LiDAR 노드·recorder 함께 종료.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

_DEVICES = [0, 1, 2, 3]
_LIDAR_TOPICS = ["/unilidar/cloud", "/unilidar/imu", "/tf", "/tf_static"]


def generate_launch_description():
    camera_topics = [f"/camera{d}/image_raw/compressed" for d in _DEVICES]
    topics = camera_topics + _LIDAR_TOPICS

    capture = Node(
        package="econ_camera_ros",
        executable="capture",
        name="econ_camera_capture",
        output="screen",
    )

    lidar_launch = os.path.join(
        get_package_share_directory("unitree_lidar_ros2"),
        "launch_no_rviz.py",
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch),
    )

    record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-s", "mcap", *topics],
        output="screen",
    )

    return LaunchDescription([capture, lidar, record])
