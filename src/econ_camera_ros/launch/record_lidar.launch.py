"""Unitree L2 LiDAR만 기동하고 단일 bag(mcap)으로 녹화.

실행: ros2 launch econ_camera_ros record_lidar.launch.py
구성: LiDAR(unitree_lidar_ros2 launch_no_rviz.py 재사용) + bag record. 카메라 미기동.
녹화 토픽(4): /unilidar/cloud, /unilidar/imu, /tf, /tf_static.
사전조건: 호스트 NIC 192.168.1.2/24, ping 192.168.1.62 OK (LiDAR 이더넷).
Ctrl-C 시 LiDAR 노드·recorder 함께 종료.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

_LIDAR_TOPICS = ["/unilidar/cloud", "/unilidar/imu", "/tf", "/tf_static"]


def generate_launch_description():
    lidar_launch = os.path.join(
        get_package_share_directory("unitree_lidar_ros2"),
        "launch_no_rviz.py",
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch),
    )

    record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-s", "mcap", *_LIDAR_TOPICS],
        output="screen",
    )

    return LaunchDescription([lidar, record])
