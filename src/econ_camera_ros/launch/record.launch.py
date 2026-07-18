"""capture 노드와 ros2 bag record(mcap)를 함께 기동하는 런치.

실행: ros2 launch econ_camera_ros record.launch.py
bag은 실행 디렉터리에 rosbag2_<timestamp>/ 로 자동 생성된다(mcap 스토리지).
Ctrl-C 시 노드와 recorder가 함께 종료된다.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

_DEVICES = [0, 1, 2, 3]


def generate_launch_description():
    topics = [f"/camera{d}/image_raw/compressed" for d in _DEVICES]

    capture = Node(
        package="econ_camera_ros",
        executable="capture",
        name="econ_camera_capture",
        output="screen",
    )

    record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-s", "mcap", *topics],
        output="screen",
    )

    return LaunchDescription([capture, record])
