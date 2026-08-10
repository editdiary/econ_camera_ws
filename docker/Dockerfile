# 수집 완료 데이터 "후처리 전용" 이미지 (서버 컴퓨터용)
#
# 범위: bag 추출 / BEV auto-label / IPM / calib 검증 / PCD·BEV 미리보기 / 순수 로직 테스트.
# 비범위: 카메라 촬영(v4l2·GStreamer), LiDAR 실시간 수집, Point-LIO 매핑(colcon 빌드).
#         후처리 코드는 rosbag2_py/rclpy 를 함수 안에서 지연 import 하는 평범한 스크립트라
#         colcon 빌드 없이 `python3 <경로>` 로 그대로 돌아간다.
#
# base 를 ros:humble 로 잡는 이유: Ubuntu 22.04 + Python 3.10 조합이 원래 Jetson 환경과
# 같아 검증 부담이 가장 적고, rosbag2(mcap) 런타임을 그대로 얻는다.
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

# - rosbag2-storage-mcap: mcap bag 읽기(ros-base 이미지엔 없음). bag_extract/check_lidar_bag 필수.
# - libgl1/libgomp1/libusb-1.0-0: open3d 런타임 의존(헤드리스로 파일 IO 만 해도 로드됨).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rosbag2-storage-mcap \
        python3-pip \
        libgl1 \
        libgomp1 \
        libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# numpy 는 반드시 1.x. 이유(둘 다 해당):
#   1) Humble 의 rclpy/rosbag2_py 가 numpy 1.x 에 대해 빌드돼 있어 2.x 를 얹으면 bag 읽기가 깨진다.
#   2) open3d 0.18 은 numpy<2 전용.
# opencv 는 헤드리스 서버용이므로 -headless (GUI 심볼 없음 → pick_correspondences 는 여기서 못 씀).
RUN pip3 install --no-cache-dir \
        numpy==1.26.4 \
        scipy==1.13.1 \
        opencv-python-headless==4.10.0.84 \
        open3d==0.18.0 \
        PyYAML==6.0.2 \
        Pillow==10.4.0 \
        mcap==1.2.1 \
        pytest==8.3.3

# ros_entrypoint.sh(base 이미지 제공)가 /opt/ros/humble/setup.bash 를 소스한 뒤 CMD 를 exec 한다.
WORKDIR /ws
CMD ["bash"]
