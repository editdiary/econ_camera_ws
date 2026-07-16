# econ_camera_ws

e-con AR0234 4-camera 모듈용 **ROS2 연속 수집 워크스페이스**.

4대의 카메라를 하드웨어 동기(`frame_sync`)가 맞춰진 상태로 끊김 없이 캡처하여
**ROS2 bag(mcap)** 으로 저장한다. 런치 하나를 실행하면 촬영이 시작되어 `1280x720@30`의 4대
이미지가 `sensor_msgs/CompressedImage`(HW JPEG)로 계속 기록된다. 수집 데이터는 이후 딥러닝
학습에 사용한다.

> 상태: **설계 확정, 구현 전.** 설계 문서를 먼저 참조하라 —
> `docs/superpowers/specs/2026-07-16-econ-camera-ros2-capture-design.md`

## 요구 환경

- Jetson AGX Orin, JetPack 6.1 / L4T R36.4 (Ubuntu 22.04)
- ROS2 **Humble** (호스트 네이티브 설치), `colcon`, `rosbag2` + `rosbag2_storage_mcap`
- 시스템 GStreamer + Python `gi` + HW 플러그인(`nvvidconv`, `nvjpegenc`)
- 선행 프로젝트 `../Multi-Cam_module_test` (순수 로직 `econ_cam` 재사용)

## 구조

```
econ_camera_ws/
  src/econ_camera_ros/          # ament_python 패키지 (구현 예정)
    econ_camera_ros/
      gst_builder.py            # 연속 수집 파이프라인 문자열 (순수 함수)
      capture_node.py           # 파이프라인 소유 + CompressedImage 발행 + 동기 로그
      monitor_node.py           # 온디맨드 2×2 시각화 (구독 전용)
    launch/record.launch.py     # 카메라 노드 + ros2 bag record 동시 기동
  docs/                         # 설계 스펙 / 구현 계획
```

## 사용 (구현 후 예정)

```bash
# 1) econ_cam 순수 로직 재사용
pip install -e ../Multi-Cam_module_test

# 2) 빌드
colcon build
source install/setup.bash

# 3) 촬영 시작 (노드 + bag 기록 동시 기동)
ros2 launch econ_camera_ros record.launch.py

# 4) (선택) 다른 터미널에서 실시간 확인
ros2 run econ_camera_ros monitor
```

토픽: `/camera0/image_raw/compressed` ~ `/camera3/image_raw/compressed`
(`sensor_msgs/CompressedImage`, `header.stamp`=캡처 PTS, `header.frame_id`="camera{N}").

## 왜 Argus가 아닌 V4L2인가

e-con 모듈이 UYVY를 직출력(모듈 내부 디베이어링)하여 Jetson ISP를 경유하지 않으므로,
libargus/`nvarguscamerasrc`/Isaac ROS Argus Camera는 이 카메라들을 인식하지 못한다
("No cameras available"). 촬영은 `v4l2src` 순수 V4L2 경로로만 가능하며, 하드웨어 캡처
타임스탬프는 `v4l2src` 버퍼 PTS로 확보된다. 상세: 설계 문서 §4.
