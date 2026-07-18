# econ_camera_ws

e-con AR0234 4-camera 모듈용 **ROS2 연속 수집 워크스페이스**.

4대의 카메라를 하드웨어 동기(`frame_sync`)가 맞춰진 상태로 끊김 없이 캡처하여
**ROS2 bag(mcap)** 으로 저장한다. 런치 하나를 실행하면 촬영이 시작되어 `1280x720@30`의 4대
이미지가 `sensor_msgs/CompressedImage`(HW JPEG)로 계속 기록된다. 수집 데이터는 이후 딥러닝
학습(BEV)에 사용한다.

> **상태: 카메라 수집 + 어안 캘리브레이션(Kalibr) 실기 관통 검증 완료.**
> **바로 쓰려면 → 수집은 [`docs/USAGE.md`](docs/USAGE.md), 캘리브는 [`docs/CALIBRATION.md`](docs/CALIBRATION.md).**

## 요구 환경

- Jetson AGX Orin, JetPack 6.1 / L4T R36.4 (Ubuntu 22.04)
- ROS2 **Humble** (호스트 네이티브 설치), `colcon`, `rosbag2` + `rosbag2_storage_mcap`
- 시스템 GStreamer + Python `gi` + HW 플러그인(`nvvidconv`, `nvjpegenc`)
- 선행 프로젝트 `../Multi-Cam_module_test` (순수 로직 `econ_cam` 재사용)

## 구조

```
econ_camera_ws/
  src/econ_camera_ros/          # ament_python 패키지
    econ_camera_ros/
      gst_builder.py            # 연속 수집 파이프라인 문자열 (순수 함수)
      capture_node.py           # 파이프라인 소유 + CompressedImage 발행 + 워밍업 + 동기 로그
      web_monitor_node.py       # 브라우저 2×2 실시간 모니터 (구독 전용, 헤드리스 대응)
      bag_extract.py            # bag(mcap) → 동기 세트별 JPEG 추출
      kalibr_bridge.py          # 동기 세트 → Kalibr 데이터셋(4Hz 다운샘플)
      calib_convert.py          # Kalibr camchain → calib.yaml 사이드카
    launch/record.launch.py     # capture 노드 + ros2 bag record 동시 기동
    test/                       # 순수 로직 pytest (18 passed, 하드웨어 불필요)
  calibration/                  # Kalibr(arm64 Docker) 캘리브레이션 도구
    build_kalibr_arm64.sh       # Kalibr arm64 이미지 빌드(1회)
    run_kalibr.sh               # bagcreater + ds-none/eucm-none 실행
    aprilgrid.yaml              # 타깃 설정(7×5/0.04/0.25)
  docs/
    USAGE.md                    # ★ 상세 사용 가이드 (녹화·모니터·추출)
    CALIBRATION.md              # ★ 캘리브레이션 가이드 (촬영·Kalibr·calib.yaml)
    superpowers/specs/          # 설계 스펙
```

## 빠른 시작

```bash
# 1) econ_cam 순수 로직 재사용
pip install -e ../Multi-Cam_module_test

# 2) 빌드
colcon build --packages-select econ_camera_ros
source install/setup.bash

# 3) 촬영 시작 (capture 노드 + bag 기록 동시 기동)
ros2 launch econ_camera_ros record.launch.py     # → 현재 디렉터리에 rosbag2_<timestamp>/

# 4) (선택) 다른 터미널에서 실시간 확인
ros2 run econ_camera_ros monitor                 # → http://<Orin-IP>:10010

# 5) 촬영 후 bag → 동기 세트별 JPEG 추출
ros2 run econ_camera_ros bag_extract rosbag2_<timestamp> -o dataset
```

시작 시 `capture`는 **워밍업 4초** 동안 프레임을 폐기하며 4대 정상 동작을 확인한 뒤, 같은
주기의 4대가 모인 첫 사이클부터 발행한다(카메라별 시작 프레임 수 일치). 파라미터·문제해결
등 전체 사용법은 **[`docs/USAGE.md`](docs/USAGE.md)** 참조.

토픽: `/camera0/image_raw/compressed` ~ `/camera3/image_raw/compressed`
(`sensor_msgs/CompressedImage`, `header.stamp`=캡처 PTS 앵커, `header.frame_id`="camera{N}").

## 캘리브레이션 (어안 4대 intrinsic + 카메라 간 extrinsic)

Kalibr(arm64 Docker)로 수행하며, 실기에서 전 과정을 관통 검증했다. 요약:

```bash
sudo bash calibration/build_kalibr_arm64.sh                 # (1회) Kalibr 이미지 빌드
ros2 launch econ_camera_ros record.launch.py                # 캘리브 시퀀스 촬영(보드 크게·다포즈)
ros2 run econ_camera_ros bag_extract <bag_dir> -o extracted
ros2 run econ_camera_ros kalibr_bridge extracted -o dataset --rate 4.0
cp calibration/aprilgrid.yaml . && sudo bash calibration/run_kalibr.sh "$(pwd)"
ros2 run econ_camera_ros calib_convert calib-camchain-ds.yaml --model ds -o calib.yaml --rms ...
```

촬영 요령·결과 판정 기준·문제해결 등 전체는 **[`docs/CALIBRATION.md`](docs/CALIBRATION.md)** 참조.
(cam↔LiDAR extrinsic 은 Unitree L2 도착 후 추가 예정.)

## 왜 Argus가 아닌 V4L2인가

e-con 모듈이 UYVY를 직출력(모듈 내부 디베이어링)하여 Jetson ISP를 경유하지 않으므로,
libargus/`nvarguscamerasrc`/Isaac ROS Argus Camera는 이 카메라들을 인식하지 못한다
("No cameras available"). 촬영은 `v4l2src` 순수 V4L2 경로로만 가능하며, 하드웨어 캡처
타임스탬프는 `v4l2src` 버퍼 PTS로 확보된다. 상세: 설계 문서.
