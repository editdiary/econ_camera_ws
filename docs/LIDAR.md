# LiDAR (Unitree 4D L2) 사용 가이드

카메라 수집 ws에 통합된 Unitree 4D LiDAR **L2**의 연결·빌드·녹화·검증·문제해결 가이드.
SDK는 `unilidar_sdk2` v2.0.10을 `src/unitree_lidar_ros2`(패키지) + `third_party/unitree_lidar_sdk`(prebuilt SDK)로 벤더링했다.

## 1. 하드웨어 / 네트워크 (이더넷 UDP)

| 항목 | 값 |
|---|---|
| LiDAR IP / port | `192.168.1.62` / `6101` |
| 호스트(Orin) IP / port | `192.168.1.2/24` / `6201` |

호스트 NIC를 `192.168.1.2/24`로 설정해야 통신된다.

```bash
ip -4 addr show | grep 192.168.1     # 호스트에 192.168.1.2/24 있는지
ping -c 3 192.168.1.62               # LiDAR 응답 확인
```

## 2. 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/econ_camera_ws
rosdep install --from-paths src --ignore-src -r -y   # pcl, pcl_conversions, tf2_ros 등 (최초 1회)
colcon build
source install/setup.bash
```
- 아키텍처별 `.a`는 CMake가 `CMAKE_SYSTEM_PROCESSOR` 기준 자동 선택(Jetson=aarch64).

## 3. 실행 (카메라 + LiDAR 통합 녹화)

```bash
ros2 launch econ_camera_ros record_all.launch.py
```
카메라 4대(`capture`) + LiDAR + `ros2 bag record`(mcap)가 함께 뜬다. `rosbag2_<timestamp>/`가 실행 디렉터리에 생성된다. 종료는 `Ctrl-C`(LiDAR는 재부팅되지 않고 스트리밍만 멈춤).

> LiDAR만 단독 확인이 필요하면 `ros2 launch unitree_lidar_ros2 launch_no_rviz.py`.
> **`ros2 run unitree_lidar_ros2 ...` 단독 실행 금지** — 노드 하드코딩 기본값의 `lidar_ip`/`local_ip`가 뒤바뀌어 있음. 반드시 launch로 실행.

## 4. 발행 토픽 / 좌표계

| 토픽 | 타입 | frame_id | 비고 |
|---|---|---|---|
| `/unilidar/cloud` | `sensor_msgs/PointCloud2` | `unilidar_lidar` | ~5 Hz (정상) |
| `/unilidar/imu` | `sensor_msgs/Imu` | `unilidar_imu` | |
| `/tf`, `/tf_static` | tf2 | | `unilidar_imu_initial → unilidar_imu → unilidar_lidar` |

> `/tf_static`는 현재 발행하는 노드가 없어 비어 있을 수 있다(정상). imu→lidar를 포함한 모든
> 프레임이 동적 토픽 `/tf`로 발행되며, `/tf_static`은 향후 cam-lidar 캘리브레이션 대비로 녹화
> 목록에 유지한다.

> **~5 Hz는 정상.** L2 기본 포인트클라우드 프레임레이트이며, 프레임레이트가 낮을수록 프레임당 점 밀도가 높다. 변경은 Unitree 상위 SW 소관.

## 5. 파라미터

launch 파라미터는 벤더 패키지 `launch_no_rviz.py`에 정의되어 있고 통합 런치가 이를 그대로 포함한다. 주요 값: `initialize_type=2`(이더넷/UDP), `work_mode=0`, `lidar_ip/lidar_port=192.168.1.62/6101`, `local_ip/local_port=192.168.1.2/6201`, `cloud_scan_num=18`, `cloud_topic=unilidar/cloud`, `imu_topic=unilidar/imu`.

## 6. 하드웨어 검증 체크리스트 (LiDAR 연결 상태에서 수행)

각 단계는 이전 단계 통과 후 진행. `<ts>`는 생성된 `rosbag2_<timestamp>` 이름.

| # | 명령어 | 기대 결과 (통과 기준) |
|---|--------|----------------------|
| 0-a | `ip -4 addr show \| grep 192.168.1` | 호스트에 `192.168.1.2/24` 표시 |
| 0-b | `ping -c 3 192.168.1.62` | 3 packets, 0% loss |
| 1 | `colcon build` (ws 루트) | `2 packages finished`, 에러 0 |
| 2 | `ros2 launch econ_camera_ros record_all.launch.py` | 노드·LiDAR·recorder 기동, `No cameras`/`initialize_type` 에러 없음, `rosbag2_<ts>/` 생성 |
| 3-a | `ros2 topic hz /camera0/image_raw/compressed` (0~3 각각) | 각 ≈ 30 Hz |
| 3-b | `ros2 topic hz /unilidar/cloud` | ≈ 5 Hz |
| 3-c | `ros2 topic echo /unilidar/imu --once` | Imu 1건(orientation/angular_velocity 값 존재) |
| 3-d | `ros2 topic list \| grep tf` | `/tf` 존재 (`/tf_static`은 현재 미발행이라 없을 수 있음) |
| 4 | (Ctrl-C 후) `ros2 bag info rosbag2_<ts>` | 카메라4·`/unilidar/cloud`·`/unilidar/imu`·`/tf` 7개는 count > 0; `/tf_static`은 현재 발행 노드가 없어 0건이거나 목록에 없을 수 있음(정상 — extrinsic은 `/tf`에 실림); duration 정상 |

## 7. 트러블슈팅

| 증상 | 확인 / 조치 |
|---|---|
| 포인트 없음 | `ros2 topic hz /unilidar/cloud` → 없으면 `ping 192.168.1.62`, 전원/이더넷 확인 |
| `initialize_type is not right! exit now` | launch로 실행했는지 확인(단독 `ros2 run` 금지) |
| UDP 초기화 실패 | 호스트 IP(`192.168.1.2/24`), 방화벽, 포트(6101/6201) 확인 |
| 빌드 시 SDK 헤더/`.a` 못 찾음 | `third_party/unitree_lidar_sdk/{include,lib}` 존재 및 CMake 경로(`../../third_party/...`) 확인 |
| 라이다 반복 재부팅 | `unitree_lidar_sdk/examples/example_lidar_udp`(resetLidar 호출) 반복 실행이 원인 → ROS2 launch만 사용 |
| `Unilidar is not initialized!` 반복 (토픽 미발행) | LiDAR 런치 중복 실행으로 포트/장치 선점 충돌 → [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) T1 참조 |

## 8. 벤더링 구조 / 업데이트

- `src/unitree_lidar_ros2/` — ROS2 패키지(C++). CMake만 SDK 경로 수정.
- `third_party/unitree_lidar_sdk/{include,lib}` — prebuilt SDK(헤더 + `libunilidar_sdk2.a`). `third_party/COLCON_IGNORE`로 colcon 스캔 제외.
- SDK 버전 업데이트 시: 원본 저장소에서 위 두 경로만 다시 복사하고 `colcon build`.
- 설계·결정 근거: [`superpowers/specs/2026-07-20-lidar-integration-design.md`](superpowers/specs/2026-07-20-lidar-integration-design.md)
