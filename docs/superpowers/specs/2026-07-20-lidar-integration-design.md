# LiDAR SDK 통합 설계 (Unitree L2 → econ_camera_ws)

- 작성일: 2026-07-20
- 브랜치: `feat/lidar-integration`
- 상태: 설계 확정

## 1. 배경 / 목표

현재 `econ_camera_ws`는 e-con AR0234 4-camera 모듈을 하드웨어 동기(`frame_sync`)로
연속 캡처하여 `CompressedImage`(HW JPEG)로 발행·녹화한다. 여기에 **Unitree 4D LiDAR L2**를
추가하여, 최종적으로 **카메라 4대 이미지 + LiDAR PointCloud + LiDAR IMU + TF**가
**한 ROS 시스템에서 동시에 하나의 bag(mcap)으로 저장**되도록 한다.

LiDAR SDK(`unilidar_sdk2`, v2.0.10)는 별도 저장소(`~/Desktop/unilidar_sdk2`)에서
실기 빌드·토픽 수신이 검증된 상태다(작업 기록: 해당 저장소 `L2_ROS2_SETUP.md`).

### 전제 사실 (검증됨)
- LiDAR ROS2 패키지(`unitree_lidar_ros2`, C++)가 다음 토픽을 발행:
  - `/unilidar/cloud` (`sensor_msgs/PointCloud2`, frame `unilidar_lidar`, ~5 Hz)
  - `/unilidar/imu` (`sensor_msgs/Imu`, frame `unilidar_imu`)
  - TF: `unilidar_imu_initial → unilidar_imu → unilidar_lidar`
- 연결은 **이더넷 UDP 모드**: LiDAR `192.168.1.62:6101`, 호스트 `192.168.1.2/24:6201`.
- SDK는 prebuilt 정적 라이브러리 `libunilidar_sdk2.a`(aarch64 / x86_64)를 CMake가 링크.
- 단독 `ros2 run`은 노드 하드코딩 기본값(`lidar_ip`/`local_ip` 뒤바뀜) 때문에 **금지**,
  반드시 launch 파라미터로 실행.

## 2. 통합 방식 (확정 결정)

- **벤더링 = 선택적 복사** (fresh clone 아님). 원본에 사용자가 검증한 로컬 변경
  (`CMakeLists.txt` install 목록 수정 + 신규 `launch_no_rviz.py`)이 있어, clone은 이를
  잃고 upstream 드리프트 위험이 있음. 복사는 검증된 v2.0.10 트리를 그대로 보존한다.
- **SDK 위치 = ws 루트 `third_party/`** (벤더 바이너리를 소스와 분리).
- **런치 = 신규 `record_all.launch.py` 추가**, 기존 `record.launch.py`(카메라만)는 유지.
- **녹화 토픽 = 카메라4 + cloud + imu + tf + tf_static (전부 포함)** — 후속 Cam-LiDAR
  캘리브레이션 대비.

## 3. 디렉터리 구조

```
econ_camera_ws/
├── src/
│   ├── econ_camera_ros/          (기존)
│   └── unitree_lidar_ros2/       ← 복사: 원본 unitree_lidar_ros2/src/unitree_lidar_ros2/
│                                    (src/, include/, launch/, rviz/, CMakeLists.txt, package.xml)
└── third_party/
    └── unitree_lidar_sdk/        ← 복사: include/ + lib/{aarch64,x86_64} 만
                                     (bin/, build/, examples/ 제외)
```

- `build/`, `install/`, `log/`는 복사하지 않고 ws에서 새로 `colcon build`.
- `third_party/`는 colcon 패키지가 아니므로 자동 무시됨(package.xml 없음). 필요 시 `COLCON_IGNORE` 추가.

## 4. CMake 경로 수정

`src/unitree_lidar_ros2/CMakeLists.txt`의 SDK 참조 2줄을 `third_party` 상대경로로 수정
(경로 기준 = CMakeLists.txt 위치 `src/unitree_lidar_ros2/`):

```cmake
include_directories(
  ${PCL_INCLUDE_DIRS}
  include
  ../../third_party/unitree_lidar_sdk/include        # was ../../../unitree_lidar_sdk/include
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ../../third_party/unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}   # was ../../../...
)
```

링크 대상 `libunilidar_sdk2.a` 및 나머지 CMake 내용은 변경하지 않는다.
(사용자가 추가한 launch install 수정분은 복사 시 함께 포함됨.)

## 5. 통합 런치 `record_all.launch.py`

`src/econ_camera_ros/launch/record_all.launch.py` (신규):

- **카메라**: `capture` 노드 (`record.launch.py`와 동일 구성).
- **LiDAR**: 벤더 패키지의 `launch_no_rviz.py`를 `IncludeLaunchDescription`으로 재사용
  → 검증된 파라미터(initialize_type=2, work_mode=0, IP/포트, frame/topic 이름 등)를
  중복 정의 없이 그대로 사용(DRY). launch 파라미터 경로이므로 IP 뒤바뀜 문제 회피.
- **bag record**: 아래 토픽을 명시하여 `ros2 bag record -s mcap` 실행.

Ctrl-C 시 카메라 노드·LiDAR 노드·recorder가 함께 종료.

### 녹화 토픽 (8개)
```
/camera0/image_raw/compressed
/camera1/image_raw/compressed
/camera2/image_raw/compressed
/camera3/image_raw/compressed
/unilidar/cloud
/unilidar/imu
/tf
/tf_static
```
(정확히는 `/tf`·`/tf_static` 포함 8개 토픽. 스토리지 `-s mcap`.)

## 6. 타임스탬프 정합

- 카메라: 첫 프레임 PTS를 ROS 시각(`get_clock().now()`, 기본 시스템 시간)에 앵커링 후
  PTS 델타 누적 → 시스템 시간 기반.
- LiDAR: `use_system_timestamp: True` → 시스템 시간 기반.
- 두 센서 모두 동일 시스템 시간 도메인 → bag 내 카메라·LiDAR 스탬프 직접 비교 가능.
- **경미한 리스크(기록만)**: 카메라는 파이프라인 클럭 델타 누적, LiDAR는 매 메시지
  wall-time이라 장시간 녹화 시 미세 드리프트 가능. 후속 캘리브 정밀도에 영향 시
  하드웨어 시간 동기(PTP 등) 검토 — 현재 범위 밖.

## 7. 빌드 & 검증

### 빌드
```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/econ_camera_ws
rosdep install --from-paths src --ignore-src -r -y   # pcl, pcl_conversions, tf2_ros 등 누락 시
colcon build
source install/setup.bash
```
- 아키텍처별 `.a`는 CMake가 `CMAKE_SYSTEM_PROCESSOR` 기준 자동 선택(Jetson=aarch64).

### 검증 (하드웨어 필요 — 사용자가 LiDAR 연결 상태에서 직접 수행)

LiDAR는 prebuilt C++라 자동 테스트가 없으므로, 아래 체크리스트를 **명령어 → 기대 결과**
형태로 `docs/LIDAR.md`와 `docs/USAGE.md`에 그대로 수록한다. 각 단계는 이전 단계 통과 후 진행.

| # | 명령어 | 기대 결과 (통과 기준) |
|---|--------|----------------------|
| 0-a | `ip -4 addr show \| grep 192.168.1` | 호스트에 `192.168.1.2/24` 할당 표시 |
| 0-b | `ping -c 3 192.168.1.62` | 3/3 응답(0% packet loss) |
| 1 | `colcon build` (ws 루트) | `unitree_lidar_ros2`, `econ_camera_ros` 모두 `Finished`, 에러 0 |
| 2 | `ros2 launch econ_camera_ros record_all.launch.py` | 카메라·LiDAR 노드·recorder 기동, `No cameras`/`initialize_type` 에러 없음, `rosbag2_<ts>/` 생성 |
| 3-a | `ros2 topic hz /camera0/image_raw/compressed` (0~3) | 각 ≈ 30 Hz |
| 3-b | `ros2 topic hz /unilidar/cloud` | ≈ 5 Hz |
| 3-c | `ros2 topic echo /unilidar/imu --once` | Imu 메시지 1건 출력(orientation/angular_velocity 값 존재) |
| 3-d | `ros2 topic list \| grep -E 'tf'` | `/tf`, `/tf_static` 존재 |
| 4 | (Ctrl-C로 종료 후) `ros2 bag info rosbag2_<ts>` | 카메라4·`/unilidar/cloud`·`/unilidar/imu`·`/tf` 7개는 각 message count > 0; `/tf_static`은 현재 발행 노드가 없어 0건이거나 목록에 없을 수 있음(정상 — extrinsic은 `/tf`에 실림); duration 정상 |

- 자동 테스트: 기존 카메라 pytest 18개(`cd src/econ_camera_ros && python3 -m pytest test/`)는 그대로 통과 유지(회귀 확인).

## 8. 문서 업데이트

- **`docs/LIDAR.md` (신규, LiDAR 전용 가이드)**: 하드웨어/네트워크 연결(IP·포트·NIC 설정),
  빌드, 통합 런치 실행, 발행 토픽/좌표계, launch 파라미터, **7절 검증 체크리스트(명령어→기대
  결과)**, 트러블슈팅. 원본 `L2_ROS2_SETUP.md`의 핵심(단독 `ros2 run` 금지, `example_lidar_udp`
  재부팅 주의)을 이 ws 맥락으로 재정리.
- `docs/USAGE.md`: LiDAR 네트워크 사전조건 요약 + `record_all.launch.py` 사용법 +
  bag 확인. 상세는 `docs/LIDAR.md`로 링크.
- `CLAUDE.md`: 현재 상태에 LiDAR 통합(패키지·런치·토픽·`docs/LIDAR.md` 링크) 반영.

## 9. Git

- 본 작업은 `feat/lidar-integration` 브랜치에서 진행.
- 커밋까지만 수행. 병합·푸시는 사용자가 직접(프로젝트 규칙).

## 10. 범위 밖 (YAGNI)

- LiDAR bag → PLY/PCD 추출(`bag_extract` LiDAR 확장) — 후속.
- Cam-LiDAR extrinsic 캘리브레이션 절차 자체 — 후속.
- LiDAR 프레임레이트 변경(Unitree 상위 SW 소관), 시리얼 모드.
- 하드웨어 정밀 시간 동기(PTP 등).
