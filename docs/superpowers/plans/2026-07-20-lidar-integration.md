# LiDAR SDK 통합 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Unitree 4D LiDAR L2를 `econ_camera_ws`로 벤더링하여, 카메라 4대 + LiDAR PointCloud + LiDAR IMU + TF를 단일 bag(mcap)으로 동시 녹화한다.

**Architecture:** LiDAR ROS2 패키지(`unitree_lidar_ros2`, C++)를 `src/`로, prebuilt SDK(헤더+`.a`)를 `third_party/`로 선택 복사한다. CMake의 SDK 상대경로만 수정하고, 새 통합 런치 `record_all.launch.py`가 카메라 노드 + 벤더 LiDAR 런치(`launch_no_rviz.py`) + `ros2 bag record`를 함께 기동한다.

**Tech Stack:** ROS2 Humble, colcon, ament_cmake(C++ LiDAR 패키지) / ament_python(기존 카메라 패키지), rosbag2 mcap, Jetson AGX Orin(aarch64).

## Global Constraints

- 원본 SDK 저장소: `~/Desktop/unilidar_sdk2` (v2.0.10, 사용자 검증 완료). **fresh clone 금지 — 선택 복사만** (검증된 로컬 변경 보존).
- SDK는 **prebuilt 정적 라이브러리** `libunilidar_sdk2.a` (`lib/aarch64`, `lib/x86_64`)를 링크. 소스 재빌드 없음.
- `build/`, `install/`, `log/`는 복사하지 않음. ws에서 새로 `colcon build`.
- LiDAR 노드는 **단독 `ros2 run` 금지** (하드코딩 기본값 `lidar_ip`/`local_ip` 뒤바뀜). 반드시 launch 파라미터로 실행.
- 녹화 토픽 8개: `/camera{0..3}/image_raw/compressed`, `/unilidar/cloud`, `/unilidar/imu`, `/tf`, `/tf_static`. 스토리지 `-s mcap`.
- LiDAR 네트워크: LiDAR `192.168.1.62:6101`, 호스트 NIC `192.168.1.2/24:6201`, 이더넷 UDP.
- Git: `feat/lidar-integration` 브랜치에서만 작업, **커밋까지만**(병합/푸시는 사용자). 커밋 트레일러: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- 기존 카메라 pytest 18개 회귀 없음: `cd src/econ_camera_ros && python3 -m pytest test/`.

## File Structure

**생성/복사:**
- `src/unitree_lidar_ros2/` ← 원본 `unitree_lidar_ros2/src/unitree_lidar_ros2/` 복사 (C++ LiDAR 패키지: `src/`, `include/`, `launch/`, `rviz/`, `CMakeLists.txt`, `package.xml`)
- `third_party/unitree_lidar_sdk/include/` ← SDK 헤더 (7개 `.h`)
- `third_party/unitree_lidar_sdk/lib/{aarch64,x86_64}/libunilidar_sdk2.a` ← prebuilt 라이브러리
- `third_party/COLCON_IGNORE` ← colcon이 `third_party/`를 패키지 스캔에서 제외
- `src/econ_camera_ros/launch/record_all.launch.py` ← 통합 런치 (신규)
- `docs/LIDAR.md` ← LiDAR 전용 가이드 (신규)

**수정:**
- `src/unitree_lidar_ros2/CMakeLists.txt` ← SDK 경로 2줄 (`../../../` → `../../third_party/`)
- `docs/USAGE.md` ← 9절(향후 LiDAR)을 실제 사용법으로 갱신
- `CLAUDE.md` ← 현재 상태에 LiDAR 통합 반영

**변경 없음:** `src/econ_camera_ros/setup.py` (launch glob이 `record_all.launch.py`를 자동 설치), 기존 카메라 소스/테스트.

---

## Task 1: LiDAR 패키지 + SDK 벤더링 (선택 복사)

**Files:**
- Create: `src/unitree_lidar_ros2/` (원본 패키지 복사본)
- Create: `third_party/unitree_lidar_sdk/{include,lib}/`
- Create: `third_party/COLCON_IGNORE`

**Interfaces:**
- Consumes: 원본 `~/Desktop/unilidar_sdk2`.
- Produces: ws 내부 벤더 트리. Task 2의 CMake가 `third_party/unitree_lidar_sdk/{include,lib}`를 참조.

- [ ] **Step 1: LiDAR ROS2 패키지를 src/로 복사 (빌드 산출물·pyc 제외)**

```bash
cd ~/Desktop/econ_camera_ws
rsync -a --exclude='__pycache__' --exclude='*.pyc' \
  ~/Desktop/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2/ \
  src/unitree_lidar_ros2/
```

- [ ] **Step 2: SDK 헤더 + 라이브러리만 third_party/로 복사**

```bash
cd ~/Desktop/econ_camera_ws
mkdir -p third_party/unitree_lidar_sdk
rsync -a ~/Desktop/unilidar_sdk2/unitree_lidar_sdk/include third_party/unitree_lidar_sdk/
rsync -a ~/Desktop/unilidar_sdk2/unitree_lidar_sdk/lib     third_party/unitree_lidar_sdk/
```

- [ ] **Step 3: third_party/에 COLCON_IGNORE 추가**

```bash
touch ~/Desktop/econ_camera_ws/third_party/COLCON_IGNORE
```

- [ ] **Step 4: 복사 결과 검증**

```bash
cd ~/Desktop/econ_camera_ws
ls src/unitree_lidar_ros2/{package.xml,CMakeLists.txt}
ls src/unitree_lidar_ros2/src/unitree_lidar_ros2_node.cpp
ls src/unitree_lidar_ros2/launch/launch_no_rviz.py
ls third_party/unitree_lidar_sdk/include/unitree_lidar_sdk.h
ls third_party/unitree_lidar_sdk/lib/aarch64/libunilidar_sdk2.a
ls third_party/COLCON_IGNORE
# 빌드 산출물이 딸려오지 않았는지 확인 (출력 없어야 정상)
find src/unitree_lidar_ros2 third_party -type d -name build -o -type d -name install -o -type d -name __pycache__
```
Expected: 앞의 `ls`들은 모두 파일 경로 출력(에러 없음), 마지막 `find`는 **출력 없음**.

- [ ] **Step 5: Commit**

```bash
cd ~/Desktop/econ_camera_ws
git add src/unitree_lidar_ros2 third_party
git commit -m "$(cat <<'EOF'
feat(lidar): unitree_lidar_ros2 패키지 + SDK 벤더링

원본 unilidar_sdk2 v2.0.10에서 ROS2 패키지는 src/로, prebuilt SDK(헤더+.a)는
third_party/로 선택 복사(build/install/log 제외). third_party는 COLCON_IGNORE.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: CMake SDK 경로 수정 + 전체 빌드

**Files:**
- Modify: `src/unitree_lidar_ros2/CMakeLists.txt` (SDK 경로 2줄)

**Interfaces:**
- Consumes: Task 1의 `third_party/unitree_lidar_sdk/{include,lib}`.
- Produces: 빌드 산출물 `install/unitree_lidar_ros2/lib/unitree_lidar_ros2/unitree_lidar_ros2_node`, 설치된 launch (`share/unitree_lidar_ros2/launch_no_rviz.py`). Task 3의 런치가 이 share 경로를 참조.

**참고 — 경로 기준:** CMake 상대경로는 CMakeLists.txt 위치(`src/unitree_lidar_ros2/`) 기준. `third_party/`는 ws 루트에 있으므로 2단계 상위: `../../third_party/...` (원본은 3단계 상위 `../../../`였음).

- [ ] **Step 1: include_directories 경로 수정**

`src/unitree_lidar_ros2/CMakeLists.txt`에서:

```cmake
include_directories(
  ${PCL_INCLUDE_DIRS} 
  include
  ../../../unitree_lidar_sdk/include
)
```
를 다음으로 변경:
```cmake
include_directories(
  ${PCL_INCLUDE_DIRS} 
  include
  ../../third_party/unitree_lidar_sdk/include
)
```

- [ ] **Step 2: link_directories 경로 수정**

같은 파일에서:

```cmake
link_directories(
  ${PCL_LIBRARY_DIRS}
  ../../../unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}
)
```
를 다음으로 변경:
```cmake
link_directories(
  ${PCL_LIBRARY_DIRS}
  ../../third_party/unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}
)
```

- [ ] **Step 3: C++ 의존성 설치 (누락 대비)**

```bash
cd ~/Desktop/econ_camera_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```
Expected: `#All required rosdeps installed successfully` 또는 이미 충족.

- [ ] **Step 4: 전체 빌드**

```bash
cd ~/Desktop/econ_camera_ws
source /opt/ros/humble/setup.bash
colcon build
```
Expected: `Summary: 2 packages finished` (`econ_camera_ros`, `unitree_lidar_ros2`), 에러 0. SDK 헤더/라이브러리 못 찾는 에러 없음.

- [ ] **Step 5: 빌드 산출물 확인**

```bash
cd ~/Desktop/econ_camera_ws
ls install/unitree_lidar_ros2/lib/unitree_lidar_ros2/unitree_lidar_ros2_node
ls install/unitree_lidar_ros2/share/unitree_lidar_ros2/launch_no_rviz.py
```
Expected: 두 파일 모두 존재.

- [ ] **Step 6: 카메라 pytest 회귀 확인**

```bash
cd ~/Desktop/econ_camera_ws/src/econ_camera_ros && python3 -m pytest test/ -q
```
Expected: 18 passed.

- [ ] **Step 7: Commit**

```bash
cd ~/Desktop/econ_camera_ws
git add src/unitree_lidar_ros2/CMakeLists.txt
git commit -m "$(cat <<'EOF'
fix(lidar): CMake SDK 경로를 third_party/ 기준으로 수정

벤더링 위치(ws third_party/)에 맞춰 include/link 경로를 ../../../ → ../../third_party/로.
colcon build로 unitree_lidar_ros2_node 생성 확인.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: 통합 런치 `record_all.launch.py`

**Files:**
- Create: `src/econ_camera_ros/launch/record_all.launch.py`

**Interfaces:**
- Consumes: 카메라 `capture` 실행자(`econ_camera_ros` console_scripts, `record.launch.py`와 동일), Task 2가 설치한 `share/unitree_lidar_ros2/launch_no_rviz.py`.
- Produces: `ros2 launch econ_camera_ros record_all.launch.py` — 카메라 노드 + LiDAR 노드 + 8토픽 bag record 동시 기동.

**참고:** `setup.py`의 `data_files` glob(`launch/*.launch.py`)이 이 파일을 자동 설치하므로 setup.py 수정 불필요. 벤더 CMakeLists는 launch 파일을 `share/unitree_lidar_ros2/`(flat)에 설치하므로 include 경로에 `launch/` 하위 디렉터리가 없음에 유의.

- [ ] **Step 1: 통합 런치 파일 작성**

`src/econ_camera_ros/launch/record_all.launch.py`:

```python
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
```

- [ ] **Step 2: 재빌드하여 런치 설치**

```bash
cd ~/Desktop/econ_camera_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select econ_camera_ros
ls install/econ_camera_ros/share/econ_camera_ros/launch/record_all.launch.py
```
Expected: `record_all.launch.py` 설치 확인.

- [ ] **Step 3: 런치 파일 파싱 검증 (하드웨어 불필요)**

`ros2 launch --print`은 실행하지 않고 LaunchDescription 생성만 확인한다(카메라/LiDAR 미연결이어도 파싱은 성공해야 함).

```bash
cd ~/Desktop/econ_camera_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch econ_camera_ros record_all.launch.py --print
```
Expected: 액션 트리에 `capture`(Node), 포함된 `unitree_lidar_ros2_node`(Node), `ros2 bag record ...`(ExecuteProcess)가 표시되고 파싱 에러 없음. (`get_package_share_directory("unitree_lidar_ros2")`가 해석되어야 하므로 Task 2 빌드가 선행되어야 함.)

- [ ] **Step 4: Commit**

```bash
cd ~/Desktop/econ_camera_ws
git add src/econ_camera_ros/launch/record_all.launch.py
git commit -m "$(cat <<'EOF'
feat(lidar): 카메라+LiDAR 통합 녹화 런치 record_all.launch.py

capture 노드 + 벤더 LiDAR launch_no_rviz.py(IncludeLaunchDescription 재사용) +
ros2 bag record(-s mcap, 8토픽) 동시 기동. 기존 record.launch.py는 유지.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: 문서 (LIDAR.md 신규 + USAGE.md/CLAUDE.md 갱신)

**Files:**
- Create: `docs/LIDAR.md`
- Modify: `docs/USAGE.md` (9절 교체)
- Modify: `CLAUDE.md` (현재 상태에 LiDAR 반영)

**Interfaces:**
- Consumes: Task 1~3 결과(경로·런치·토픽).
- Produces: 사용자가 하드웨어 검증(Task 5)을 그대로 따라 할 수 있는 문서.

- [ ] **Step 1: `docs/LIDAR.md` 작성**

`docs/LIDAR.md` 전체 내용:

````markdown
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
| 3-d | `ros2 topic list \| grep tf` | `/tf`, `/tf_static` 존재 |
| 4 | (Ctrl-C 후) `ros2 bag info rosbag2_<ts>` | 카메라4·`/unilidar/cloud`·`/unilidar/imu`·`/tf` 7개는 count > 0; `/tf_static`은 현재 발행 노드가 없어 0건이거나 목록에 없을 수 있음(정상 — extrinsic은 `/tf`에 실림); duration 정상 |

## 7. 트러블슈팅

| 증상 | 확인 / 조치 |
|---|---|
| 포인트 없음 | `ros2 topic hz /unilidar/cloud` → 없으면 `ping 192.168.1.62`, 전원/이더넷 확인 |
| `initialize_type is not right! exit now` | launch로 실행했는지 확인(단독 `ros2 run` 금지) |
| UDP 초기화 실패 | 호스트 IP(`192.168.1.2/24`), 방화벽, 포트(6101/6201) 확인 |
| 빌드 시 SDK 헤더/`.a` 못 찾음 | `third_party/unitree_lidar_sdk/{include,lib}` 존재 및 CMake 경로(`../../third_party/...`) 확인 |
| 라이다 반복 재부팅 | `unitree_lidar_sdk/examples/example_lidar_udp`(resetLidar 호출) 반복 실행이 원인 → ROS2 launch만 사용 |

## 8. 벤더링 구조 / 업데이트

- `src/unitree_lidar_ros2/` — ROS2 패키지(C++). CMake만 SDK 경로 수정.
- `third_party/unitree_lidar_sdk/{include,lib}` — prebuilt SDK(헤더 + `libunilidar_sdk2.a`). `third_party/COLCON_IGNORE`로 colcon 스캔 제외.
- SDK 버전 업데이트 시: 원본 저장소에서 위 두 경로만 다시 복사하고 `colcon build`.
- 설계·결정 근거: [`superpowers/specs/2026-07-20-lidar-integration-design.md`](superpowers/specs/2026-07-20-lidar-integration-design.md)
````

- [ ] **Step 2: `docs/USAGE.md` 9절 교체**

`docs/USAGE.md`의 기존 9절 전체(제목 `## 9. 향후: LiDAR 추가 → BEV`부터 파일 끝까지)를 다음으로 교체:

```markdown
## 9. LiDAR 함께 수집 (카메라 + LiDAR 통합 bag)

Unitree 4D LiDAR L2(IMU 내장)가 같은 ws에 통합되어 있다. 카메라 4대 + LiDAR 점군/IMU/TF를
**단일 bag(mcap)** 으로 동시 녹화한다.

```bash
# 사전조건: 호스트 NIC 192.168.1.2/24, ping 192.168.1.62 OK
ros2 launch econ_camera_ros record_all.launch.py
```

녹화 토픽(8): `/camera{0..3}/image_raw/compressed`, `/unilidar/cloud`, `/unilidar/imu`, `/tf`, `/tf_static`.
`ros2 bag info <bag>`로 8토픽·메시지 수를 확인한다.

- 연결·빌드·검증·문제해결 상세: [`LIDAR.md`](LIDAR.md)
- 캘리브레이션(intrinsic/extrinsic)은 **정적 상수** → bag에 넣지 않고 `calib.yaml` 사이드카.
- 동적 데이터(점군·IMU)는 **주행 중 bag에 기록** → ego-pose는 사후 복원(LIO).
- 융합/BEV 설계: [`superpowers/specs/2026-07-18-data-collection-bag-and-fusion-design.md`](superpowers/specs/2026-07-18-data-collection-bag-and-fusion-design.md)
```

- [ ] **Step 3: `CLAUDE.md` 현재 상태에 LiDAR 반영**

`CLAUDE.md`에서 `record.launch.py` 항목(라인 14 부근) 바로 다음에 아래 두 줄을 추가:

```markdown
- `record_all.launch.py`: `capture` + **LiDAR**(`unitree_lidar_ros2`) + `ros2 bag record -s mcap`
  (카메라4 + `/unilidar/cloud`·`/unilidar/imu` + `/tf`·`/tf_static`, 8토픽) 동시 기동.
- **LiDAR**(Unitree 4D L2): `src/unitree_lidar_ros2`(벤더 패키지) + `third_party/unitree_lidar_sdk`
  (prebuilt SDK). 이더넷 UDP(호스트 `192.168.1.2/24`). 절차·검증은 `docs/LIDAR.md`. 실기 검증만 남음.
```

- [ ] **Step 4: 문서 링크/정합 확인**

```bash
cd ~/Desktop/econ_camera_ws
ls docs/LIDAR.md
grep -n "record_all.launch.py" docs/USAGE.md CLAUDE.md
grep -n "LIDAR.md" docs/USAGE.md CLAUDE.md
```
Expected: `docs/LIDAR.md` 존재, `record_all.launch.py`가 USAGE·CLAUDE 양쪽에, `LIDAR.md` 링크가 양쪽에 존재.

- [ ] **Step 5: Commit**

```bash
cd ~/Desktop/econ_camera_ws
git add docs/LIDAR.md docs/USAGE.md CLAUDE.md
git commit -m "$(cat <<'EOF'
docs(lidar): LIDAR.md 전용 가이드 + USAGE/CLAUDE LiDAR 통합 반영

연결·빌드·통합 런치·발행 토픽·검증 체크리스트·트러블슈팅 문서화.
USAGE 9절을 실제 사용법으로 갱신, CLAUDE 현재 상태에 record_all·벤더 구조 반영.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: 하드웨어 통합 검증 (사용자가 LiDAR 연결 상태에서 수행)

**Files:** 없음 (실기 검증 — 코드 변경 없음).

**Interfaces:**
- Consumes: Task 1~4의 통합 결과.
- Produces: 카메라+LiDAR 단일 bag 동시 녹화가 실제로 동작함을 확인(수용 기준).

> **이 태스크는 자동화할 수 없으며 사용자가 LiDAR·카메라 4대 연결 상태에서 직접 수행한다.** `docs/LIDAR.md` 6절 체크리스트와 동일. 어느 단계에서든 기대 결과가 안 나오면 `docs/LIDAR.md` 7절 트러블슈팅 → 필요 시 systematic-debugging.

- [ ] **Step 1: 네트워크 사전 점검**

```bash
ip -4 addr show | grep 192.168.1     # 192.168.1.2/24 표시되어야 함
ping -c 3 192.168.1.62               # 0% packet loss
```

- [ ] **Step 2: 통합 런치 기동**

```bash
cd ~/Desktop/econ_camera_ws
source install/setup.bash
ros2 launch econ_camera_ros record_all.launch.py
```
Expected: 카메라·LiDAR 노드·recorder 기동, `No cameras`/`initialize_type` 에러 없음, `rosbag2_<ts>/` 생성.

- [ ] **Step 3: 토픽 발행 확인 (별도 터미널)**

```bash
cd ~/Desktop/econ_camera_ws && source install/setup.bash
ros2 topic hz /camera0/image_raw/compressed   # 0~3 각각 ≈ 30 Hz
ros2 topic hz /unilidar/cloud                 # ≈ 5 Hz
ros2 topic echo /unilidar/imu --once          # Imu 1건 출력
ros2 topic list | grep tf                     # /tf, /tf_static
```

- [ ] **Step 4: bag 내용 확인**

런치 터미널에서 `Ctrl-C` 종료 후:
```bash
ros2 bag info rosbag2_<ts>
```
Expected: 8개 토픽(`/camera0..3/image_raw/compressed`, `/unilidar/cloud`, `/unilidar/imu`, `/tf`, `/tf_static`) 모두 존재; 카메라4·`/unilidar/cloud`·`/unilidar/imu`·`/tf` 7개는 각 message count > 0; `/tf_static`은 현재 발행 노드가 없어 0건이거나 목록에 없을 수 있음(정상 — extrinsic은 `/tf`에 실림); duration 정상.

- [ ] **Step 5: 검증 결과 기록 (선택)**

문제가 있었다면 원인/조치를 `docs/LIDAR.md` 7절에 추가하고 커밋. 문제 없으면 브랜치 완료 → `superpowers:finishing-a-development-branch`로 병합/PR 옵션 진행(병합·푸시는 사용자).

---

## Self-Review

**Spec coverage (스펙 2026-07-20-lidar-integration-design.md 대비):**
- §2 벤더링(선택 복사)·§3 디렉터리 구조 → Task 1 ✓
- §4 CMake 경로 수정 → Task 2 ✓
- §5 통합 런치·§5 8토픽 → Task 3 ✓
- §6 타임스탬프 정합 → 코드 변경 불필요(양 센서 시스템 시간). 문서 언급은 스펙에 존재, 구현 태스크 없음(설계 근거로 충분) ✓
- §7 빌드·검증 → Task 2(빌드·회귀) + Task 5(하드웨어 체크리스트) ✓
- §8 문서(LIDAR.md/USAGE/CLAUDE) → Task 4 ✓
- §9 Git 브랜치 → Global Constraints + 각 태스크 커밋 ✓
- §10 범위 밖 → 태스크 없음(의도된 제외) ✓

**Placeholder scan:** TBD/TODO 없음. 모든 코드·명령·기대결과 명시. 문서 파일은 전체 내용 수록.

**Type/이름 consistency:** 토픽명(`/unilidar/cloud`, `/unilidar/imu`), 실행자(`capture`), 벤더 launch 파일명(`launch_no_rviz.py`), 설치 경로(`share/unitree_lidar_ros2/launch_no_rviz.py`), CMake 경로(`../../third_party/...`)가 태스크 전반 일치.
