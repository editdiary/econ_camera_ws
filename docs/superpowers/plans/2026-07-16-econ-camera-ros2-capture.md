# e-con 4-Camera ROS2 연속 수집 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** e-con AR0234 4대를 하드웨어 동기(`frame_sync=1`)로 연속 캡처해 카메라별 `sensor_msgs/CompressedImage`(HW JPEG)로 발행하고, 런치 하나로 ROS2 bag(mcap)에 계속 기록한다.

**Architecture:** 단일 공유클럭 GStreamer 파이프라인(4×`v4l2src`→`nvvidconv`→`nvjpegenc`→`appsink`)을 소유하는 ROS2 Python 노드(`capture_node`)가 각 appsink의 `new-sample` 콜백에서 즉시 발행한다. 런치 파일이 이 노드와 `ros2 bag record`(mcap)를 함께 기동한다. 온디맨드 `monitor_node`가 토픽을 구독해 2×2로 표시한다(별도 프로세스·격리).

**Tech Stack:** ROS2 Humble (ament_python), Python `gi`(GStreamer), Jetson HW 플러그인(`nvvidconv`/`nvjpegenc`), rosbag2 + mcap, cv2(모니터 전용). 순수 로직 재사용: `econ_cam.controls`, `econ_cam.stats`.

## Global Constraints

- ROS2 **Humble**, 패키지 빌드 타입 **ament_python**. (JetPack 6.1 / L4T R36.4 / Ubuntu 22.04)
- 수집 설정: **1280×720 @ 30Hz**, `frame_sync=1`. `v4l2src` 순수 V4L2 경로만 사용(**Argus 사용 불가**).
- 메시지: `sensor_msgs/CompressedImage`, `format="jpeg"`, `header.frame_id="camera{N}"`, `header.stamp`=캡처 PTS를 ROS 시각에 앵커링.
- 토픽: `/camera0/image_raw/compressed` ~ `/camera3/image_raw/compressed`.
- QoS: **발행/recorder = reliable, keep-last(depth 10)**. **monitor = best-effort, keep-last(depth 1)**.
- 저장: rosbag2 스토리지 **mcap**.
- 재사용: `econ_cam.controls`/`econ_cam.stats`는 **import**(복붙 금지). `pip install -e ../Multi-Cam_module_test`.
- 범위 밖: `camera_info` 발행, LiDAR 패키지.
- Git: 구현은 **새 브랜치**에서 진행, 커밋까지만(merge/push는 사용자). 선행 repo 변경도 그 repo의 새 브랜치에서.
- 절대 경로 기준: 신규 프로젝트 `~/Desktop/econ_camera_ws`, 선행 프로젝트 `~/Desktop/Multi-Cam_module_test`.

## 구현 참고 노트 (선행 프로젝트에서 검증된 사항)

나중 세션이 실수로 되돌리지 않도록, `../Multi-Cam_module_test`에서 검증된 함정을 기록한다.

- **`nvvidconv`는 `nvjpegenc` 앞에 필수** — 스케일링이 없어도 제거 금지. `nvjpegenc`는 NVMM
  입력을 요구하며, `v4l2src`(시스템 메모리 UYVY) → `nvvidconv`(NVMM 변환) → `nvjpegenc` 순서라야
  동작한다. 선행 `econ_cam/gst_pipeline.py`의 풀해상도 캡처 갈래도 동일하게 nvvidconv를 거친다.
- **`frame_sync`는 파이프라인 PLAYING 전에 설정** — `capture_node`가 스트리밍 시작 전에
  `controls.set_frame_sync`를 호출한다(Task 3 반영). 스트리밍 중 변경은 반영 안 될 수 있다.
- **연속 4×풀해상도 JPEG는 새로운 부하 프로파일** — 선행 프로젝트는 풀해상도 JPEG를 *촬영 순간에만*
  (valve gated) 인코딩했고, 상시 인코딩은 저해상도 프리뷰뿐이었다. 4대 상시 풀해상도 `nvjpegenc`는
  검증된 적이 없으므로 Task 3·최종 검증에서 `tegrastats`로 CPU/전력/드롭을 반드시 확인하고, 동시
  `nvjpegenc` 인스턴스 한계를 주시한다.
- **`cv2`는 `apt python3-opencv`로 설치** — 시스템 numpy 1.21.5와 호환. pip `opencv-python`은 numpy
  충돌 위험(선행 프로젝트가 겪어 venv를 분리했던 문제). 모니터에만 필요(핵심 촬영/녹화는 cv2 불필요).
- **재사용 모듈은 이미 테스트됨** — 선행 `tests/test_controls.py`·`test_stats.py`가 `controls`/`stats`를
  커버하므로 새 프로젝트에선 재테스트 불필요. 신규 단위 테스트는 `gst_builder`만.
- **(선택) 장치 인덱스 런타임 확인** — `controls.detect_cameras()`로 `ar0234` 카드를 감지할 수 있다.
  하드코딩 `[0,1,2,3]` 대신 런타임 감지를 쓰면 장치 순서 변동에 견고해진다.

## File Structure

```
econ_camera_ws/
  src/econ_camera_ros/
    package.xml                    # ament_python 메타 + exec_depend
    setup.py  setup.cfg           # 엔트리포인트(capture, monitor) + 데이터파일(launch)
    resource/econ_camera_ros      # ament 리소스 마커(빈 파일)
    econ_camera_ros/
      __init__.py                 # 빈 파일
      gst_builder.py              # 연속 수집 파이프라인 문자열 (순수 함수) — 단위 테스트 대상
      capture_node.py             # 파이프라인 소유 + CompressedImage 발행 + 동기 품질 주기 로그
      monitor_node.py             # 온디맨드 2×2 시각화 (구독 전용, cv2)
    launch/record.launch.py       # capture 노드 + ros2 bag record(mcap) 동시 기동
    test/test_gst_builder.py      # gst_builder 순수 로직 pytest
```

**선행 repo에 추가(재사용용, 별도 브랜치·커밋):** `Multi-Cam_module_test/pyproject.toml`.

---

## Task 0: 환경 준비 (Prerequisites Gate)

ROS2 미설치 상태에서 시작한다. 이 태스크는 TDD가 아니라 **검증 게이트**다. `sudo`가 필요한 단계는 **사용자가 실행**한다(세션에서 `! <명령>` 으로 실행하거나 직접). 각 단계 끝의 검증이 통과해야 다음 태스크로 넘어간다.

**Files:** 없음(설치/검증). 단, econ_cam 재사용을 위해 선행 repo에 `pyproject.toml` 추가.

- [ ] **Step 1: ROS2 Humble 설치 (사용자 실행, sudo)**

이미 설치돼 있으면 건너뛴다(`ls /opt/ros/humble` 로 확인). 없으면 공식 절차대로 설치:

```bash
# locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ROS2 apt 소스
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# ROS2 (GUI 모니터를 쓸 것이므로 desktop, colcon)
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
```

- [ ] **Step 2: mcap 스토리지 + 검증**

```bash
sudo apt install -y ros-humble-rosbag2-storage-mcap
```

Verify:
```bash
source /opt/ros/humble/setup.bash
ros2 --version                          # 버전 출력
ros2 bag record --help | grep -i mcap   # -s mcap 관련 언급 확인
```
Expected: `ros2` 명령 동작, mcap 스토리지 인식.

- [ ] **Step 3: 선행 repo에 pyproject.toml 추가(재사용용, 선행 repo 새 브랜치)**

`econ_cam.controls`/`econ_cam.stats`를 import 가능하게 최소 패키징을 추가한다. **선행 repo에서 새 브랜치**로 작업한다.

```bash
cd ~/Desktop/Multi-Cam_module_test
git checkout -b feat/pip-installable
```

Create `~/Desktop/Multi-Cam_module_test/pyproject.toml`:
```toml
[build-system]
requires = ["setuptools>=61"]
build-backend = "setuptools.build_meta"

[project]
name = "econ_cam"
version = "0.1.0"
description = "e-con multicam pure logic (controls, stats) — reused by econ_camera_ros"
requires-python = ">=3.10"

[tool.setuptools]
packages = ["econ_cam"]
```

Commit:
```bash
cd ~/Desktop/Multi-Cam_module_test
git add pyproject.toml
git commit -m "chore: add minimal pyproject for econ_cam reuse

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

- [ ] **Step 4: econ_cam editable 설치 (system python3 = ROS2 런타임)**

ROS2 노드는 system python3(3.10)로 실행된다. 사용자 사이트에 editable 설치하면 `ros2 run`에서 import된다.

```bash
python3 -m pip install --user -e ~/Desktop/Multi-Cam_module_test
```

Verify:
```bash
python3 -c "from econ_cam import controls, stats; print('econ_cam OK', controls.set_frame_sync.__name__, stats.timestamp_stats.__name__)"
```
Expected: `econ_cam OK set_frame_sync timestamp_stats` (numpy/opencv/flask 없이 import됨 — 두 모듈은 표준 라이브러리만 사용).

- [ ] **Step 5: GStreamer / gi / 카메라 / HW 인코더 검증**

```bash
python3 -c "import gi; gi.require_version('Gst','1.0'); from gi.repository import Gst; Gst.init(None); print('gi OK', Gst.version_string())"
gst-inspect-1.0 nvjpegenc | head -2      # HW JPEG 인코더 존재
gst-inspect-1.0 nvvidconv | head -2      # HW 변환기 존재
python3 -c "from econ_cam import controls; print([c['dev'] for c in controls.detect_cameras()])"
```
Expected: `gi OK ...`, nvjpegenc/nvvidconv 팩토리 출력, 카메라 dev 리스트 `[0, 1, 2, 3]`.

- [ ] **Step 6: (선택) 모니터용 cv2 설치**

모니터를 쓸 경우에만 필요(핵심 촬영/녹화는 cv2 불필요). system numpy와 호환되는 apt 버전 권장:
```bash
sudo apt install -y python3-opencv
python3 -c "import cv2, numpy; print('cv2', cv2.__version__, 'numpy', numpy.__version__)"
```
Expected: cv2/numpy 버전 출력. (GUI `imshow` 가능한 apt 빌드.)

---

## Task 1: ament_python 패키지 스캐폴드

**Files:**
- Create: `src/econ_camera_ros/package.xml`
- Create: `src/econ_camera_ros/setup.py`
- Create: `src/econ_camera_ros/setup.cfg`
- Create: `src/econ_camera_ros/resource/econ_camera_ros`
- Create: `src/econ_camera_ros/econ_camera_ros/__init__.py`

**Interfaces:**
- Produces: 빌드 가능한 빈 ament_python 패키지 `econ_camera_ros`. 엔트리포인트 `capture`, `monitor`는 이후 태스크의 `main`을 가리킨다(아직 모듈 없음 → 빌드는 되고 실행만 후속 태스크에서).

- [ ] **Step 1: 구현 브랜치 생성 (신규 repo)**

```bash
cd ~/Desktop/econ_camera_ws
git checkout -b feat/ros2-capture
```

- [ ] **Step 2: 디렉터리 생성**

```bash
mkdir -p ~/Desktop/econ_camera_ws/src/econ_camera_ros/econ_camera_ros \
         ~/Desktop/econ_camera_ws/src/econ_camera_ros/resource \
         ~/Desktop/econ_camera_ws/src/econ_camera_ros/launch \
         ~/Desktop/econ_camera_ws/src/econ_camera_ros/test
```

- [ ] **Step 3: `resource/econ_camera_ros` (빈 마커) + `econ_camera_ros/__init__.py` (빈 파일)**

두 파일 모두 빈 내용으로 생성:
```bash
: > ~/Desktop/econ_camera_ws/src/econ_camera_ros/resource/econ_camera_ros
: > ~/Desktop/econ_camera_ws/src/econ_camera_ros/econ_camera_ros/__init__.py
```

- [ ] **Step 4: `package.xml` 작성**

`src/econ_camera_ros/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>econ_camera_ros</name>
  <version>0.1.0</version>
  <description>e-con AR0234 4-camera 연속 동기 수집 ROS2 노드</description>
  <maintainer email="dhlee@sju.ac.kr">DoHyeon_Lee</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>rosbag2_storage_mcap</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 5: `setup.py` 작성**

`src/econ_camera_ros/setup.py`:
```python
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
            'monitor = econ_camera_ros.monitor_node:main',
        ],
    },
)
```

- [ ] **Step 6: `setup.cfg` 작성**

`src/econ_camera_ros/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/econ_camera_ros
[install]
install_scripts=$base/lib/econ_camera_ros
```

- [ ] **Step 7: 빌드 검증**

Run:
```bash
cd ~/Desktop/econ_camera_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select econ_camera_ros
```
Expected: `Finished <<< econ_camera_ros` (엔트리포인트 모듈은 아직 없지만 빌드/설치는 성공). 참고: 아직 `ros2 run`은 모듈 부재로 실패하는 게 정상.

- [ ] **Step 8: 커밋**

```bash
cd ~/Desktop/econ_camera_ws
git add src/econ_camera_ros/package.xml src/econ_camera_ros/setup.py src/econ_camera_ros/setup.cfg \
        src/econ_camera_ros/resource src/econ_camera_ros/econ_camera_ros/__init__.py
git commit -m "feat: econ_camera_ros ament_python 패키지 스캐폴드

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: gst_builder.py — 연속 수집 파이프라인 문자열 (TDD, 순수)

**Files:**
- Create: `src/econ_camera_ros/econ_camera_ros/gst_builder.py`
- Test: `src/econ_camera_ros/test/test_gst_builder.py`

**Interfaces:**
- Produces: `capture_pipeline(devs, width, height, jpeg_quality=90, max_buffers=5, sink_prefix="sink") -> str`
  - `devs`: `list[int]`. 각 dev마다 `v4l2src device=/dev/video{dev}` 브랜치를 만들고, 브랜치들을 공백으로 이어 하나의 파이프라인 문자열로 반환(단일 공유 클럭).
  - appsink 이름: `f"{sink_prefix}{dev}"`. capture_node가 이 이름으로 `get_by_name` 한다.
  - caps: `video/x-raw,format=UYVY,width={width},height={height}`. 인코더: `nvjpegenc quality={jpeg_quality}`. appsink 속성: `max-buffers={max_buffers} drop=false sync=false`.

- [ ] **Step 1: 실패하는 테스트 작성**

`src/econ_camera_ros/test/test_gst_builder.py`:
```python
from econ_camera_ros import gst_builder


def test_one_branch_per_device():
    desc = gst_builder.capture_pipeline([0, 1, 2, 3], 1280, 720)
    assert desc.count("v4l2src") == 4
    for d in (0, 1, 2, 3):
        assert f"device=/dev/video{d}" in desc
        assert f"appsink name=sink{d}" in desc


def test_caps_and_encoder():
    desc = gst_builder.capture_pipeline([0], 1280, 720, jpeg_quality=90)
    assert "format=UYVY,width=1280,height=720" in desc
    assert "nvvidconv" in desc
    assert "nvjpegenc quality=90" in desc
    assert "image/jpeg" in desc


def test_appsink_no_drop_for_continuous_capture():
    desc = gst_builder.capture_pipeline([0], 1280, 720, max_buffers=5)
    assert "max-buffers=5" in desc
    assert "drop=false" in desc
    assert "sync=false" in desc


def test_sink_prefix_override():
    desc = gst_builder.capture_pipeline([2], 640, 480, sink_prefix="cap")
    assert "appsink name=cap2" in desc
```

- [ ] **Step 2: 테스트 실패 확인**

Run:
```bash
cd ~/Desktop/econ_camera_ws/src/econ_camera_ros
python3 -m pytest test/test_gst_builder.py -v
```
Expected: FAIL — `ModuleNotFoundError` 또는 `AttributeError: module 'econ_camera_ros.gst_builder' has no attribute 'capture_pipeline'`.

- [ ] **Step 3: 최소 구현**

`src/econ_camera_ros/econ_camera_ros/gst_builder.py`:
```python
"""연속 수집용 GStreamer 파이프라인 문자열 빌더 (순수 함수).

선행 프로젝트(econ_cam.gst_pipeline)의 tee/valve 게이팅 구조와 달리, 연속 수집은
tee·valve 없이 원본을 계속 JPEG로 인코딩해 흘려보낸다. 4개 v4l2src 브랜치를 하나의
문자열로 이어 단일 파이프라인(공유 클럭)으로 실행하면 카메라 간 PTS 비교가 유효하다.

실기 검증된 경로: v4l2src ! video/x-raw,format=UYVY ! nvvidconv ! nvjpegenc ! image/jpeg ! appsink
"""


def _branch(dev, width, height, jpeg_quality, max_buffers, sink_prefix):
    return (
        f"v4l2src device=/dev/video{dev} "
        f"! video/x-raw,format=UYVY,width={width},height={height} "
        f"! nvvidconv ! nvjpegenc quality={jpeg_quality} ! image/jpeg "
        f"! appsink name={sink_prefix}{dev} max-buffers={max_buffers} drop=false sync=false"
    )


def capture_pipeline(devs, width, height, jpeg_quality=90, max_buffers=5,
                     sink_prefix="sink"):
    """N개 v4l2src 연속 캡처 브랜치를 단일 공유클럭 파이프라인 문자열로 반환.

    appsink 이름은 f"{sink_prefix}{dev}". drop=false 로 무단 드롭을 피한다(소비자가
    new-sample 콜백에서 즉시 pull).
    """
    return "   ".join(
        _branch(d, width, height, jpeg_quality, max_buffers, sink_prefix)
        for d in devs
    )
```

- [ ] **Step 4: 테스트 통과 확인**

Run:
```bash
cd ~/Desktop/econ_camera_ws/src/econ_camera_ros
python3 -m pytest test/test_gst_builder.py -v
```
Expected: PASS (4 passed).

- [ ] **Step 5: 커밋**

```bash
cd ~/Desktop/econ_camera_ws
git add src/econ_camera_ros/econ_camera_ros/gst_builder.py src/econ_camera_ros/test/test_gst_builder.py
git commit -m "feat: 연속 수집 파이프라인 문자열 빌더(gst_builder) + 테스트

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: capture_node.py — 발행 노드 + 동기 품질 주기 로그

**Files:**
- Create: `src/econ_camera_ros/econ_camera_ros/capture_node.py`

**Interfaces:**
- Consumes: `gst_builder.capture_pipeline(...)`; `econ_cam.controls.set_frame_sync(dev, mode) -> bool`; `econ_cam.stats.match_frames({dev: [(pts, payload), ...]}) -> {dev: (pts, payload)}`; `econ_cam.stats.timestamp_stats({dev: pts_seconds}) -> {"spread_ms","std_ms","per_camera","ref_dev"}`.
- Produces: 실행형 `main()`. 토픽 `/camera{dev}/image_raw/compressed`(`sensor_msgs/CompressedImage`) 발행. ROS 파라미터: `devices`(int list, 기본 [0,1,2,3]), `width`(1280), `height`(720), `sync_mode`(1), `jpeg_quality`(90), `log_period_s`(5.0).

- [ ] **Step 1: 노드 구현 작성**

`src/econ_camera_ros/econ_camera_ros/capture_node.py`:
```python
"""e-con 4-camera 연속 동기 수집 ROS2 노드.

단일 공유클럭 GStreamer 파이프라인을 소유하고, 각 카메라 appsink의 new-sample 콜백에서
JPEG 버퍼를 꺼내 sensor_msgs/CompressedImage 로 즉시 발행한다. 버퍼 PTS(파이프라인 공유
클럭)를 ROS 시각에 앵커링해 header.stamp 로 쓰므로 카메라 간 stamp 가 직접 비교 가능하다.
주기적으로 4대 타임스탬프 편차(spread/std)와 카메라별 수신 프레임 수를 로그한다.
"""

import threading

import gi
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

from econ_cam import controls, stats  # noqa: E402
from econ_camera_ros import gst_builder  # noqa: E402

Gst.init(None)

_RING = 10  # 동기 로그용 카메라별 최근 PTS 개수


class CaptureNode(Node):
    def __init__(self):
        super().__init__("econ_camera_capture")
        self.declare_parameter("devices", [0, 1, 2, 3])
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("sync_mode", 1)
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("log_period_s", 5.0)

        self.devs = list(self.get_parameter("devices").value)
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.sync_mode = self.get_parameter("sync_mode").value
        quality = self.get_parameter("jpeg_quality").value
        log_period = self.get_parameter("log_period_s").value

        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._pubs = {
            d: self.create_publisher(
                CompressedImage, f"/camera{d}/image_raw/compressed", qos)
            for d in self.devs
        }

        self._lock = threading.Lock()
        self._pts_ring = {d: [] for d in self.devs}
        self._counts = {d: 0 for d in self.devs}
        self._t0_ros = None   # 첫 프레임 ROS 시각(ns)
        self._t0_pts = None   # 첫 프레임 PTS(ns)

        for d in self.devs:
            if not controls.set_frame_sync(d, self.sync_mode):
                self.get_logger().warn(f"frame_sync 설정 실패: /dev/video{d}")

        desc = gst_builder.capture_pipeline(
            self.devs, self.width, self.height, jpeg_quality=quality)
        self.get_logger().info(f"pipeline: {desc}")
        self._pipeline = Gst.parse_launch(desc)
        for d in self.devs:
            sink = self._pipeline.get_by_name(f"sink{d}")
            sink.set_property("emit-signals", True)
            sink.connect("new-sample", self._on_sample, d)
        self._pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(
            f"촬영 시작: devices={self.devs} @ {self.width}x{self.height}, "
            f"sync_mode={self.sync_mode}, jpeg_quality={quality}")

        self._timer = self.create_timer(log_period, self._log_sync)

    def _on_sample(self, sink, dev):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        data = buf.extract_dup(0, buf.get_size())
        pts_ns = buf.pts

        with self._lock:
            if pts_ns == Gst.CLOCK_TIME_NONE:
                stamp_ns = self.get_clock().now().nanoseconds
            else:
                if self._t0_ros is None:
                    self._t0_ros = self.get_clock().now().nanoseconds
                    self._t0_pts = pts_ns
                stamp_ns = self._t0_ros + (pts_ns - self._t0_pts)
                ring = self._pts_ring[dev]
                ring.append(pts_ns / 1e9)
                del ring[:-_RING]
            self._counts[dev] += 1

        msg = CompressedImage()
        msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        msg.header.frame_id = f"camera{dev}"
        msg.format = "jpeg"
        msg.data = data
        self._pubs[dev].publish(msg)
        return Gst.FlowReturn.OK

    def _log_sync(self):
        with self._lock:
            rings = {d: list(r) for d, r in self._pts_ring.items()}
            counts = dict(self._counts)
        frames = {d: [(p, None) for p in r] for d, r in rings.items() if r}
        if len(frames) < len(self.devs):
            self.get_logger().info(f"수신 대기중... frames={counts}")
            return
        chosen = stats.match_frames(frames)
        if not chosen:
            return
        s = stats.timestamp_stats({d: c[0] for d, c in chosen.items()})
        self.get_logger().info(
            f"동기 spread={s['spread_ms']:.2f}ms std={s['std_ms']:.2f}ms | "
            f"frames={counts}")

    def destroy_node(self):
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 빌드 후 실행 (하드웨어 검증)**

Run:
```bash
cd ~/Desktop/econ_camera_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select econ_camera_ros
source install/setup.bash
ros2 run econ_camera_ros capture
```
Expected(로그): `촬영 시작: devices=[0, 1, 2, 3] ...` 후, 몇 초 내 `동기 spread=... std=... | frames={0: N, 1: N, 2: N, 3: N}` 가 주기적으로 출력. spread/std는 작아야 함(frame_sync=1). Ctrl-C로 정상 종료.

- [ ] **Step 3: 토픽 발행/주파수 검증 (다른 터미널)**

Run(노드 실행 중 별도 터미널):
```bash
source /opt/ros/humble/setup.bash && source ~/Desktop/econ_camera_ws/install/setup.bash
ros2 topic list | grep image_raw/compressed          # 4개 토픽
ros2 topic hz /camera0/image_raw/compressed          # ~30Hz
ros2 topic bw /camera0/image_raw/compressed          # 수백 KB/s대(JPEG)
```
Expected: 토픽 4개, `average rate: 30.0` 근처, 대역폭이 raw 대비 작음.

- [ ] **Step 4: 커밋**

```bash
cd ~/Desktop/econ_camera_ws
git add src/econ_camera_ros/econ_camera_ros/capture_node.py
git commit -m "feat: capture_node — CompressedImage 발행 + 동기 품질 주기 로그

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: record.launch.py — 노드 + bag record(mcap) 동시 기동

**Files:**
- Create: `src/econ_camera_ros/launch/record.launch.py`

**Interfaces:**
- Consumes: `econ_camera_ros` 패키지의 `capture` 엔트리포인트, 토픽 `/camera{0..3}/image_raw/compressed`.
- Produces: `ros2 launch econ_camera_ros record.launch.py` 실행 시 capture 노드 + `ros2 bag record -s mcap <4토픽>` 동시 기동. bag은 실행 디렉터리에 `rosbag2_YYYY_MM_DD-HH_MM_SS/` 로 자동 생성.

- [ ] **Step 1: 런치 파일 작성**

`src/econ_camera_ros/launch/record.launch.py`:
```python
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
```

- [ ] **Step 2: 빌드 + 실행 검증 (하드웨어)**

Run:
```bash
cd ~/Desktop/econ_camera_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select econ_camera_ros
source install/setup.bash
cd ~/Desktop/econ_camera_ws          # bag 생성 위치
ros2 launch econ_camera_ros record.launch.py
```
Expected: capture 로그(촬영 시작/동기 로그)와 recorder 로그가 함께 출력, `rosbag2_...` 디렉터리 생성. 수 초 후 Ctrl-C.

- [ ] **Step 3: bag 내용 검증**

Run:
```bash
ros2 bag info ~/Desktop/econ_camera_ws/rosbag2_*   # 가장 최근 디렉터리
```
Expected: Storage id: `mcap`, Topics 4개(`/camera0..3/image_raw/compressed`, type `sensor_msgs/msg/CompressedImage`), 각 토픽 Count가 촬영시간×30 근처(드롭 미미).

- [ ] **Step 4: 커밋**

```bash
cd ~/Desktop/econ_camera_ws
git add src/econ_camera_ros/launch/record.launch.py
git commit -m "feat: record.launch.py — capture 노드 + bag record(mcap) 동시 기동

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: monitor_node.py — 온디맨드 2×2 시각화

**Files:**
- Create: `src/econ_camera_ros/econ_camera_ros/monitor_node.py`

**Interfaces:**
- Consumes: 토픽 `/camera{dev}/image_raw/compressed`(`sensor_msgs/CompressedImage`, JPEG). `cv2`, `numpy`.
- Produces: 실행형 `main()`. 구독 전용(best-effort), ~10fps로 2×2 그리드 표시. ROS 파라미터: `devices`([0,1,2,3]), `cell_width`(480), `cell_height`(270).

- [ ] **Step 1: 모니터 구현 작성**

`src/econ_camera_ros/econ_camera_ros/monitor_node.py`:
```python
"""온디맨드 실시간 모니터 (구독 전용, 녹화와 격리).

발행 중인 CompressedImage 토픽만 구독해 4대를 2×2 그리드로 표시한다. best-effort QoS라
모니터가 느려져도 발행/녹화에 backpressure를 주지 않는다. 안 켜면 비용 0.
실행: ros2 run econ_camera_ros monitor  (q 키로 종료)
"""

import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class MonitorNode(Node):
    def __init__(self):
        super().__init__("econ_camera_monitor")
        self.declare_parameter("devices", [0, 1, 2, 3])
        self.declare_parameter("cell_width", 480)
        self.declare_parameter("cell_height", 270)
        self.devs = list(self.get_parameter("devices").value)
        self.cw = self.get_parameter("cell_width").value
        self.ch = self.get_parameter("cell_height").value

        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self._frames = {}
        self._lock = threading.Lock()
        for d in self.devs:
            self.create_subscription(
                CompressedImage, f"/camera{d}/image_raw/compressed",
                lambda msg, dev=d: self._on_image(msg, dev), qos)
        self._timer = self.create_timer(0.1, self._render)  # ~10fps

    def _on_image(self, msg, dev):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return
        img = cv2.resize(img, (self.cw, self.ch))
        with self._lock:
            self._frames[dev] = img

    def _render(self):
        with self._lock:
            frames = dict(self._frames)
        cells = []
        for d in self.devs:
            cell = frames.get(d)
            if cell is None:
                cell = np.zeros((self.ch, self.cw, 3), dtype=np.uint8)
                cv2.putText(cell, f"cam{d} (no signal)", (10, self.ch // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                cv2.putText(cell, f"cam{d}", (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cells.append(cell)
        while len(cells) < 4:
            cells.append(np.zeros((self.ch, self.cw, 3), dtype=np.uint8))
        grid = np.vstack([np.hstack(cells[0:2]), np.hstack(cells[2:4])])
        cv2.imshow("econ camera monitor (press q to quit)", grid)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 빌드 + 실행 검증 (촬영 중 별도 터미널, 하드웨어)**

터미널 A에서 촬영 실행(`ros2 launch econ_camera_ros record.launch.py`), 터미널 B에서:
```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/econ_camera_ws && colcon build --packages-select econ_camera_ros && source install/setup.bash
ros2 run econ_camera_ros monitor
```
Expected: 2×2 그리드 창에 4대 실시간 영상. `q`로 종료. **모니터를 켜고 끄는 동안에도 터미널 A의 bag Count가 정상 증가**(격리 확인).

- [ ] **Step 3: 커밋**

```bash
cd ~/Desktop/econ_camera_ws
git add src/econ_camera_ros/econ_camera_ros/monitor_node.py
git commit -m "feat: monitor_node — 온디맨드 2x2 시각화(구독 전용, 녹화와 격리)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## 최종 검증 (전체 통합, 하드웨어)

- [ ] `ros2 launch econ_camera_ros record.launch.py` → 4개 토픽 ~30Hz 발행, 동기 spread/std 로그 정상.
- [ ] `ros2 bag info` → mcap에 4개 토픽 기록, Count ≈ 시간×30(드롭 미미).
- [ ] 촬영 중 `ros2 run econ_camera_ros monitor` → 2×2 표시, 모니터 on/off와 무관하게 bag 안정.
- [ ] 촬영 중 `tegrastats` → CPU/전력 여유 범위.
- [ ] `python3 -m pytest src/econ_camera_ros/test/ -v` → gst_builder 테스트 통과.

완료 후 `superpowers:finishing-a-development-branch`로 마무리(브랜치 병합/푸시는 사용자).
