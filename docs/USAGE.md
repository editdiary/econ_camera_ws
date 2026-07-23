# econ_camera_ws 사용 가이드

e-con AR0234 4-camera 모듈을 ROS2로 **연속 동기 수집**하고, 기록한 bag을 다시 꺼내
쓰는 전 과정을 처음 보는 사람도 바로 따라 할 수 있게 정리한 문서다.

- 개요·설계 배경: [`../README.md`](../README.md), 설계 스펙: [`superpowers/specs/`](superpowers/specs/)
- 이 문서는 **어떻게 쓰는가(how-to)**에 집중한다. *왜 이렇게 설계했는가*는 스펙을 참조.

---

## 0. 한눈에 보기

이 워크스페이스는 노드 2개 + 도구 2개 + 런치 3개로 구성된다(패키지 `econ_camera_ros`).

| 실행 이름 | 파일 | 역할 |
|---|---|---|
| `capture` | `capture_node.py` | 카메라 4대를 단일 GStreamer 파이프라인으로 캡처 → JPEG 발행 |
| `monitor` | `web_monitor_node.py` | 발행 토픽을 구독해 브라우저에서 2×2 실시간 확인(**구독 전용**) |
| `bag_extract` | `bag_extract.py` | 녹화된 bag(mcap)을 동기 세트별 JPEG로 추출 |
| `check_recording` | `tools/check_recording.py` | 녹화된 bag의 프레임 간격으로 FPS·끊김·4대 정렬 판정(§5) |
| `record.launch.py` | `launch/` | `capture` + `ros2 bag record`(카메라 4토픽) 동시 기동 |
| `record_all.launch.py` | `launch/` | `capture` + LiDAR + record(카메라4+LiDAR+TF, 8토픽) 동시 기동(§9) |
| `record_lidar.launch.py` | `launch/` | 카메라 없이 LiDAR만 record(`/unilidar/*`+`/tf`, 4토픽)(§9) |

전형적인 흐름:

```
[촬영]  ros2 launch econ_camera_ros record.launch.py     →  rosbag2_<timestamp>/ (mcap)
[확인]  ros2 run econ_camera_ros monitor                 →  http://<Orin-IP>:10010
[재생]  ros2 bag play rosbag2_<timestamp>                →  녹화 토픽을 그대로 재발행 (+monitor로 눈 확인)
[추출]  ros2 run econ_camera_ros bag_extract <bag> -o out →  out/frame_*/cam{0..3}.jpg
```

---

## 1. 요구 환경

- **Jetson AGX Orin**, JetPack 6.1 / L4T R36.4 (Ubuntu 22.04)
- **ROS2 Humble** (호스트 네이티브 설치. Docker 미사용), `colcon`
- `rosbag2` + **`rosbag2_storage_mcap`** (mcap 스토리지 플러그인)
- 시스템 GStreamer + Python `gi` + Jetson HW 플러그인(`nvvidconv`, `nvjpegenc`)
- 카메라 4대 `/dev/video0`~`/dev/video3` (e-con AR0234, `tegra-video` CSI, **UYVY** 직출력)
- 선행 프로젝트 `../Multi-Cam_module_test` — 순수 로직 `econ_cam`(`controls`, `stats`) 재사용

> **왜 Argus가 아니라 V4L2인가**: 모듈이 UYVY를 직출력(내부 디베이어링)해 Jetson ISP를
> 경유하지 않으므로 libargus/`nvarguscamerasrc`가 카메라를 인식하지 못한다("No cameras
> available"). 캡처는 `v4l2src` 순수 V4L2 경로로만 가능하다.

### 사전 점검 (촬영 전 1회)

```bash
# 카메라 4대가 UYVY로 잡히는지
for n in 0 1 2 3; do echo "== /dev/video$n =="; v4l2-ctl -d /dev/video$n --list-formats-ext | grep -A2 UYVY; done

# frame_sync 컨트롤 존재 확인 (capture 노드가 자동으로 1로 설정하지만 수동 확인용)
v4l2-ctl -d /dev/video0 -L | grep frame_sync
```

`frame_sync` 값: `0`=Disable, `1`=30Hz, `2`=60Hz. 이 프로젝트는 기본 **1(30Hz)** 을 쓴다.

---

## 2. 설치 & 빌드

```bash
# 1) 선행 프로젝트의 순수 로직(econ_cam) 재사용 — 복붙 아님, import
pip install -e ../Multi-Cam_module_test

# 2) 워크스페이스 루트에서 빌드
cd ~/Desktop/econ_camera_ws
colcon build --packages-select econ_camera_ros
source install/setup.bash        # 새 터미널마다 필요
```

`source install/setup.bash`는 **터미널을 새로 열 때마다** 실행해야 `ros2 run/launch`가
패키지를 찾는다. 매번 치기 번거로우면 `~/.bashrc`에 추가해도 된다.

---

## 3. 촬영 (녹화)

### 3-1. 가장 간단한 방법 — 런치

```bash
ros2 launch econ_camera_ros record.launch.py
```

- `capture` 노드와 `ros2 bag record -s mcap`이 **함께** 뜬다.
- 실행한 **현재 디렉터리**에 `rosbag2_<timestamp>/`(mcap)가 생성된다.
- **Ctrl-C** 한 번이면 노드와 recorder가 함께 깔끔히 종료된다.

**시작 직후 로그 흐름** (정상):

```
[capture] pipeline: v4l2src device=/dev/video0 ... appsink name=sink0 ...
[capture] 파이프라인 시작: devices=[0,1,2,3] @ 1280x720, sync_mode=1, ... 워밍업 4.0s 후 발행 시작
[capture] 워밍업 중... 폐기 프레임={0: 30, 1: 30, 2: 29, 3: 30}
[capture] 워밍업 4.1s 완료(4대 정상 촬영 확인) — 발행 시작. 폐기 프레임={...}
[capture] 동기 spread=0.21ms std=0.09ms | frames={0: 150, 1: 150, 2: 150, 3: 150}
```

> **워밍업**: 시작하자마자 녹화하면 4대의 첫 프레임 도착 시점이 어긋나 카메라별 기록
> 프레임 수가 달라진다. 그래서 `capture`는 처음 `warmup_s`(기본 4초) 동안 프레임을
> **폐기**하며 4대 정상 동작을 확인하고, **같은 주기의 4대가 모두 모인 첫 사이클부터**
> 발행을 시작한다 → 4대가 동일한 프레임 수로 시작. 워밍업 중에는 발행이 없어 bag 시작도
> 깔끔하다.

### 3-2. 파라미터를 바꿔 촬영 — `ros2 run` + 수동 record

런치 파일은 **기본값으로만** 기동한다(파라미터를 노드에 전달하지 않음). 해상도·워밍업
시간 등을 바꾸려면 노드를 직접 실행하고, 녹화는 다른 터미널에서 건다.

터미널 A (캡처):
```bash
ros2 run econ_camera_ros capture --ros-args \
  -p warmup_s:=3.0 \
  -p jpeg_quality:=85 \
  -p log_period_s:=2.0
```

터미널 B (녹화 — 반드시 토픽을 **명시**. `-a` 전체기록 금지):
```bash
ros2 bag record -s mcap \
  /camera0/image_raw/compressed /camera1/image_raw/compressed \
  /camera2/image_raw/compressed /camera3/image_raw/compressed
```

#### `capture` 노드 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `devices` | `[0,1,2,3]` | 사용할 `/dev/videoN` 번호 목록 |
| `width` | `1280` | 캡처 폭 |
| `height` | `720` | 캡처 높이 |
| `sync_mode` | `1` | `frame_sync`: 0=off, 1=30Hz, 2=60Hz |
| `jpeg_quality` | `90` | HW JPEG 품질(0–100) |
| `log_period_s` | `5.0` | 동기/프레임수 로그 주기(초) |
| `warmup_s` | `4.0` | 발행 전 워밍업(프레임 폐기) 시간(초) |

> 파라미터를 바꾼 촬영을 자주 쓴다면 `record.launch.py`의 `Node(...)`에 `parameters=[{...}]`
> 를 추가하면 런치 하나로 끝낼 수 있다(현재는 기본값 고정).

---

## 4. 실시간 모니터링

캡처가 도는 상태에서 **다른 터미널**에 띄운다(구독만 하므로 녹화에 영향 0. 안 켜면 비용 0):

```bash
ros2 run econ_camera_ros monitor
# 접속: 브라우저에서  http://<Orin-IP>:10010
```

- 2×2 그리드로 4대 MJPEG 스트림 + 상단 상태바(카메라별 수신 fps, 동기 spread/std).
- 발행 페이로드가 이미 JPEG라 **디코드/재인코딩 없이** 그대로 흘려보낸다(가볍다).
- **헤드리스/SSH**: 화면이 없어도 브라우저로 확인 가능. SSH만 열려 있으면 포트 포워딩:
  ```bash
  ssh -L 10010:localhost:10010 <user>@<Orin-IP>
  # 이후 로컬 브라우저에서  http://localhost:10010
  ```

#### `monitor` 노드 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `devices` | `[0,1,2,3]` | 표시할 카메라 번호 |
| `port` | `10010` | HTTP 포트 |
| `host` | `0.0.0.0` | 바인드 주소 |
| `fps` | `15` | 브라우저로 밀어주는 표시 fps(수집 fps와 무관) |

포트가 이미 사용 중이면 트레이스백 대신 안내가 뜬다. 다른 포트로:
```bash
ros2 run econ_camera_ros monitor --ros-args -p port:=10011
```

---

## 5. 기록되는 토픽 & bag 다루기

### 발행 토픽 — 이미지 4개가 전부

| 토픽 | 타입 | QoS |
|---|---|---|
| `/camera0/image_raw/compressed` | `sensor_msgs/CompressedImage` (jpeg) | Reliable, KeepLast(10) |
| `/camera1/image_raw/compressed` | 〃 | 〃 |
| `/camera2/image_raw/compressed` | 〃 | 〃 |
| `/camera3/image_raw/compressed` | 〃 | 〃 |

- `header.frame_id` = `"camera{N}"`, `header.stamp` = **파이프라인 공유클럭 PTS를 첫 프레임
  ROS 시각(시스템 시간)에 앵커링**한 값 → 카메라 4대 간 stamp 직접 비교 가능(실측 sub-ms).
- rclpy 기본 토픽 `/rosout`, `/parameter_events`는 광고되지만 **record 대상이 아니라
  bag에 안 들어간다**(런치가 카메라 4토픽만 명시 기록).
- `monitor`는 **발행 0**(구독 전용) → bag에 아무 영향 없음.

### bag 디렉터리 구조

`rosbag2_<timestamp>/`는 파일 하나가 아니라 디렉터리다.

```
rosbag2_2026_07_18-15_30_00/
  metadata.yaml               # 토픽 목록·타입·메시지 수·QoS·기간 (사람이 읽는 요약)
  rosbag2_..._0.mcap          # 실제 메시지가 담긴 mcap 파일(용량 크면 _1, _2...로 분할)
```

- **bag을 옮길 땐 디렉터리째** 복사/이동해야 한다(`.mcap`만 떼면 재생 불가).
- `metadata.yaml`이 손상/분실되면 `ros2 bag reindex -s mcap rosbag2_<timestamp>`로 재생성.

### bag 내용 확인 (`ros2 bag info`)

```bash
ros2 bag info rosbag2_<timestamp>
```

토픽별 타입·**메시지 수**·기간·저장 크기가 나온다. 정상이면 4개 토픽의 메시지 수가 서로
같아야 한다(워밍업 덕분에 시작 프레임 수 일치). 크게 어긋나면 §7 문제해결을 본다.

### 녹화 품질 판정 (`check_recording.py`)

`ros2 bag info`는 메시지 수만 보여줄 뿐 **끊김·간격 불균일**은 못 잡는다. 리플레이/모니터
화면의 버벅임은 전송·표시 단계 문제라 녹화 품질과 무관하다. 녹화 성공 여부는 **저장된
프레임의 타임스탬프 간격**으로 판단해야 하므로, 이 도구가 bag을 직접 읽어 판정한다.

```bash
python3 tools/check_recording.py <bag_dir 또는 .mcap> [--fps 30] [--pattern camera]
# 의존성: pip install mcap  (ROS 소싱 불필요)
```

- 카메라별 `n·dur·fps`, 간격 mean/median/p95/max, **끊김(>기대주기×1.5) 횟수**를 출력.
- 판정 ❌ 조건: 카메라 간 프레임 수 차 3장 초과 / FPS < 기대치×0.95 / 끊김 검출.
- LiDAR 등 `camera` 미포함 토픽은 참고용 rate만 표시.

### bag 재생 (`ros2 bag play`)

녹화한 토픽을 **원래 타이밍 그대로 다시 발행**한다. 카메라 없이도 수집분을 그대로 재현할 수 있다.

```bash
ros2 bag play rosbag2_<timestamp>
```

재생 중 다른 터미널에서 라이브 때와 똑같이 확인:

```bash
ros2 topic list                                   # /camera0..3/image_raw/compressed 보임
ros2 topic hz /camera0/image_raw/compressed       # 재생 fps 확인(원본 30fps 근처)
ros2 topic echo /camera0/image_raw/compressed --field header   # stamp/frame_id만(바이너리 제외)
```

> `ros2 topic echo`를 필드 지정 없이 쓰면 JPEG 바이트 수만 개가 쏟아진다. 이미지는
> `--field header`(헤더만) 또는 `--no-arr`(배열 생략)로 본다.

**재생 화면을 눈으로 보기** — `monitor`는 토픽 이름만 같으면 라이브·재생을 구분하지 않는다.
한 터미널에서 `monitor`, 다른 터미널에서 `bag play` 하면 브라우저에서 녹화분이 그대로 재생된다:

```bash
# 터미널 A
ros2 run econ_camera_ros monitor            # http://<Orin-IP>:10010
# 터미널 B
ros2 bag play rosbag2_<timestamp> --loop    # 반복 재생
```

자주 쓰는 `play` 옵션:

| 옵션 | 설명 |
|---|---|
| `--rate 0.5` / `--rate 2.0` | 재생 속도 배율(느리게/빠르게) |
| `--loop` | 끝나면 처음부터 반복 |
| `--start-paused` | 일시정지 상태로 시작. **스페이스바**로 재생/정지 토글 |
| `--start-offset 10` | 앞 10초 건너뛰고 시작 |
| `--topics /camera0/image_raw/compressed` | 지정 토픽만 재생 |
| `--clock` | `/clock` 발행(다른 노드가 `use_sim_time`으로 bag 시각을 따르게 할 때) |

> **QoS 참고**: 녹화 토픽은 Reliable, `monitor` 구독은 BestEffort다. Publisher(Reliable)가
> Subscriber(BestEffort)보다 강한 조합이라 호환되어 정상 수신된다.

### bag 복사 · 일부만 추리기 (`ros2 bag convert`)

토픽을 골라내거나 mcap을 다시 묶을 때. `out.yaml`에 입출력/필터를 적어 실행한다.

```yaml
# convert.yaml — 카메라 0,1만 새 bag으로
output_bags:
  - uri: rosbag2_cam01
    storage_id: mcap
    topics: [/camera0/image_raw/compressed, /camera1/image_raw/compressed]
```
```bash
ros2 bag convert -i rosbag2_<timestamp> -o convert.yaml
```

---

## 6. bag → 학습용 JPEG 추출 (`bag_extract`)

녹화된 bag에서 4대를 **동기 세트**로 묶어 프레임별 폴더로 떨군다.

```bash
ros2 run econ_camera_ros bag_extract rosbag2_<timestamp> -o dataset
# 또는 ROS 없이도(순수 로직):  python3 -m econ_camera_ros.bag_extract <bag> -o dataset
```

출력 구조:
```
dataset/
  frame_000000/  cam0.jpg cam1.jpg cam2.jpg cam3.jpg
  frame_000001/  ...
  sets.csv       # idx, stamp0..stamp3, spread_ms
```

- `sets.csv`의 `spread_ms` = 그 세트 4대 stamp의 최대-최소(ms). frame_sync가 잘 맞으면
  전부 sub-ms.
- 페이로드가 이미 JPEG라 **디코드 없이** 그대로 파일로 쓴다(빠르고 무손실).
- **완성 세트(4대 다 채워진 것)만** 저장. 한 대라도 빠진 순간은 세트에서 제외된다.

#### 옵션

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `-o, --out` | `extracted` | 출력 디렉터리 |
| `--tolerance` | `0.001` | 동기 허용오차(초). 이 값 이내로 묶인 프레임만 한 세트 |
| `--limit` | (없음) | 처음 N 세트만 추출(빠른 확인용) |

> **허용오차 1ms**: frame_sync 동기가 실측 sub-ms라 1ms면 충분히 여유롭다. 1ms를 넘는
> 프레임은 정상 동기 세트가 아니므로 자연스럽게 걸러진다(품질 필터 겸용). 하드웨어 동기가
> 없는 데이터를 다룰 때만 값을 키운다.

---

## 7. 문제 해결 (Troubleshooting)

| 증상 | 확인 / 해결 |
|---|---|
| `No cameras available` / Argus 오류 | 정상. 이 카메라는 UYVY 직출력이라 Argus 불가. `v4l2src` 경로(이 패키지)를 쓴다. |
| `ros2 run`이 패키지를 못 찾음 | `source install/setup.bash` 안 함. 새 터미널마다 필요. |
| `frame_sync 설정 실패: /dev/videoN` 경고 | 해당 노드가 권한/장치 문제. `v4l2-ctl -d /dev/videoN -L`로 컨트롤 확인, 장치 연결 점검. |
| bag의 토픽별 메시지 수가 크게 다름 | 워밍업이 시작 불일치는 잡지만, **중간 드롭**은 별개. `capture` 로그의 `frames={...}`와 `동기 spread` 확인. USB/CSI 대역·발열 점검. |
| `nvjpegenc`/`nvvidconv` 없음 | JetPack HW 플러그인 미설치. Jetson 정품 이미지/GStreamer nvidia 플러그인 확인. |
| 모니터 포트 바인드 실패 | 다른 포트로: `monitor --ros-args -p port:=10011`. |
| 헤드리스에서 모니터 안 보임 | 브라우저로 접속. SSH면 `ssh -L 10010:localhost:10010`. |

**디버깅 원칙**: 프레임 수 불일치·동기 문제는 추측 말고 `capture`의 주기 로그
(`동기 spread=... std=... | frames={...}`)로 **어느 카메라가 어디서 어긋나는지** 먼저 관측한다.

---

## 8. 개발 · 테스트

순수 로직(파이프라인 문자열 빌더, 동기 그룹핑, 모니터 HTML/MJPEG)은 **하드웨어 없이**
검증한다. 실제 캡처·동기·녹화는 4대 실기에서 수동 검증.

```bash
# 순수 로직 테스트 (ROS 소싱 불필요)
cd src/econ_camera_ros && python3 -m pytest test/ -q      # 18 passed
```

- 파일은 관심사별로 작게: `gst_builder`(파이프라인 문자열) / `capture_node`(캡처·발행) /
  `web_monitor_node`(시각화) / `bag_extract`(추출).
- 선행 프로젝트의 `econ_cam.controls`(frame_sync) / `econ_cam.stats`(동기 지표)는
  **복붙 금지, import 재사용**.

---

## 9. LiDAR 함께 수집 (카메라 + LiDAR 통합 bag)

Unitree 4D LiDAR L2(IMU 내장)가 같은 ws에 통합되어 있다. 카메라 4대 + LiDAR 점군/IMU/TF를
**단일 bag(mcap)** 으로 동시 녹화한다.

```bash
# 사전조건: 호스트 NIC 192.168.1.2/24, ping 192.168.1.62 OK
ros2 launch econ_camera_ros record_all.launch.py
```

녹화 토픽(8): `/camera{0..3}/image_raw/compressed`, `/unilidar/cloud`, `/unilidar/imu`, `/tf`, `/tf_static`.
`ros2 bag info <bag>`로 8토픽·메시지 수를 확인한다.

**LiDAR만 녹화** (카메라 없이 라이다 점검·수집용):
```bash
ros2 launch econ_camera_ros record_lidar.launch.py
```
녹화 토픽(4): `/unilidar/cloud`, `/unilidar/imu`, `/tf`, `/tf_static`.

- 연결·빌드·검증·문제해결 상세: [`LIDAR.md`](LIDAR.md)
- 캘리브레이션(intrinsic/extrinsic)은 **정적 상수** → bag에 넣지 않고 `calib.yaml` 사이드카.
- 동적 데이터(점군·IMU)는 **주행 중 bag에 기록** → ego-pose는 사후 복원(LIO).
- 융합/BEV 설계: [`superpowers/specs/2026-07-18-data-collection-bag-and-fusion-design.md`](superpowers/specs/2026-07-18-data-collection-bag-and-fusion-design.md)

---

## 10. 오프라인 매핑 (bag → 궤적·맵)

녹화된 bag에서 ego 궤적과 3D 맵을 복원한다(BEV 데이터 전제). 상세는 `docs/MAPPING.md`.

```bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions   # 최초 1회
colcon build --packages-select point_lio && source install/setup.bash
./mapping/lio_map_bag.sh <bag_경로> data/ds_mapping_out
```
산출물: `data/ds_mapping_out/{map.pcd, trajectory.tum, run_info.txt, preview/}`.
