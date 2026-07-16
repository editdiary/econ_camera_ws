# e-con 4-Camera ROS2 연속 수집 — 설계 문서 (Design Spec)

- 작성일: 2026-07-16
- 상태: 설계 확정 (구현 전)
- 대상 하드웨어: NVIDIA Jetson AGX Orin (JetPack 6.1 / L4T R36.4, Ubuntu 22.04), e-con systems AR0234 4-camera 모듈
- 선행 프로젝트: `../Multi-Cam_module_test` (Flask 기반 촬영·캘리브레이션 테스트 도구). 본 프로젝트는
  그 프로젝트의 순수 로직(`econ_cam.controls`, `econ_cam.stats`)을 재사용한다.

---

## 1. 목표 (Goal)

연결된 4대의 e-con AR0234 카메라를 **하드웨어 동기(`frame_sync`)가 맞춰진 상태로 끊김 없이 연속
수집**하여 **ROS2 bag(mcap)** 으로 저장한다. **스크립트/런치 하나를 실행하면 촬영이 시작되어
4대의 이미지가 지정 프레임레이트로 bag에 계속 쌓이는 것**이 최종 결과다. 수집된 데이터는 이후
**딥러닝 모델 학습**에 사용한다.

향후 LiDAR 등 다른 센서를 같은 ROS2 워크스페이스에 추가할 수 있도록, 카메라 촬영 코드를
**ROS2 패키지 형태**로 만든다(단, LiDAR 통합 자체는 본 spec 범위 밖).

## 2. 범위 (Scope)

### 포함
- **ROS2 카메라 수집 노드**: 단일 공유클럭 GStreamer 파이프라인으로 4대를 동시 캡처, HW JPEG
  인코딩 후 카메라별 `sensor_msgs/CompressedImage` 토픽 발행.
- **동기 품질 주기 로그**: 촬영 중 4대 타임스탬프 편차(spread/std, ms)를 주기적으로 로그.
- **녹화 + 실행**: 런치 파일 하나로 카메라 노드 + `ros2 bag record`(mcap)를 함께 기동.
- **온디맨드 실시간 시각화**: 발행 중인 토픽을 구독해 4대를 2×2 그리드로 표시하는 **별도 프로세스**
  모니터(녹화와 격리, 안 켜면 비용 0).
- **순수 로직 단위 테스트**(하드웨어 불필요): 파이프라인 문자열 빌더 등.

### 제외 (본 spec 범위 밖)
- **LiDAR 패키지** (같은 ws에 후속 추가).
- **`camera_info`(K·왜곡계수) 발행** (선행 프로젝트 Mode 4에서 계산 가능하나 이번엔 미발행).
- **Docker 환경** — ROS2는 호스트 네이티브(Humble) 설치로 결정.
- **실제 extrinsic/캘리브레이션 연산**, **후처리·데이터셋 추출 파이프라인**.

## 3. 하드웨어 / 환경 사실 (탐색으로 확인됨)

| 항목 | 내용 |
|------|------|
| 카메라 | e-con AR0234 4대, `/dev/video0`~`/dev/video3`, `tegra-video`(CSI) 드라이버 |
| 출력 포맷 | **UYVY** 4:2:2 (모듈 내부 디베이어링 완료 → ISP/Argus 미경유), NV16도 지원 |
| 캡처 경로 | **순수 V4L2 경로** (`v4l2src`). **libargus/Isaac ROS Argus Camera 사용 불가** (§4 참조) |
| 목표 수집 | **1280×720 @ 30Hz** (`frame_sync=1`) |
| 동기화 | V4L2 `frame_sync` (menu): `0=Disable`, `1=30Hz`, `2=60Hz`. `v4l2-ctl -c frame_sync=1 -d /dev/videoN` |
| HW 가속 | `nvvidconv`(변환), `nvjpegenc`(JPEG 인코딩) — GStreamer 시스템 플러그인 |
| 플랫폼 | Jetson AGX Orin, 12코어, 61GB RAM, NVMe 845GB 여유 |
| ROS2 | **미설치** → 호스트에 **Humble** 네이티브 설치 예정 (JetPack 6.1 = Ubuntu 22.04 매치) |

## 4. Argus 배제 근거 (탐색으로 확정)

Isaac ROS Argus Camera / `nvarguscamerasrc` / libargus는 **Bayer 센서가 Jetson ISP를 경유**하는
경우에만 동작한다(`센서(RAW Bayer) → CSI → VI → ISP → libargus`). 본 하드웨어에서는:

- `/dev/video0`이 **UYVY/NV16**(YUV)로 출력 → 모듈이 디베이어링을 끝내고 **ISP를 우회**.
- 실측: `gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=1 ! fakesink` → **"No cameras
  available"**. nvargus-daemon이 떠 있어도 Argus가 이 카메라들을 하나도 인식하지 못한다.

따라서 촬영은 **`v4l2src` 순수 V4L2 경로**로 진행한다. "하드웨어 캡처 타임스탬프"라는 Argus의 이점은
V4L2 경로에서도 이미 확보된다: `v4l2src` 버퍼 PTS가 커널 VI 캡처 타임스탬프(monotonic, HW 기반)에서
나온다. 카메라 간 동기 비교에 필요한 것은 (1) `frame_sync` HW 트리거와 (2) 단일 공유 클럭이며, 둘 다
본 설계로 충족된다.

## 5. 핵심 기술 결정 (Key Technical Decisions)

1. **ROS2 = 호스트 네이티브 설치 (Humble)**. Docker 미사용. (근거: 아래 §11 트레이드오프.)
2. **캡처·인코딩 = GStreamer (Python `gi` + appsink)**. 선행 프로젝트에서 실기 검증된
   `v4l2src ! UYVY ! nvvidconv ! nvjpegenc ! appsink` 경로 계승. cv2는 모니터의 JPEG 디코드/표시에만 사용.
3. **다중 동기 = 단일 파이프라인(4개 `v4l2src`, 공유 클럭)**. 카메라별 `appsink`에서 버퍼 PTS를
   같은 기준으로 비교 가능. 선행 프로젝트 `SyncSession` 구조 계승(단, valve/tee 게이팅 제거 → 연속 스트림).
4. **타임스탬프 = appsink 버퍼 PTS**. 공유 파이프라인 클럭에서 산출 → `header.stamp`로 사용,
   카메라 간 stamp가 직접 비교 가능. (파이프라인 base-time을 ROS 시각에 앵커링해 wall-clock 의미 부여.)
5. **저장 이미지 = `sensor_msgs/CompressedImage` (JPEG)**. HW `nvjpegenc`(고품질, `quality≈90`)
   재사용. 프레임당 ~180KB로 전달·기록 부담 최소 → 프레임 드롭 위험 최소. 딥러닝 학습에 JPEG 적합.
6. **녹화 = rosbag2, 스토리지 mcap**. 런치 파일이 노드와 함께 `ros2 bag record`를 기동.
7. **컨트롤(frame_sync 등) = `econ_cam.controls` 재사용** (`v4l2-ctl` subprocess 래퍼).

## 6. 아키텍처 (Architecture)

```
[단일 GStreamer 파이프라인 · 공유 클럭]
  v4l2src /dev/video0 ─ UYVY ─ nvvidconv ─ nvjpegenc ─ image/jpeg ─ appsink sink0 ┐
  v4l2src /dev/video1 ─ UYVY ─ nvvidconv ─ nvjpegenc ─ image/jpeg ─ appsink sink1 ├→ capture_node
  v4l2src /dev/video2 ─ UYVY ─ nvvidconv ─ nvjpegenc ─ image/jpeg ─ appsink sink2 │   (new-sample 콜백마다
  v4l2src /dev/video3 ─ UYVY ─ nvvidconv ─ nvjpegenc ─ image/jpeg ─ appsink sink3 ┘    CompressedImage publish)
                                                                                        │
                    ┌───────────────────────────────────────────────────────────────────┤
                    ▼ (구독)                                        ▼ (구독)             ▼ (주기)
              ros2 bag record (mcap)                          monitor_node        동기 품질 로그
              /camera{0..3}/image_raw/compressed              (온디맨드 2×2)      (spread/std ms)
```

- **노드 시작**: `econ_cam.controls.set_frame_sync(dev, 1)`로 4대 HW 동기 설정 → 파이프라인 PLAYING.
- **발행**: 각 `appsink` `new-sample` 콜백에서 JPEG 버퍼를 꺼내 `header.stamp = PTS`, `header.frame_id
  = "cameraN"`로 `CompressedImage` 즉시 발행.
- **동기 로그**: 카메라별 최근 PTS 링을 유지, `econ_cam.stats`로 매칭·통계 산출해 N초마다 로그.
- **종료**: SIGINT(Ctrl-C)에 파이프라인 NULL 전환 + bag 정상 종료.

### 대안 및 배제
- **gscam ×4**: 카메라마다 파이프라인 분리 → 공유 클럭 동기 어려움, nvjpegenc/CompressedImage 연결 번거로움. 배제.
- **개별 노드 ×4**: 클럭 분리로 카메라 간 PTS 비교 부정확 → 동기 검증 불가. 배제.

## 7. 토픽 & 메시지 (Interfaces)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/camera0/image_raw/compressed` ~ `/camera3/image_raw/compressed` | `sensor_msgs/CompressedImage` | `format="jpeg"`, `header.stamp`=PTS, `header.frame_id`="camera{N}" |

- ROS 메시지는 **카메라별 독립 4개 스트림**으로 발행된다(하나로 묶지 않음). "같은 순간 4장"의
  매칭은 **소비 시점**에 `header.stamp` 기준으로 수행한다(`message_filters.ApproximateTimeSynchronizer`
  또는 bag 후처리). `frame_sync` + 공유 클럭 덕분에 이 매칭이 안정적이다.
- **QoS**: 발행은 **reliable + keep-last(적정 depth)** 로 두어 recorder가 프레임을 놓치지 않게 한다
  (recorder 구독도 reliable). **모니터 구독자는 best-effort** 로 두면, reliable 발행자라도 모니터에는
  재전송/backpressure가 발생하지 않아 모니터가 느려져도 발행·녹화에 영향이 없다(§9 격리).

## 8. 파일 구조 (File Structure)

```
econ_camera_ws/                         # colcon 워크스페이스 루트 = 프로젝트 루트
  CLAUDE.md                             # 본 프로젝트 작업 지침
  README.md                             # 개요·설치·실행
  .gitignore                            # build/ install/ log/ 등 제외
  docs/
    superpowers/specs/2026-07-16-econ-camera-ros2-capture-design.md   # 본 문서
    (plans/ ...)                        # 후속 구현 계획
  src/
    econ_camera_ros/                    # ament_python 패키지
      package.xml
      setup.py  setup.cfg
      resource/econ_camera_ros
      econ_camera_ros/
        __init__.py
        gst_builder.py                  # 연속 수집 파이프라인 문자열 빌더 (순수 함수, 테스트 가능)
        capture_node.py                 # 파이프라인 소유 + CompressedImage 발행 + 동기 로그
        monitor_node.py                 # 온디맨드 2×2 시각화 (구독 전용, cv2)
      launch/
        record.launch.py                # 카메라 노드 + ros2 bag record 동시 기동
      test/
        test_gst_builder.py             # 파이프라인 문자열 단위 테스트
  (build/ install/ log/)                # colcon 산출물 (gitignore)
```

**코드 재사용**: `econ_cam.controls`(frame_sync/detect), `econ_cam.stats`(동기 통계)는 선행
프로젝트에서 가져온다 — `pip install -e ../Multi-Cam_module_test`. 연속 수집 파이프라인은 기존
valve/tee 구조와 달라 `gst_builder.py`에 신규 작성한다.

## 9. 실시간 시각화 (Monitor) — 격리·저비용

- **별도 프로세스·구독 전용·온디맨드**: 카메라 파이프라인/인코딩을 추가로 돌리지 않고, 이미 발행 중인
  CompressedImage 토픽을 구독만 한다. 비용은 **JPEG 디코드 + 표시**뿐.
- **저비용화**: 표시 ~10fps로 throttle + 다운스케일, best-effort QoS.
- **녹화와 격리**: 모니터가 느려져도 recorder(별도 구독자)에 backpressure를 주지 못한다 →
  **모니터가 버벅여도 bag 프레임은 빠지지 않는다.** 안 켜면 비용 0.
- 실행: `ros2 run econ_camera_ros monitor` (촬영 중 다른 터미널에서).

## 10. 리소스 / 프레임 드롭 대책

- **인코딩**: JPEG는 HW(`nvjpegenc`) → CPU 부하 낮음.
- **기록량**: 4 × 30fps × ~180KB ≈ **~21MB/s** → NVMe(845GB 여유)에 여유. 촬영 시간은 길지 않음(수 분 단위).
- **드롭 방지**: `appsink`에서 무단 드롭을 피하도록 적정 큐 설정, 노드가 **카메라별 수신 프레임
  카운트를 주기 로그**해 한 대라도 뒤처지면 즉시 확인.
- **검증**: 촬영 중 `tegrastats`로 CPU/EMC/전력 실측.

## 11. 환경 트레이드오프 기록 (Docker vs 호스트 네이티브)

- **호스트 네이티브(채택)**: HW GStreamer 플러그인을 바로 사용, ROS2 개발/실행 단순. 대신 호스트
  apt에 ROS2 Humble(수백 패키지)이 추가된다.
- **Docker(미채택)**: 호스트 apt 무변경 가능하나, 컨테이너에서 Jetson HW GStreamer(`nvvidconv`/
  `nvjpegenc`) 접근을 위해 NVIDIA 컨테이너 런타임 + L4T 베이스 이미지 세팅이 필요.
- 결정: 사용자가 **호스트 네이티브 설치**를 선택. (선행 프로젝트의 "호스트 apt 무변경" 규칙은
  본 신규 프로젝트에는 적용하지 않는다.)

## 12. 검증 기준 (Success Criteria)

### 자동 (pytest / colcon test, 하드웨어 불필요)
- `test_gst_builder.py`: 파이프라인 문자열이 각 device/UYVY caps/`nvjpegenc`/`appsink name=sink{dev}`을
  올바르게 포함하는지 검증.

### 수동 (실제 4대 하드웨어)
1. `ros2 launch econ_camera_ros record.launch.py` 실행 → 4개 토픽이 **각각 ~30Hz**로 발행됨
   (`ros2 topic hz /camera0/image_raw/compressed`).
2. 노드 로그에 **동기 품질(spread/std ms)** 이 주기적으로 표시되고, `frame_sync=1`에서 편차가 작음.
3. `ros2 bag info`로 4개 토픽이 mcap에 기록되고, 카메라별 메시지 수가 시간×30에 근접(드롭 미미).
4. 촬영 중 `ros2 run econ_camera_ros monitor`로 2×2 그리드가 뜨고, **모니터 실행 여부와 무관하게**
   bag 기록이 안정적.
5. `tegrastats`로 CPU/전력이 여유 범위.

## 13. 열린 항목 / 향후 (Out of scope, tracked)
- LiDAR 패키지 및 카메라-LiDAR 시각/시간 동기.
- `camera_info` 발행(선행 프로젝트 Mode 4 intrinsic 재사용).
- bag → 학습용 프레임/데이터셋 추출 파이프라인.
- 필요 시 Docker/ROS2 이식.
