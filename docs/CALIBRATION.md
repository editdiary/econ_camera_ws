# 카메라 캘리브레이션 운영 가이드 (어안 4대)

4대 어안(180°) 카메라의 **intrinsic**(K·왜곡)과 **카메라 간 extrinsic**(상대 자세)을
Kalibr로 구하고 `calib.yaml` 사이드카로 남기는 실전 절차. 실기(Jetson AGX Orin, arm64)에서
전 과정을 관통해 검증한 내용을 그대로 담았다.

- 설계 근거: [`superpowers/specs/2026-07-18-camera-calibration-and-verification-design.md`](superpowers/specs/2026-07-18-camera-calibration-and-verification-design.md)
- 구현 계획: [`superpowers/plans/2026-07-18-camera-calibration-and-verification.md`](superpowers/plans/2026-07-18-camera-calibration-and-verification.md)
- cam↔LiDAR extrinsic(`T_front_lidar`)은 별도 문서: [`CAM_LIDAR_CALIBRATION.md`](CAM_LIDAR_CALIBRATION.md).

---

## TL;DR (한 번 익힌 뒤 요약용)

```bash
# 0) (최초 1회) Kalibr arm64 이미지 빌드
sudo bash calibration/build_kalibr_arm64.sh

# 1) 촬영 — 보드를 크게·여러 포즈로, 인접 카메라 겹침도 포함
ros2 launch econ_camera_ros record.launch.py          # 찍고 Ctrl-C

# 2) 추출 → 3) Kalibr 데이터셋 → 4) 실행
ros2 run econ_camera_ros bag_extract <bag_dir> -o extracted
ros2 run econ_camera_ros kalibr_bridge extracted -o dataset --rate 4.0
cp calibration/aprilgrid.yaml .
cp calibration/orientation.example.json orientation.json   # 커버 테스트로 확인한 방향으로 편집
sudo bash calibration/run_kalibr.sh "$(pwd)"           # ds-none·eucm-none 둘 다

# 5) 리포트 비교 → 6) calib.yaml
cat calib-results-cam-ds.txt calib-results-cam-eucm.txt
ros2 run econ_camera_ros calib_convert calib-camchain-ds.yaml \
  --model ds -o calib.yaml --orientation orientation.json \
  --rms front=.. right=.. rear=.. left=..
```

---

## 0. 사전 준비 (최초 1회)

### 0.1 Kalibr arm64 Docker 이미지

Kalibr 는 ROS1(Noetic) 도구라 arm64 네이티브 이미지를 빌드해 쓴다. 한 스크립트로 완성된다:

```bash
sudo bash calibration/build_kalibr_arm64.sh          # 수십 분 (catkin 빌드)
```

내부에서 하는 일(참고): ethz-asl/kalibr clone → 베이스 `osrf/ros:noetic-desktop-full`(arm64
미제공)를 **`arm64v8/ros:noetic`** 로 교체 → vision 스택(cv_bridge 등) apt 보충 →
`docker build --network=host` → cv2 선로딩 패치 레이어 적용 → 최종 `kalibr:arm64` 태깅.

> **왜 sudo?** 현재 계정이 `docker` 그룹 밖. 매번 sudo가 번거로우면 한 번만
> `sudo usermod -aG docker $USER` 후 재로그인.

동작 확인(스모크 테스트):
```bash
sudo docker run --rm --network=host -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
  --entrypoint bash kalibr:arm64 \
  -c 'source /catkin_ws/devel/setup.bash && rosrun kalibr kalibr_calibrate_cameras --help' | head
```
→ 옵션 도움말이 나오면 정상. (`Unable to init server`/`Gdk-CRITICAL` 경고는 헤드리스라 뜨는
무해한 X11 메시지.)

### 0.2 타깃

`calibration/aprilgrid.yaml` = 보유 보드 실측값(7×5, 태그 4cm, 태그 사이 1cm):
```yaml
target_type: 'aprilgrid'
tagCols: 7
tagRows: 5
tagSize: 0.04       # [m]
tagSpacing: 0.25    # 간격 1cm / 태그 4cm
```

### 0.3 수집 패키지 빌드

```bash
colcon build --packages-select econ_camera_ros
source install/setup.bash
```

---

## 1. 촬영 (가장 중요 — 결과 품질을 좌우)

리그를 **완전히 고정**한 뒤 `record.launch.py`로 녹화하며 AprilGrid 판을 움직인다.

```bash
ros2 launch econ_camera_ros record.launch.py     # 충분히 찍고 Ctrl-C → rosbag2_<ts>/
```

**어안 캘리브 촬영 원칙:**

| 항목 | 요령 | 이유 |
|---|---|---|
| **크기** | 보드가 화면 **가로 절반 이상** 차지. 태그 한 변 **≥ 50–80px** | 태그가 작으면 검출 실패(실측: ~25px는 검출 0) |
| **포즈** | 정면·좌우·상하 틸트, 원근 다양, **극단 가장자리·모서리까지** | 어안은 주변부 왜곡이 가장 크고 데이터가 부족한 곳 |
| **선명도** | 천천히(모션블러 금지), 조명 밝게, 초점 확인 | 흐리면 검출 실패 |
| **카메라별** | 4대 각각 근접 다포즈 구간을 둔다 | intrinsic 은 그 카메라가 잘 봐야 나옴 |
| **겹침** | 인접 쌍(0-1,1-2,2-3,3-0) 경계에서 **두 카메라가 동시에** 보드를 보게 | ⚠️ **카메라 간 extrinsic 은 동시 관측 없으면 못 품** |

> **intrinsic vs extrinsic:** intrinsic만 필요하면 한 대씩 크게로 충분. **카메라 간 extrinsic
> 까지** 원하면 인접 쌍이 동시에 보드를 보는 프레임이 반드시 있어야 한다(한 bag에 둘 다 담는다).

---

## 2. 동기 세트 추출

```bash
ros2 run econ_camera_ros bag_extract <bag_dir> -o extracted
```
→ `extracted/frame_NNNNNN/cam{0..3}.jpg` + `sets.csv`. 4대 하드웨어 동기(sub-ms)라 세트가
거의 손실 없이 묶인다(실측 spread ~0.02–0.13ms).

---

## 3. Kalibr 데이터셋 생성 (브릿지)

```bash
ros2 run econ_camera_ros kalibr_bridge extracted -o dataset --rate 4.0
cp calibration/aprilgrid.yaml .        # dataset/ 과 같은 폴더에 둔다
```
→ `dataset/cam{0..3}/<나노초>.png`. 30fps 세트를 **~4Hz로 다운샘플**(중복·블러 프레임 제거로
최적화 안정화). 한 세트의 4대는 같은 타임스탬프를 파일명으로 공유한다.

### 3.1 카메라 방향 매핑 (`orientation.json`) — 포트 재연결 대비

포트 연결에 따라 cam 번호(`/dev/video0~3`)가 바뀔 수 있다. 360° 어안 4대는 **인접(90°)만
겹치고 반대편은 안 겹치므로**, Kalibr `--topics` 순서상 연속된 카메라가 물리적으로 인접해야
extrinsic 이 풀린다. 이를 매 촬영마다 신경 쓰지 않도록 **방향 매핑 파일**을 둔다.

```bash
cp calibration/orientation.example.json orientation.json   # dataset/ 옆에 두고 편집
```

**어느 cam이 어느 방향인지 확인:** `monitor`(2×2 화면)를 켜고 카메라를 하나씩 손으로 가려,
어두워지는 칸의 cam 번호를 그 방향에 적는다.

```json
{"cam0": "left", "cam1": "right", "cam2": "front", "cam3": "rear"}
```

- 값은 정확히 `front/right/rear/left` 각 1개씩. `run_kalibr.sh`가 이 파일을 읽어 링 순서
  `front→right→rear→left`로 토픽을 정렬한다. **파일이 없으면 기존 cam0~3 순서로 폴백**.
- 실행 끝에 `Kalibr cam{i} = 방향 (/camN/image_raw)` 매핑 표가 출력된다 → `results.txt`(Kalibr는
  cam0~3 라벨)를 방향과 대조하고, `calib_convert --rms` 입력에 사용.

---

## 4. Kalibr 실행 (ds-none·eucm-none 둘 다 자동)

`dataset/` 과 `aprilgrid.yaml` 이 있는 폴더를 인자로:

```bash
sudo bash calibration/run_kalibr.sh "$(pwd)"      # 모델당 10~15분 (Jetson)
```

내부: `kalibr_bagcreater`로 ROS1 bag 생성 → `ds-none`, `eucm-none` 각각
`kalibr_calibrate_cameras` 실행 → 모델별 접미사로 저장:
```
calib-camchain-ds.yaml   calib-results-cam-ds.txt   calib-report-cam-ds.pdf
calib-camchain-eucm.yaml calib-results-cam-eucm.txt calib-report-cam-eucm.pdf
```

> `orientation.json`을 두면 링 순서로 자동 정렬되고, 실행 끝에 **Kalibr cam↔방향 매핑 표**가
> 출력된다. `results.txt`의 `cam0~3`은 이 표 기준(링 순서: front·right·rear·left)임에 유의.

**Kalibr 파이프라인이 오래 걸리는 이유:** 1136장(284×4) 서브픽셀 코너 추출 → 카메라 intrinsic +
카메라 간 extrinsic + **모든 뷰의 보드 6DoF 자세**를 한꺼번에 푸는 배치 번들 조정 → 뷰를
점진 추가하며 outlier 제거 후 재최적화 → 리포트 렌더. Jetson(ARM)에서 두 모델이면 20~30분 정상.

> `run_kalibr.sh` 가 자동 처리하는 실기 함정(참고): `--network=host`(iptables raw 테이블 부재
> 우회), `--entrypoint bash`(Kalibr 이미지의 shell-form ENTRYPOINT가 명령을 무시하는 것 우회),
> `KALIBR_MANUAL_FOCAL_LENGTH_INIT=1`(어안 초점거리 자동초기화 실패 대비),
> 모델명 **`ds-none`/`eucm-none`**(Kalibr 유효 목록).

---

## 5. 결과 판정 — 무엇이 "좋은" 결과인가

### 5.1 검출 수 (커버리지)

실행 로그의 `Extracted corners for N images (of M)` — 카메라별 검출 프레임 수. **적은 카메라가
있으면 그 방향으로 포즈가 부족**한 것(재촬영 신호). report PDF 의 **관측 코너 커버리지 플롯**으로
이미지 어디가 비었는지 확인(어안은 주변부가 비기 쉽다).

**이미지별 코너 수가 필요하면**(어떤 프레임이 검출 부족인지 콕 집거나, 성긴 프레임을 걸러 품질을
높이려면) `run_detect_report.sh` 로 Kalibr 검출기 그대로 이미지별 코너 수 CSV(`detect_report.csv`)를
뽑는다. 단, intrinsic 초기화 크래시(§8 의 DLT)는 검출 수와 무관하므로 이 필터로는 못 고친다.

### 5.2 재투영 오차 (핵심 품질 지표)

`calib-results-cam-*.txt` 의 각 카메라 `reprojection error: [mean] +- [std]` 에서 **std(px)** 가
실질 지표(mean 은 ~0이어야 정상).

| std(px) | 판정 |
|---|---|
| ≲ 0.5 | 우수 |
| ~1 | 어안에선 양호 |
| 2–3 | 보통(포즈·커버리지 개선 여지) |
| ≳ 5 | 불량 — **재촬영 권장** |

- **잔차 산포**(report PDF)가 무작위 구름이어야 한다. **방사형/체계적 패턴**이면 모델 부적합
  신호 → 다른 모델(ds↔eucm) 또는 KB 검토.
- **정성 확인**: 왜곡 보정 후 직선이 직선으로 펴지는지(특히 주변부).

### 5.3 모델 채택 (ds vs eucm)

두 `*-results-*.txt` 의 카메라별 std 를 비교해 **더 작은 쪽** 채택. 보통 근소차이이며, 실측
사례에선 **DS(Double Sphere)** 가 전 카메라에서 근소 우수했다. 두 모델의 extrinsic(baseline)이
거의 일치하면 결과 일관성이 좋다는 신호.

### 5.4 실측 참고치 (검증 캡처, *비*프로덕션)

도구 검증용 캡처(포즈 다양성 부족)에서의 DS 결과 — **좋은 목표가 아니라 "이 정도면 재촬영" 예시**:

| cam | 재투영 std(px) | 검출 |
|---|---|---|
| cam1 | 1.6 | 237 |
| cam2 | 3.0 | 123 |
| cam3 | 3.0 | 160 |
| cam0 | 6.0 | 246 |

→ cam1은 양호, cam0는 불량. 원인은 포즈 다양성/커버리지 부족. §1 요령대로 재촬영하면 개선된다.

---

## 6. `calib.yaml` 생성

채택 모델의 camchain 과 재투영 std 를 넣어 사이드카를 만든다:

```bash
ros2 run econ_camera_ros calib_convert calib-camchain-ds.yaml \
  --model ds -o calib.yaml --orientation orientation.json \
  --rms front=3.0 right=1.6 rear=3.0 left=6.0     # 매핑 표 보고 방향별로(예: §5.4 실측을 방향으로 환산)
```

**`calib.yaml` 구조:**
```yaml
model_chosen: ds
cameras:
  cam0: {camera_model: ds, intrinsics: [xi, alpha, fx, fy, cx, cy], distortion_model: none,
         distortion_coeffs: [], resolution: [1280, 720]}
  cam1: ...   cam2: ...   cam3: ...
extrinsics:
  T_cam0_cam0: [[4x4 항등]]          # 기준
  T_cam1_cam0: [[4x4]]               # cam0→camN 점 변환(누적)
  T_cam2_cam0: [[4x4]]
  T_cam3_cam0: [[4x4]]
verification:
  reproj_rms_px: {cam0: .., cam1: .., cam2: .., cam3: ..}
```
- `intrinsics` 는 DS: `[xi, alpha, fx, fy, cx, cy]`, EUCM: `[alpha, beta, fx, fy, cx, cy]`. DS/EUCM은
  왜곡이 투영모델에 내장돼 `distortion_coeffs` 는 빈 리스트.
- 데이터셋 옆에 `calib.yaml` 을 둔다. cam↔LiDAR extrinsic 은 L2 도착 후 이 파일에 추가.

> `--orientation`을 주면 `cameras`·`extrinsics` 키가 `cam0~3` 대신 **방향명**(front/right/rear/
> left)이 되고 기준은 `front`(`T_right_front` 등)다. 카메라 번호가 바뀌어도 결과 해석이
> 안 흔들린다. 생략하면 기존 `camN` 라벨 그대로.

---

## 6.5 시각 검증 (`calib.yaml` 이 실제로 맞는지 눈으로)

Kalibr 리포트의 재투영 수치(§5.2)를 넘어, **실제 장면이 잘 펴지고 4대가 잘 이어지는지**를
본다. 검출 불필요(DS 투영/역투영만)라 호스트 파이썬만으로 돌고, calib 이미지든 직접 수집한
일반 이미지든 동일하게 적용된다. 상세·판정 포인트는 `calibration/verify/README.md`.

```bash
# 언디스토션(RAW|핀홀|원통) — 세상 직선이 곧게 펴지나
python3 calibration/verify/verify_undistort.py --images data/calib_260723/extracted --frames "800,1600"
# 360° 파노라마 — intrinsic+extrinsic 동시(먼 구조물이 이음새 넘어 이어지나)
python3 calibration/verify/verify_panorama.py  --images data/calib_260723/extracted --frames "800,1600"
# 카메라 간 겹침(checkerboard/blend) — extrinsic
python3 calibration/verify/verify_extrinsics.py --images data/calib_260723/extracted --frames "800,1600"
# 직접 수집 이미지에 적용(최종 실증) — --out 로 분리 저장
python3 calibration/verify/verify_undistort.py --images data/cam_out/raws1_out-images --out data/cam_out/verify/undistort
```

- **판정**: 핀홀에서 격자·창틀·기둥이 곧은 직선, 원통에서 수직선이 수직, 파노라마/겹침에서
  **먼 구조물**이 이음새를 넘어 매끄럽게 이어지면 양호.
- ⚠️ 파노라마/겹침은 회전-전용 근사라 **가까운 물체의 이음새 어긋남·유령현상은 정상**(시차).
- 정량 재투영 오차는 여전히 Kalibr 리포트(§5.2)가 권위 소스(AprilGrid 는 호스트에서 검출 불가, §7).

---

## 7. 도구 레퍼런스 (신규 캘리브 코드)

| 도구 | 위치 | 역할 |
|---|---|---|
| `build_kalibr_arm64.sh` | `calibration/` | Kalibr arm64 이미지 빌드(+vision 스택·cv2 선로딩) |
| `Dockerfile.patch` | `calibration/` | 기존 이미지에 vision 스택·cv2 선로딩 얹는 패치 레이어 |
| `run_kalibr.sh` | `calibration/` | bagcreater + ds-none/eucm-none 실행(실기 함정 자동 처리) |
| `run_detect_report.sh` | `calibration/` | Kalibr 검출기로 이미지별 코너 수 리포트(+`<N`코너 프레임 자동 제외) |
| `kalibr_detect_report.py` | `calibration/` | 위 래퍼가 컨테이너 안에서 실행하는 검출·필터 본체 |
| `aprilgrid.yaml` | `calibration/` | 타깃 설정(7×5/0.04/0.25) |
| `cam_layout` | `econ_camera_ros/` | orientation.json → 링 순서 토픽 정렬(run_kalibr·calib_convert 공유) |
| `orientation.example.json` | `calibration/` | cam↔방향 매핑 템플릿(복사·편집) |
| `kalibr_bridge` | `ros2 run econ_camera_ros` | 동기 세트 → Kalibr 데이터셋(4Hz 다운샘플) |
| `calib_convert` | `ros2 run econ_camera_ros` | Kalibr camchain → `calib.yaml` |
| `bag_extract` | `ros2 run econ_camera_ros` | bag → 동기 세트 JPEG(재사용) |
| `verify/` (undistort·panorama·extrinsics) | `calibration/verify/` | `calib.yaml` 시각 검증(§6.5, 검출 불필요·호스트 파이썬) |

> **커버리지 게이트 참고:** 초기엔 별도 cv2.aruco 기반 `calib_coverage` 를 뒀으나, 실기 어안
> 프레임에서 cv2.aruco·pupil-apriltags 모두 검출 실패(촘촘한 AprilGrid 배열은 낱개 태그
> 검출기가 못 읽음). **Kalibr 자체 검출이 권위 소스**라 그 도구는 은퇴했고, 커버리지는 §5.1로
> 판정한다.

---

## 8. 문제해결 (실기에서 실제로 겪은 것들)

| 증상 | 원인 | 해결 |
|---|---|---|
| `docker: ... iptables ... table 'raw': Table does not exist` | Jetson 커널에 iptables raw 모듈 없음 | `docker run/build` 에 **`--network=host`** (헬퍼에 반영됨) |
| `docker run` 이 **아무 출력 없이 exit 0** | Kalibr 이미지 shell-form ENTRYPOINT 가 명령 무시 | **`--entrypoint bash`** 로 덮어쓰기(헬퍼에 반영됨) |
| `ModuleNotFoundError: No module named 'cv_bridge'` | arm64 ros-base 에 vision 스택 없음 | 빌드 스크립트가 `ros-noetic-cv-bridge` 등 설치(반영됨) |
| `initialization of cv_bridge_boost raised unreported exception` | cv2(OpenCV)가 cv_bridge 보다 먼저 로드돼야 함 | `.pth` 로 cv2 선로딩(패치 레이어에 반영됨) |
| `--models ds` 인식 실패 | Kalibr 모델명은 **`ds-none`/`eucm-none`** | 헬퍼가 올바른 이름 사용 |
| 검출 0 / 재투영 std 큼 | 보드가 작음/흐림/포즈 부족 | §1 요령으로 재촬영(보드 크게·다포즈·선명) |
| `DLT algorithm needs at least 6 points ... 'count' is 5/4` (초기화 크래시) | Kalibr 버그: `*Projection::estimateTransformation` 이 코너를 광축 80° 콘으로 컬링한 뒤 `cv::solvePnP` 를 부르는데 가드가 `< 4` 뿐. 어안 주변부 프레임은 컬링 후 4~5점만 남아 통과 → DLT(≥6) 크래시(ds/eucm/omni/pinhole 전부 동일, 모델 교체·검출필터로 해결 불가). **검출 코너 수와 무관**(검출 40개도 컬링 후 4개 가능) | 가드를 `< 6` 으로 올린 뒤 이미지 재빌드 → `sudo docker build --network=host -t kalibr:arm64 -f calibration/Dockerfile.patch calibration/` (수 분, C++ 재컴파일). `Dockerfile.patch` 에 반영돼 있음 |
| `docker` 권한 거부 | 계정이 docker 그룹 밖 | `sudo` 또는 `usermod -aG docker $USER` 후 재로그인 |
| 산출물이 root 소유 | `sudo docker` 로 생성 | 필요 시 `sudo chown -R $USER:$USER <dir>` |

---

## 부록: 최종 데이터 흐름

```
[촬영]  record.launch.py → rosbag2_<ts>/ (카메라 4토픽)
   │  bag_extract → extracted/frame_*/cam{0..3}.jpg + sets.csv
   │  kalibr_bridge(4Hz) → dataset/cam{0..3}/<ns>.png
[Kalibr] run_kalibr.sh(Docker,arm64) → bagcreater → ds-none/eucm-none 캘리브
   │  → camchain·results·report(모델별)
[판정]  results.txt 재투영 std + report 커버리지/잔차 → 모델 채택 or 재촬영
   │  calib_convert → calib.yaml (intrinsic + extrinsic + reproj_rms)
[산출]  calib.yaml 사이드카 (+ 나중에 cam↔LiDAR extrinsic)
```
