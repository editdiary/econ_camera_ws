# 어안 4카메라 캘리브레이션 & 검증 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** e-con AR0234 4대(180° 어안)의 intrinsic/카메라 간 extrinsic을 Kalibr(arm64 Docker)로 계산하고, 촬영 품질을 현장에서 보장하며(커버리지 체크), 결과를 `calib.yaml` 사이드카로 남기고 검증(Kalibr 리포트 기반)하는 오프라인 툴체인과 운영 가이드를 만든다.

**Architecture:** 호스트(ROS2 Humble/plain Python) 측에 순수 로직 중심 툴 3종을 둔다 — `kalibr_bridge`(bag_extract 산출물 → Kalibr 데이터셋 폴더), `calib_coverage`(AprilGrid 검출률·커버리지 히트맵 현장 체크), `calib_convert`(Kalibr camchain → 프로젝트 `calib.yaml`). Kalibr 자체는 arm64로 빌드한 Docker 컨테이너에서 offline 실행한다. 기존 `capture`(녹화)·`bag_extract`(동기 세트 추출)는 재사용한다.

**Tech Stack:** Python 3.10, pytest(순수 로직), cv2(apt `python3-opencv`, 어안 프레임 검출·JPEG 디코드), PyYAML(camchain/calib), Docker(arm64 Kalibr = ROS1 Noetic), Kalibr(`kalibr_bagcreater`, `kalibr_calibrate_cameras`).

## Global Constraints

- 플랫폼: **Jetson AGX Orin, arch `aarch64`**, JetPack 6.1 / L4T R36.4 (Ubuntu 22.04). Docker 기설치(v29.5.2).
- Kalibr는 **ROS1 도구 → arm64 네이티브 Docker 이미지(`kalibr:arm64`)** 로만 실행. 베이스 `arm64v8/ros:noetic`.
- AprilGrid 확정값: `tagCols: 7`, `tagRows: 5`, `tagSize: 0.04`(m), `tagSpacing: 0.25`.
- 카메라 모델: **`ds`(Double Sphere)와 `eucm` 둘 다** 같은 데이터에 실행 후 리포트 비교로 채택.
- 캡처: `1280x720@30`, 하드웨어 `frame_sync` 동기. Kalibr 입력은 **~4Hz로 다운샘플**.
- 검증: **(A) 지금** = Kalibr 내장 리포트(재투영 RMS·잔차·쌍별 오차) 해석 + 정성 육안. 독립 도구(루프/cross-projection)는 **(B) 나중**.
- 헤드리스/SSH 환경: Kalibr 실행에 `--dont-show-report` 사용(GUI 없이 `report-*.pdf`+`*-camchain.yaml` 저장).
- 재사용: `capture`(녹화)·`bag_extract`(동기 세트 JPEG + `sets.csv`). 복붙 금지, import/산출물 재사용.
- 코드 관례(기존 `bag_extract` 패턴 준수): **순수 로직은 무거운 임포트(cv2 등) 없이 import·pytest 가능**해야 하며, cv2/파일 I/O는 함수 내부 지연 임포트 + `main()`에서만. 순수 로직만 단위 테스트, I/O·하드웨어는 수동 검증.
- 테스트 실행: `cd src/econ_camera_ros && python3 -m pytest test/`.
- Git: 구현은 **새 브랜치**에서, **커밋까지만**(merge/push는 사용자). 커밋 co-author 트레일러 유지.
- 절대 경로 기준: 프로젝트 `~/Desktop/econ_camera_ws`. Kalibr clone은 repo 밖(`~/Desktop/kalibr`).

## File Structure

```
econ_camera_ws/
  calibration/                         # Kalibr(비-Python) 아티팩트 — 컨테이너에 마운트
    aprilgrid.yaml                     # 확정된 타깃 설정 (Task 1)
    build_kalibr_arm64.sh              # kalibr clone + FROM 교체 + docker build (Task 1)
    run_kalibr.sh                      # bagcreater + ds/eucm 2회 실행 헬퍼 (Task 5)
    README.md                          # 빌드/실행 요약 (Task 1)
  src/econ_camera_ros/
    setup.py                           # console_scripts에 3개 엔트리포인트 추가
    econ_camera_ros/
      kalibr_bridge.py                 # extracted → Kalibr 데이터셋 폴더 (Task 2)
      calib_coverage.py                # AprilGrid 검출률·커버리지 현장 체크 (Task 3)
      calib_convert.py                 # Kalibr camchain → calib.yaml (Task 4)
    test/
      test_kalibr_bridge.py            # (Task 2) 순수 로직
      test_calib_coverage.py           # (Task 3) 순수 로직
      test_calib_convert.py            # (Task 4) 순수 로직
  docs/
    CALIBRATION.md                     # 촬영법 + 캘리브 실행 순서 운영 가이드 (Task 6)
```

---

### Task 1: Kalibr arm64 Docker 이미지 + AprilGrid 설정 + calibration/ 스캐폴딩

하드웨어/외부 의존 작업이라 pytest 불가 → **빌드 성공 + 스모크 테스트**로 검증한다.

**Files:**
- Create: `calibration/aprilgrid.yaml`
- Create: `calibration/build_kalibr_arm64.sh`
- Create: `calibration/README.md`

**Interfaces:**
- Consumes: 없음(환경 셋업).
- Produces: Docker 이미지 `kalibr:arm64`(컨테이너 내 `rosrun kalibr ...` 실행 가능), 마운트용 `calibration/aprilgrid.yaml`.

- [ ] **Step 1: AprilGrid 설정 파일 작성**

`calibration/aprilgrid.yaml`:
```yaml
target_type: 'aprilgrid'
tagCols: 7          # 가로 태그 개수
tagRows: 5          # 세로 태그 개수
tagSize: 0.04       # 태그 한 변 [m] = 4cm
tagSpacing: 0.25    # (태그 사이 간격 1cm) / (태그 4cm)
```

- [ ] **Step 2: 빌드 스크립트 작성**

`calibration/build_kalibr_arm64.sh`:
```bash
#!/usr/bin/env bash
# Kalibr(ROS1)를 arm64 네이티브로 빌드한다. Jetson(aarch64)에서 실행.
# 사용: bash calibration/build_kalibr_arm64.sh [KALIBR_CLONE_DIR]
set -euo pipefail

KALIBR_DIR="${1:-$HOME/Desktop/kalibr}"

if [ ! -d "$KALIBR_DIR/.git" ]; then
  git clone https://github.com/ethz-asl/kalibr.git "$KALIBR_DIR"
fi
cd "$KALIBR_DIR"

# 베이스 이미지를 arm64 명시본으로 교체(멀티아치라도 확실히 arm64 강제)
sed -i 's|^FROM ros:noetic|FROM arm64v8/ros:noetic|' Dockerfile_ros1_20_04

# 빌드(컴파일 양 많음 — 수십 분 소요 가능). docker 그룹 밖이면 sudo 필요.
docker build -t kalibr:arm64 -f Dockerfile_ros1_20_04 .

echo "DONE: image 'kalibr:arm64' built."
```

- [ ] **Step 3: 빌드 실행**

Run: `bash calibration/build_kalibr_arm64.sh`
Expected: 마지막에 `DONE: image 'kalibr:arm64' built.` 출력. (권한 오류 시 `sudo usermod -aG docker $USER` 후 재로그인, 또는 `sudo`로 재실행.)

- [ ] **Step 4: 스모크 테스트 — 컨테이너 내 Kalibr 실행 확인**

Run:
```bash
docker run --rm kalibr:arm64 bash -lc \
  'source /catkin_ws/devel/setup.bash && rosrun kalibr kalibr_calibrate_cameras --help' | head -n 20
```
Expected: `kalibr_calibrate_cameras`의 usage/옵션 도움말이 출력됨.
- 만약 `source: No such file` 이면 workspace 경로가 다른 것 → clone한 `Dockerfile_ros1_20_04`에서 실제 catkin workspace 경로를 확인해 그 경로로 교체(§Global의 경로 주의). 확인된 경로를 Step 5 README에 기록.

- [ ] **Step 5: README 작성**

`calibration/README.md`:
```markdown
# Kalibr 캘리브레이션 (arm64 Docker)

Jetson(aarch64)에서 Kalibr(ROS1)를 네이티브 Docker로 빌드·실행한다.

## 빌드
    bash calibration/build_kalibr_arm64.sh [클론경로(기본 ~/Desktop/kalibr)]
- 결과 이미지: `kalibr:arm64`
- 컨테이너 내 catkin workspace: `/catkin_ws` (스모크 테스트로 확인된 값; 다르면 여기 수정)

## 스모크 테스트
    docker run --rm kalibr:arm64 bash -lc \
      'source /catkin_ws/devel/setup.bash && rosrun kalibr kalibr_calibrate_cameras --help'

## 타깃
- `calibration/aprilgrid.yaml` (7x5, tagSize 0.04, tagSpacing 0.25)

## 실행
- 데이터 준비·실행 순서는 `docs/CALIBRATION.md` 참조.
```

- [ ] **Step 6: Commit**

```bash
git add calibration/aprilgrid.yaml calibration/build_kalibr_arm64.sh calibration/README.md
git commit -m "feat(calib): Kalibr arm64 Docker 빌드 스크립트 + AprilGrid 설정"
```

---

### Task 2: `kalibr_bridge` — extracted 산출물 → Kalibr 데이터셋 폴더

순수 로직(레이트 선택·타임스탬프·세트 파싱)만 단위 테스트. 파일 복사/디코드 I/O는 `main`에서 수동 검증.

**Files:**
- Create: `src/econ_camera_ros/econ_camera_ros/kalibr_bridge.py`
- Test: `src/econ_camera_ros/test/test_kalibr_bridge.py`
- Modify: `src/econ_camera_ros/setup.py` (엔트리포인트 추가)

**Interfaces:**
- Consumes: `bag_extract` 산출물 디렉터리(`frame_NNNNNN/cam{0..3}.jpg` + `sets.csv`).
- Produces:
  - `select_by_rate(stamps: list[float], target_hz: float) -> list[int]` — 유지할 세트 인덱스(시간 기반 greedy).
  - `stamp_to_ns(stamp_seconds: float) -> int`
  - `parse_set_stamps(csv_text: str) -> list[float]` — `sets.csv`의 각 세트 anchor stamp(첫 stamp 열).
  - `build_dataset(extracted_dir, out_dir, target_hz=4.0, devs=(0,1,2,3)) -> int` — I/O, 유지 세트 수 반환.

- [ ] **Step 1: 실패하는 테스트 작성**

`src/econ_camera_ros/test/test_kalibr_bridge.py`:
```python
from econ_camera_ros import kalibr_bridge as kb


def test_select_by_rate_downsamples_30hz_to_4hz():
    # 30fps 30프레임(0..0.9667s) → 4Hz(0.25s 간격)면 0,0.25,0.5,0.75초 부근 = 4개
    stamps = [i / 30.0 for i in range(30)]
    kept = kb.select_by_rate(stamps, target_hz=4.0)
    assert kept[0] == 0
    # 인접 유지 세트 간 실제 간격이 0.25s 이상
    for a, b in zip(kept, kept[1:]):
        assert stamps[b] - stamps[a] >= 0.25 - 1e-9
    assert len(kept) == 4


def test_select_by_rate_empty():
    assert kb.select_by_rate([], 4.0) == []


def test_stamp_to_ns_rounds():
    assert kb.stamp_to_ns(1.000000001) == 1000000001
    assert kb.stamp_to_ns(0.0) == 0


def test_parse_set_stamps_reads_first_stamp_column():
    csv_text = (
        "idx,stamp0,stamp1,stamp2,stamp3,spread_ms\n"
        "0,100.000000000,100.000100000,100.000200000,100.000050000,0.200\n"
        "1,100.033000000,100.033100000,100.033200000,100.033050000,0.200\n"
    )
    assert kb.parse_set_stamps(csv_text) == [100.0, 100.033]
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd src/econ_camera_ros && python3 -m pytest test/test_kalibr_bridge.py -v`
Expected: FAIL — `ModuleNotFoundError: econ_camera_ros.kalibr_bridge` (모듈 없음).

- [ ] **Step 3: 최소 구현**

`src/econ_camera_ros/econ_camera_ros/kalibr_bridge.py`:
```python
"""bag_extract 산출물(frame_NNNNNN/cam{d}.jpg + sets.csv) → Kalibr 데이터셋 폴더.

Kalibr(kalibr_bagcreater)는 cam{d}/<timestamp_ns>.png 폴더 구조를 ROS1 bag으로 변환한다.
한 동기 세트의 4대는 같은 anchor stamp(ns)를 파일명으로 공유하므로 카메라 간 연관이 자명하다.
캘리브 안정화를 위해 30fps 세트를 ~4Hz로 다운샘플한다.

순수 로직(select_by_rate/stamp_to_ns/parse_set_stamps)은 cv2 없이 테스트 가능하고,
디코드/복사(build_dataset)는 cv2를 지연 임포트한다.

실행:
    python3 -m econ_camera_ros.kalibr_bridge <extracted_dir> -o dataset --rate 4.0
    ros2 run econ_camera_ros kalibr_bridge <extracted_dir> -o dataset
"""

import argparse
import os


def select_by_rate(stamps, target_hz):
    """stamp 오름차순 리스트에서 목표 레이트로 다운샘플한 인덱스(시간 기반 greedy)."""
    if not stamps:
        return []
    period = 1.0 / target_hz
    kept = [0]
    last = stamps[0]
    for i in range(1, len(stamps)):
        if stamps[i] - last >= period:
            kept.append(i)
            last = stamps[i]
    return kept


def stamp_to_ns(stamp_seconds):
    """초 단위 stamp → 나노초 정수(파일명용)."""
    return int(round(stamp_seconds * 1e9))


def parse_set_stamps(csv_text):
    """sets.csv 텍스트에서 세트별 anchor stamp(첫 stamp 열)를 순서대로 반환."""
    lines = [l for l in csv_text.splitlines() if l.strip()]
    out = []
    for line in lines[1:]:  # 헤더 스킵
        cols = line.split(",")
        out.append(float(cols[1]))
    return out


def build_dataset(extracted_dir, out_dir, target_hz=4.0, devs=(0, 1, 2, 3)):
    """extracted_dir → out_dir/cam{d}/<ns>.png. 유지 세트 수 반환(cv2 지연 임포트)."""
    import cv2

    with open(os.path.join(extracted_dir, "sets.csv")) as f:
        stamps = parse_set_stamps(f.read())
    kept = select_by_rate(stamps, target_hz)
    for d in devs:
        os.makedirs(os.path.join(out_dir, f"cam{d}"), exist_ok=True)
    for idx in kept:
        ns = stamp_to_ns(stamps[idx])
        for d in devs:
            src = os.path.join(extracted_dir, f"frame_{idx:06d}", f"cam{d}.jpg")
            dst = os.path.join(out_dir, f"cam{d}", f"{ns}.png")
            img = cv2.imread(src)
            if img is None:
                raise FileNotFoundError(src)
            cv2.imwrite(dst, img)
    return len(kept)


def main():
    p = argparse.ArgumentParser(description="extracted → Kalibr 데이터셋 폴더")
    p.add_argument("extracted", help="bag_extract 출력 디렉터리")
    p.add_argument("-o", "--out", default="dataset", help="Kalibr 데이터셋 출력(기본 dataset)")
    p.add_argument("--rate", type=float, default=4.0, help="다운샘플 목표 Hz(기본 4.0)")
    a = p.parse_args()
    n = build_dataset(a.extracted, a.out, target_hz=a.rate)
    print(f"유지 세트 {n}개 → {a.out}/cam{{0..3}}/<ns>.png")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd src/econ_camera_ros && python3 -m pytest test/test_kalibr_bridge.py -v`
Expected: 4개 PASS.

- [ ] **Step 5: 엔트리포인트 추가**

`src/econ_camera_ros/setup.py` 의 `console_scripts` 리스트에 추가(기존 3줄 아래):
```python
            'bag_extract = econ_camera_ros.bag_extract:main',
            'kalibr_bridge = econ_camera_ros.kalibr_bridge:main',
```

- [ ] **Step 6: Commit**

```bash
git add src/econ_camera_ros/econ_camera_ros/kalibr_bridge.py \
        src/econ_camera_ros/test/test_kalibr_bridge.py \
        src/econ_camera_ros/setup.py
git commit -m "feat(calib): kalibr_bridge — extracted → Kalibr 데이터셋(~4Hz 다운샘플)"
```

---

### Task 3: `calib_coverage` — AprilGrid 검출률·커버리지 현장 체크

순수 로직(그리드·검출률·주변부·판정)만 단위 테스트. cv2 AprilTag 검출은 `main`에서 수동 검증.

**Files:**
- Create: `src/econ_camera_ros/econ_camera_ros/calib_coverage.py`
- Test: `src/econ_camera_ros/test/test_calib_coverage.py`
- Modify: `src/econ_camera_ros/setup.py`

**Interfaces:**
- Consumes: `bag_extract` 산출물(`frame_NNNNNN/cam{d}.jpg`), 해상도 1280x720.
- Produces:
  - `coverage_grid(corners, width, height, grid=8) -> list[list[int]]`
  - `detection_rate(per_frame_counts: list[int]) -> float`
  - `periphery_filled(grid: list[list[int]]) -> float`
  - `evaluate(per_frame_counts, corners, width, height, grid=8, min_rate=0.5, min_periphery=0.5) -> dict` — `{detection_rate, periphery_filled, grid, passed}`
  - `detect_corners(image_path) -> list[tuple[float,float]]` — cv2.aruco(수동).

- [ ] **Step 1: 실패하는 테스트 작성**

`src/econ_camera_ros/test/test_calib_coverage.py`:
```python
from econ_camera_ros import calib_coverage as cc


def test_coverage_grid_bins_corners():
    # 1280x720, grid=8 → 셀 폭 160, 높이 90. (10,10)=셀(0,0), (1270,710)=셀(7,7)
    corners = [(10, 10), (1270, 710), (20, 20)]
    g = cc.coverage_grid(corners, 1280, 720, grid=8)
    assert g[0][0] == 2   # (10,10),(20,20)
    assert g[7][7] == 1   # (1270,710)
    assert sum(sum(row) for row in g) == 3


def test_detection_rate():
    assert cc.detection_rate([0, 4, 4, 0]) == 0.5
    assert cc.detection_rate([]) == 0.0


def test_periphery_filled_all_border_cells():
    # 3x3에서 테두리 셀은 8개. 그 중 4개만 채움 → 0.5
    grid = [[1, 0, 1],
            [0, 5, 0],
            [1, 0, 1]]
    assert cc.periphery_filled(grid) == 0.5


def test_evaluate_pass_and_fail():
    good = cc.evaluate([4, 4, 4, 4],
                       [(x, y) for x in (10, 1270) for y in (10, 710)],
                       1280, 720, grid=8, min_rate=0.5, min_periphery=0.1)
    assert good["passed"] is True
    bad = cc.evaluate([0, 0, 1], [(640, 360)], 1280, 720,
                      min_rate=0.5, min_periphery=0.5)
    assert bad["passed"] is False
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd src/econ_camera_ros && python3 -m pytest test/test_calib_coverage.py -v`
Expected: FAIL — `ModuleNotFoundError: econ_camera_ros.calib_coverage`.

- [ ] **Step 3: 최소 구현**

`src/econ_camera_ros/econ_camera_ros/calib_coverage.py`:
```python
"""캘리브 녹화 직후 현장에서 AprilGrid 커버리지를 즉시 점검하는 도구.

Kalibr는 완전 offline이라 녹화 중 피드백이 없다(검출 실패 프레임은 조용히 폐기).
이 도구는 추출 프레임에 AprilTag 검출을 1회 돌려 카메라별 검출률과 코너 커버리지
히트맵을 뽑아, 기준 미달이면 그 자리에서 재촬영하도록 돕는다.

주의: 여기 검출기(cv2.aruco 36h11)는 Kalibr 검출기와 동일하지 않다. 극단 주변부에서
민감도가 다를 수 있어, 본 도구는 '심각한 커버리지 공백을 잡는 게이트'이지 Kalibr 등가 검증이
아니다(대안 검출기: pupil-apriltags). 순수 로직은 cv2 없이 테스트 가능.

실행:
    python3 -m econ_camera_ros.calib_coverage <extracted_dir>
    ros2 run econ_camera_ros calib_coverage <extracted_dir>
"""

import argparse
import glob
import os


def coverage_grid(corners, width, height, grid=8):
    """코너 픽셀들을 grid x grid 셀에 비닝한 카운트 2D 리스트."""
    g = [[0] * grid for _ in range(grid)]
    for (x, y) in corners:
        cx = min(int(x / width * grid), grid - 1)
        cy = min(int(y / height * grid), grid - 1)
        if 0 <= cx < grid and 0 <= cy < grid:
            g[cy][cx] += 1
    return g


def detection_rate(per_frame_counts):
    """검출 코너가 1개 이상인 프레임 비율."""
    if not per_frame_counts:
        return 0.0
    return sum(1 for c in per_frame_counts if c > 0) / len(per_frame_counts)


def periphery_filled(grid):
    """테두리 셀 중 최소 1개 이상 채워진 비율(주변부 커버리지 지표)."""
    n = len(grid)
    border = [grid[i][j] for i in range(n) for j in range(n)
              if i in (0, n - 1) or j in (0, n - 1)]
    if not border:
        return 0.0
    return sum(1 for c in border if c > 0) / len(border)


def evaluate(per_frame_counts, corners, width, height, grid=8,
             min_rate=0.5, min_periphery=0.5):
    """검출률·주변부 커버리지로 카메라 1대 통과 여부 판정."""
    rate = detection_rate(per_frame_counts)
    g = coverage_grid(corners, width, height, grid)
    periph = periphery_filled(g)
    return {
        "detection_rate": rate,
        "periphery_filled": periph,
        "grid": g,
        "passed": rate >= min_rate and periph >= min_periphery,
    }


def detect_corners(image_path):
    """cv2.aruco(AprilTag 36h11)로 검출된 모든 마커 코너 픽셀 좌표(지연 임포트)."""
    import cv2

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(image_path)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    marker_corners, ids, _ = detector.detectMarkers(img)
    pts = []
    if ids is not None:
        for c in marker_corners:
            for (x, y) in c.reshape(-1, 2):
                pts.append((float(x), float(y)))
    return pts


def main():
    p = argparse.ArgumentParser(description="AprilGrid 커버리지 현장 체크")
    p.add_argument("extracted", help="bag_extract 출력 디렉터리")
    p.add_argument("--devs", default="0,1,2,3", help="카메라 인덱스(기본 0,1,2,3)")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--min-rate", type=float, default=0.5)
    p.add_argument("--min-periphery", type=float, default=0.5)
    a = p.parse_args()

    devs = [int(x) for x in a.devs.split(",")]
    all_pass = True
    for d in devs:
        frames = sorted(glob.glob(os.path.join(a.extracted, "frame_*", f"cam{d}.jpg")))
        per_frame, corners = [], []
        for fp in frames:
            pts = detect_corners(fp)
            per_frame.append(len(pts))
            corners.extend(pts)
        r = evaluate(per_frame, corners, a.width, a.height,
                     min_rate=a.min_rate, min_periphery=a.min_periphery)
        mark = "PASS" if r["passed"] else "FAIL"
        all_pass = all_pass and r["passed"]
        print(f"[cam{d}] {mark}  검출률={r['detection_rate']:.2f}  "
              f"주변부={r['periphery_filled']:.2f}  프레임={len(frames)}")
        for row in r["grid"]:
            print("   " + " ".join(f"{c:3d}" for c in row))
    print("전체:", "PASS" if all_pass else "FAIL — 재촬영 권장")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd src/econ_camera_ros && python3 -m pytest test/test_calib_coverage.py -v`
Expected: 4개 PASS.

- [ ] **Step 5: 엔트리포인트 추가**

`src/econ_camera_ros/setup.py` `console_scripts` 에 추가:
```python
            'calib_coverage = econ_camera_ros.calib_coverage:main',
```

- [ ] **Step 6: Commit**

```bash
git add src/econ_camera_ros/econ_camera_ros/calib_coverage.py \
        src/econ_camera_ros/test/test_calib_coverage.py \
        src/econ_camera_ros/setup.py
git commit -m "feat(calib): calib_coverage — 현장 검출률·커버리지 히트맵 체크"
```

---

### Task 4: `calib_convert` — Kalibr camchain → 프로젝트 `calib.yaml`

Kalibr가 출력한 camchain(모델·intrinsic·상대자세)을 프로젝트 사이드카 스키마로 변환하고,
Kalibr 리포트의 재투영 RMS(검증 A)를 함께 기록한다. 순수 변환 로직을 단위 테스트한다.

**Files:**
- Create: `src/econ_camera_ros/econ_camera_ros/calib_convert.py`
- Test: `src/econ_camera_ros/test/test_calib_convert.py`
- Modify: `src/econ_camera_ros/setup.py`

**Interfaces:**
- Consumes: Kalibr camchain(파싱된 dict; 각 camN에 `camera_model`,`intrinsics`,`distortion_model`,`distortion_coeffs`,`resolution`, cam1부터 `T_cn_cnm1`(4x4)), 선택 재투영 RMS dict(`{cam0: float, ...}`).
- Produces:
  - `camchain_to_calib(camchain: dict, model_chosen: str, reproj_rms: dict|None=None) -> dict`
    반환 구조: `{model_chosen, cameras{camN:{camera_model,intrinsics,distortion_model,distortion_coeffs,resolution}}, extrinsics{T_camN_cam0: 4x4}, verification{reproj_rms_px}}`.
    `T_camN_cam0` = cam0→camN 점 변환(누적곱), cam0은 항등.

- [ ] **Step 1: 실패하는 테스트 작성**

`src/econ_camera_ros/test/test_calib_convert.py`:
```python
from econ_camera_ros import calib_convert as cvt


def _cam(model, intr, topic, t=None):
    c = {"camera_model": model, "intrinsics": list(intr),
         "distortion_model": "none", "distortion_coeffs": [],
         "resolution": [1280, 720], "rostopic": topic}
    if t is not None:
        c["T_cn_cnm1"] = t
    return c


def test_camchain_to_calib_basic_and_cumulative_extrinsics():
    I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    t01 = [[1, 0, 0, 0.1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    t12 = [[1, 0, 0, 0.2], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    camchain = {
        "cam0": _cam("ds", [0.1, 0.6, 700, 700, 640, 360], "/cam0/image_raw"),
        "cam1": _cam("ds", [0.1, 0.6, 701, 701, 641, 361], "/cam1/image_raw", t01),
        "cam2": _cam("ds", [0.1, 0.6, 702, 702, 642, 362], "/cam2/image_raw", t12),
    }
    out = cvt.camchain_to_calib(camchain, "ds", {"cam0": 0.30, "cam1": 0.35, "cam2": 0.33})

    assert out["model_chosen"] == "ds"
    assert out["cameras"]["cam1"]["intrinsics"] == [0.1, 0.6, 701, 701, 641, 361]
    assert out["cameras"]["cam0"]["camera_model"] == "ds"
    # cam0 = 항등, cam1 = t01, cam2 = t12 @ t01 → x 평행이동 0.3
    assert out["extrinsics"]["T_cam0_cam0"] == I
    assert out["extrinsics"]["T_cam1_cam0"] == t01
    assert out["extrinsics"]["T_cam2_cam0"][0][3] == 0.30000000000000004 or \
           abs(out["extrinsics"]["T_cam2_cam0"][0][3] - 0.3) < 1e-9
    assert out["verification"]["reproj_rms_px"]["cam2"] == 0.33


def test_camchain_to_calib_without_rms():
    camchain = {"cam0": _cam("eucm", [0.6, 1.0, 700, 700, 640, 360], "/cam0/image_raw")}
    out = cvt.camchain_to_calib(camchain, "eucm")
    assert out["verification"]["reproj_rms_px"] == {}
    assert out["extrinsics"]["T_cam0_cam0"][3][3] == 1
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd src/econ_camera_ros && python3 -m pytest test/test_calib_convert.py -v`
Expected: FAIL — `ModuleNotFoundError: econ_camera_ros.calib_convert`.

- [ ] **Step 3: 최소 구현**

`src/econ_camera_ros/econ_camera_ros/calib_convert.py`:
```python
"""Kalibr camchain(yaml) → 프로젝트 calib.yaml 사이드카.

Kalibr가 출력한 camera_model·intrinsics·상대자세(T_cn_cnm1 선형 체인)를 읽어,
카메라별 intrinsic과 cam0 기준 누적 extrinsic(T_camN_cam0)으로 정리한다.
검증(A)은 Kalibr 리포트의 재투영 RMS를 그대로 기록한다.

순수 변환(camchain_to_calib)만 단위 테스트하고, yaml 입출력은 main에서 수동 검증.

실행:
    python3 -m econ_camera_ros.calib_convert camchain.yaml --model ds -o calib.yaml \
        --rms cam0=0.31 cam1=0.29 cam2=0.33 cam3=0.30
"""

import argparse


def _mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def camchain_to_calib(camchain, model_chosen, reproj_rms=None):
    """Kalibr camchain dict → 프로젝트 calib dict."""
    cams = sorted(camchain)
    cameras, extrinsics = {}, {}
    cum = [[1 if i == j else 0 for j in range(4)] for i in range(4)]  # T_cam0_cam0 = I
    for idx, name in enumerate(cams):
        c = camchain[name]
        cameras[name] = {
            "camera_model": c["camera_model"],
            "intrinsics": list(c["intrinsics"]),
            "distortion_model": c.get("distortion_model", "none"),
            "distortion_coeffs": list(c.get("distortion_coeffs", [])),
            "resolution": list(c["resolution"]),
        }
        if idx > 0:
            cum = _mat_mul(c["T_cn_cnm1"], cum)  # T_cn_c0 = T_cn_cnm1 @ T_cnm1_c0
        extrinsics[f"T_{name}_cam0"] = cum
    return {
        "model_chosen": model_chosen,
        "cameras": cameras,
        "extrinsics": extrinsics,
        "verification": {"reproj_rms_px": dict(reproj_rms or {})},
    }


def main():
    import yaml

    p = argparse.ArgumentParser(description="Kalibr camchain → calib.yaml")
    p.add_argument("camchain", help="Kalibr가 출력한 *-camchain.yaml")
    p.add_argument("--model", required=True, help="채택 모델 라벨(ds/eucm)")
    p.add_argument("-o", "--out", default="calib.yaml")
    p.add_argument("--rms", nargs="*", default=[],
                   help="Kalibr 리포트 재투영 RMS: camN=값 ... (예: cam0=0.31)")
    a = p.parse_args()

    with open(a.camchain) as f:
        camchain = yaml.safe_load(f)
    rms = {}
    for kv in a.rms:
        k, v = kv.split("=")
        rms[k] = float(v)
    calib = camchain_to_calib(camchain, a.model, rms)
    with open(a.out, "w") as f:
        yaml.safe_dump(calib, f, sort_keys=False)
    print(f"calib 작성 → {a.out} (model={a.model}, cams={list(calib['cameras'])})")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd src/econ_camera_ros && python3 -m pytest test/test_calib_convert.py -v`
Expected: 2개 PASS.

- [ ] **Step 5: 엔트리포인트 추가 + 전체 테스트**

`src/econ_camera_ros/setup.py` `console_scripts` 에 추가:
```python
            'calib_convert = econ_camera_ros.calib_convert:main',
```
Run: `cd src/econ_camera_ros && python3 -m pytest test/ -v`
Expected: 기존 11개 + 신규 10개 = 전부 PASS.

- [ ] **Step 6: Commit**

```bash
git add src/econ_camera_ros/econ_camera_ros/calib_convert.py \
        src/econ_camera_ros/test/test_calib_convert.py \
        src/econ_camera_ros/setup.py
git commit -m "feat(calib): calib_convert — Kalibr camchain → calib.yaml(+재투영 RMS)"
```

---

### Task 5: 실기 통합 드라이런 (녹화→체크→브릿지→Kalibr→calib.yaml) + 실행 헬퍼

4대 하드웨어와 `kalibr:arm64` 이미지가 필요한 **수동 통합 검증**. pytest 불가.
여기서 실제로 파이프라인을 한 번 관통시키고, 그 과정에서 재사용할 `run_kalibr.sh`를 확정한다.

**Files:**
- Create: `calibration/run_kalibr.sh`

**Interfaces:**
- Consumes: Task 1(`kalibr:arm64`,`aprilgrid.yaml`), Task 2(`kalibr_bridge`), Task 3(`calib_coverage`), Task 4(`calib_convert`), 기존 `record.launch.py`·`bag_extract`.
- Produces: 검증된 `calibration/run_kalibr.sh`, 실제 `calib.yaml` 1벌(드라이런 산출), 실행 순서 확정(→ Task 6 문서화).

- [ ] **Step 1: 캘리브 시퀀스 녹화 (기존 파이프라인 재사용)**

AprilGrid 판을 들고 4대 화면 곳곳(특히 주변부)을 천천히 커버하며 녹화:
```bash
# ws 빌드/소싱은 기존 방식(colcon build; source install/setup.bash) 그대로
ros2 launch econ_camera_ros record.launch.py
# 30초~1분 웨이브 후 Ctrl-C. 생성된 bag 디렉터리 경로 확인.
```
Expected: mcap bag 디렉터리 생성(카메라 4토픽 기록).

- [ ] **Step 2: 동기 세트 추출 + 현장 커버리지 체크**

```bash
ros2 run econ_camera_ros bag_extract <bag_dir> -o extracted
ros2 run econ_camera_ros calib_coverage extracted
```
Expected: 카메라별 `PASS/FAIL` + 히트맵 출력. **FAIL이면 Step 1로 돌아가 재촬영**(주변부가 비었으면 판을 가장자리까지 더 이동).

- [ ] **Step 3: Kalibr 데이터셋 폴더 생성 (브릿지)**

```bash
ros2 run econ_camera_ros kalibr_bridge extracted -o dataset --rate 4.0
ls dataset/cam0 | head   # <ns>.png 파일들 확인
```
Expected: `dataset/cam{0..3}/<ns>.png` 생성, 4 폴더 파일 수 동일.

- [ ] **Step 4: 실행 헬퍼 스크립트 작성**

`calibration/run_kalibr.sh`:
```bash
#!/usr/bin/env bash
# dataset/ + aprilgrid.yaml 을 컨테이너에 마운트해 bag 생성 후 ds/eucm 2회 실행.
# 사용: bash calibration/run_kalibr.sh <DATA_DIR>
#   DATA_DIR 안에 dataset/(cam0..3/<ns>.png) 와 aprilgrid.yaml 이 있어야 함.
set -euo pipefail

DATA_DIR="$(realpath "${1:?usage: run_kalibr.sh <DATA_DIR>}")"
WS="/catkin_ws"   # 컨테이너 내 catkin workspace (calibration/README.md 확인값)
TOPICS="/cam0/image_raw /cam1/image_raw /cam2/image_raw /cam3/image_raw"

docker run --rm -v "$DATA_DIR:/data" kalibr:arm64 bash -lc "
  source $WS/devel/setup.bash
  rosrun kalibr kalibr_bagcreater --folder /data/dataset --output-bag /data/calib.bag

  for M in ds eucm; do
    rosrun kalibr kalibr_calibrate_cameras \
      --bag /data/calib.bag \
      --target /data/aprilgrid.yaml \
      --models \$M \$M \$M \$M \
      --topics $TOPICS \
      --approx-sync 0.001 \
      --dont-show-report
    # 모델별 산출물이 덮이지 않게 접미사 부여
    for f in /data/*-camchain*.yaml /data/*-results*.txt /data/*-report*.pdf; do
      [ -e \"\$f\" ] && mv \"\$f\" \"\${f%.*}-\$M.\${f##*.}\"
    done
  done
"
echo "DONE: /data 에 ds/eucm camchain·results·report 생성."
```

- [ ] **Step 5: Kalibr 실행 (컨테이너)**

```bash
cp calibration/aprilgrid.yaml ./          # dataset 과 같은 DATA_DIR 에 위치
DATA_DIR=$(pwd)                            # dataset/ 와 aprilgrid.yaml 이 있는 곳
bash calibration/run_kalibr.sh "$DATA_DIR"
ls *-camchain*-ds.yaml *-camchain*-eucm.yaml *-report*-ds.pdf 2>/dev/null
```
Expected: ds/eucm 각각 `*-camchain-*.yaml`, `*-results-*.txt`, `*-report-*.pdf` 생성.
- `kalibr_bagcreater` 가 `.png` 을 못 읽으면(글로브 패턴 문제) 브릿지 출력 확장자/이 스크립트의 폴더명을 확인. workspace 경로 오류면 README 확인값으로 `WS` 수정.

- [ ] **Step 6: 리포트 비교로 모델 채택 (검증 A)**

각 `*-results-*.txt`(또는 report PDF)에서 **카메라별 재투영 RMS**와 잔차 패턴을 확인:
- RMS가 더 작고 잔차 산포가 무작위(방사형 패턴 없음)인 모델 채택(§6.1).
- 채택 모델(예: ds)의 `*-camchain-ds.yaml` 을 사용. RMS 값을 메모.

- [ ] **Step 7: calib.yaml 생성**

```bash
ros2 run econ_camera_ros calib_convert <채택>-camchain-ds.yaml --model ds -o calib.yaml \
  --rms cam0=<v> cam1=<v> cam2=<v> cam3=<v>
cat calib.yaml
```
Expected: `cameras`(4대 intrinsic), `extrinsics`(T_camN_cam0), `verification.reproj_rms_px` 포함된 `calib.yaml`.

- [ ] **Step 8: 정성 검증(육안) + 헬퍼 커밋**

- report PDF의 재투영/잔차 플롯을 눈으로 확인(카메라 간 쌍별 오차 포함). 주변부 왜곡이 잘 잡혔는지 점검.
- 드라이런에서 얻은 실제 명령/경로/RMS 임계 감각을 Task 6 문서에 반영할 수 있도록 메모.

```bash
git add calibration/run_kalibr.sh
git commit -m "feat(calib): run_kalibr.sh — bagcreater + ds/eucm 2회 실행 헬퍼(드라이런 검증)"
```

> 산출된 `calib.bag`, `dataset/`, `*-report-*.pdf`, `calib.yaml` 등 대용량/데이터 산출물은
> 커밋하지 않는다(필요 시 `.gitignore` 확인).

---

### Task 6: 운영 가이드 문서 (`docs/CALIBRATION.md`)

Task 5에서 실제로 관통시킨 절차를 촬영법 + 실행 순서 how-to로 정리한다(드라이런으로 검증된 명령만 기재).

**Files:**
- Create: `docs/CALIBRATION.md`
- Modify: `CLAUDE.md`(상세 문서 링크 목록에 한 줄 추가), `docs/USAGE.md`(캘리브 항목 있으면 상호링크 — 없으면 스킵)

**Interfaces:**
- Consumes: Task 1~5 전부(명령·경로·임계값).
- Produces: 사람이 따라할 수 있는 운영 가이드.

- [ ] **Step 1: 가이드 작성**

`docs/CALIBRATION.md` — 아래 뼈대에 Task 5에서 확정된 실제 명령/경로를 채운다:
```markdown
# 카메라 캘리브레이션 운영 가이드 (어안 4대)

설계: docs/superpowers/specs/2026-07-18-camera-calibration-and-verification-design.md
계획: docs/superpowers/plans/2026-07-18-camera-calibration-and-verification.md

## 0. 사전 준비 (1회)
- Docker 이미지 빌드: `bash calibration/build_kalibr_arm64.sh` → `kalibr:arm64`
- 타깃: AprilGrid(7x5, 4cm, 간격 1cm) = `calibration/aprilgrid.yaml`

## 1. 촬영 (무엇을 어떻게)
- 리그를 완전히 고정한 뒤, **세션 앞뒤로 1회씩** 캘리브 시퀀스를 찍어 드리프트를 감지한다.
- AprilGrid 판을 **천천히·부드럽게** 움직이며 4대 화면을 채운다. **극단 주변부/코너까지** 반드시 커버.
- 학습(캘리브용)과 검증(held-out)을 **분리 촬영**.
- 명령: `ros2 launch econ_camera_ros record.launch.py` → 30초~1분 → Ctrl-C.

## 2. 현장 즉시 체크 (블라인드 녹화 방지)
    ros2 run econ_camera_ros bag_extract <bag_dir> -o extracted
    ros2 run econ_camera_ros calib_coverage extracted
- 카메라별 PASS 확인. FAIL(특히 주변부 공백)이면 1로 돌아가 재촬영.

## 3. Kalibr 입력 준비 (브릿지)
    ros2 run econ_camera_ros kalibr_bridge extracted -o dataset --rate 4.0
    cp calibration/aprilgrid.yaml .        # dataset/ 과 같은 폴더

## 4. Kalibr 실행 (ds/eucm 둘 다)
    bash calibration/run_kalibr.sh "$(pwd)"
- 산출: `*-camchain-{ds,eucm}.yaml`, `*-results-*.txt`, `*-report-*.pdf`

## 5. 검증 (A: Kalibr 리포트) + 모델 채택
- results/report에서 **카메라별 재투영 RMS**(sub-pixel 지향)와 **잔차 산포 무작위성** 확인.
- 방사형/체계적 패턴 = 모델 부적합 → 다른 모델 채택 또는 KB 검토.
- 쌍별 재투영 오차로 카메라 간 extrinsic 품질 확인.
- (정성) 왜곡 보정 후 직선이 직선으로 펴지는지 육안(특히 주변부).

## 6. calib.yaml 생성
    ros2 run econ_camera_ros calib_convert <채택>-camchain-<model>.yaml \
        --model <model> -o calib.yaml --rms cam0=.. cam1=.. cam2=.. cam3=..
- 데이터셋 옆에 `calib.yaml` 배치. (cam↔LiDAR extrinsic 은 L2 도착 후 추가.)

## 문제해결
- Docker 권한: `sudo usermod -aG docker $USER` 후 재로그인.
- workspace 경로 오류: `calibration/README.md`의 확인값으로 `run_kalibr.sh`의 `WS` 수정.
- bagcreater가 .png 미인식: 브릿지 출력 확장자 확인.
```

- [ ] **Step 2: CLAUDE.md 상세 문서 링크 추가**

`CLAUDE.md` 의 "## 상세 문서" 목록에 한 줄 추가:
```markdown
- **캘리브레이션 가이드**: `docs/CALIBRATION.md` (촬영법·Kalibr 실행·검증·calib.yaml)
```

- [ ] **Step 3: 링크 유효성 확인**

Run: `ls docs/CALIBRATION.md docs/superpowers/specs/2026-07-18-camera-calibration-and-verification-design.md docs/superpowers/plans/2026-07-18-camera-calibration-and-verification.md`
Expected: 세 파일 모두 존재.

- [ ] **Step 4: Commit**

```bash
git add docs/CALIBRATION.md CLAUDE.md
git commit -m "docs(calib): 촬영·Kalibr 실행·검증 운영 가이드(CALIBRATION.md)"
```

---

## Self-Review

**1. Spec coverage:**
- §2 범위(A intrinsic, B cam-cam extrinsic, C 나중) → Task 4/5(A·B 산출), C는 명시적 범위 밖. ✓
- §3.1 arm64 Docker + kalibr_bagcreater 브릿지 → Task 1, Task 2. ✓
- §3.2 ds/eucm 비교 → Task 5 Step 5-6. ✓
- §3.3 aprilgrid.yaml(7x5/0.04/0.25) → Task 1 Step 1. ✓
- §3.4 실행 커맨드(4×모델/토픽/approx-sync/dont-show-report) → Task 5 Step 4 `run_kalibr.sh`. ✓
- §4 촬영 절차(주변부·다운샘플·앞뒤·held-out) → Task 2(다운샘플), Task 6(촬영법). ✓
- §5 현장 커버리지 체크(A) → Task 3. ✓
- §6 검증 A(Kalibr 리포트)·정성 → Task 5 Step 6/8, Task 6 §5. B는 §9 나중. ✓
- §7 calib.yaml 사이드카 → Task 4. ✓
- §8 구현 항목 1~7 → Task 1~6에 각각 대응(운영 가이드=Task 6). ✓

**2. Placeholder scan:** TDD 태스크는 실제 테스트/코드 포함. 수동 태스크(1,5,6)는 하드웨어/외부 의존이라 pytest 불가함을 명시하고 관측 가능한 검증(빌드 성공·PASS·파일 생성)을 지정. `<bag_dir>`,`<v>` 등은 런타임 실제값 placeholder(플랜 결함 아님). ✓

**3. Type consistency:** `select_by_rate`,`stamp_to_ns`,`parse_set_stamps`,`build_dataset`(Task2) / `coverage_grid`,`detection_rate`,`periphery_filled`,`evaluate`,`detect_corners`(Task3) / `camchain_to_calib`(Task4) — 테스트·구현·Interfaces 블록 시그니처 일치. calib.yaml 키(`model_chosen`,`cameras`,`extrinsics`,`verification.reproj_rms_px`)는 Task4 정의를 Task5/6에서 그대로 사용. ✓
