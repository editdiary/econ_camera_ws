# BEV 슬래브 라벨 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `map_clean.pcd` + `trajectory.tum` 에서 키프레임별 BEV occupancy 라벨과 visibility 라벨을 생성한다.

**Architecture:** pose 로 body 프레임 3D crop → 로봇 높이 슬래브(하위 1% z 부터 0.8m) → 2D 기둥 누적 count≥3 으로 occupancy → ego 셀 2D 360° raycast ∧ 카메라 관측가능성으로 visibility. 순수 로직(`slab_label.py`)·IO(`slab_io.py`)·렌더(`slab_render.py`)·CLI(`generate_slab.py`) 로 분리한다. 기존 `bev_label.py`·`generate.py` 는 수정하지 않고 재사용만 한다.

**Tech Stack:** Python 3.10, numpy, scipy, opencv-python(cv2), PIL, pytest. ROS2 불필요(bag 을 읽지 않는다).

설계 근거·실측값 전체: `docs/superpowers/specs/2026-08-10-bev-slab-label-design.md`

## Global Constraints

- **데이터 출처**: `data/sj_bags/260722/maps_selfmask/<name>_mapping/` 만 사용. 구 `data/sj_bags/260722/maps/` 는 쓰지 않는다.
- **좌표 규약**: ego=body=LiDAR, x=전방·y=좌·z=상. `row=(XF−x)/RES`, `col=(YH−y)/RES`. 기존 `bev_label.rc_of` 와 동일해야 한다.
- **라벨 화소값**: `occupancy.png` 는 `0=obstacle, 1=drivable`. `visibility.png` 는 `0=unseen, 1=visible`. 기존 `label.png` 와 같은 약속을 유지한다.
- **RES 는 0.05 고정**. `XF/XR/YH` 는 RES 의 정수배여야 한다(`BevSpec.__post_init__` 가 검증).
- **기본값**: `XF=4.0 XR=2.0 YH=3.0`(=120×120) `THICK=0.8` `PCT=1.0` `MIN_PTS=3` `RAY_STEP=0.25` `GROUND_OFFSET=0.87` `KF_STEP=0.4` `SELF_MASK_CLASSES=table` `SELF_BOX near=0.4 far=2.1 yh=0.7`
- **복붙 금지, import 로 재사용**: `mapping/pcd_denoise.read_pcd_raw`·`write_pcd_raw`, `calibration/cam_lidar/cloud_io.load_tum`·`pose_at`, `calibration/cam_lidar/chain.se3_inv`·`transform`·`project`, `calibration/verify/ds_model.load_rig`, `calibration/cam_lidar/calib_io.load_T_front_lidar`, `calibration/bev_autolabel/bev_io.load_stamps`, `calibration/bev_autolabel/bev_label.BevSpec`·`rc_of`·`cell_centers`·`raycast_visible`·`select_keyframes`.
- **numpy/scipy 핀 주의**: 시스템 scipy 가 numpy<1.25 를 요구한다. `pip install` 로 numpy≥2 를 끌어오는 패키지(opencv-python≥4.10 등)를 설치하면 scipy 가 깨진다. 새 의존성을 추가하지 않는다.
- **테스트 실행**: `cd calibration/bev_autolabel && python3 -m pytest test_slab_label.py test_slab_render.py -q`
- **브랜치**: `feat/bev-slab-label`. 병합·푸시는 사용자가 한다 — 커밋까지만.

---

## File Structure

| 파일 | 책임 |
|---|---|
| `calibration/bev_autolabel/slab_label.py` (신규) | 순수 로직. crop 마스크, z_ref, 슬래브 마스크, occupancy 카운트/임계, 비네팅 검출, 카메라 관측가능성, 두 라벨 조립. 파일·하드웨어 접근 없음 |
| `calibration/bev_autolabel/slab_io.py` (신규) | 8필드 보존 PCD 읽기/부분쓰기, self 마스크 PNG 로드, 프레임 표본 로드 |
| `calibration/bev_autolabel/slab_render.py` (신규) | `occupancy.png`·`visibility.png` 팔레트 저장, 4색 `review.png` |
| `calibration/bev_autolabel/generate_slab.py` (신규) | CLI. 키프레임 순회 + 샘플 폴더 산출 + 요약 로그 |
| `calibration/bev_autolabel/slab_sheet.py` (신규) | 사람이 눈으로 볼 검수 산출물: 궤적 전체 컨택트시트·채널 비교·샘플별 통계(`center_reach` 포함) |
| `calibration/bev_autolabel/test_slab_label.py` (신규) | 순수 로직 테스트 |
| `calibration/bev_autolabel/test_slab_render.py` (신규) | 렌더 테스트 |
| `docs/BEV_AUTOLABEL.md` (수정) | §B 신설: 슬래브 라벨 실행 가이드 |
| `CLAUDE.md` (수정) | BEV auto-label 항목에 새 경로 추가 |

---

### Task 1: crop 과 슬래브 (순수 로직)

**Files:**
- Create: `calibration/bev_autolabel/slab_label.py`
- Test: `calibration/bev_autolabel/test_slab_label.py`

**Interfaces:**
- Consumes: `bev_label.BevSpec`(필드 `XF/XR/YH/RES`, 프로퍼티 `NX/NY/R_EGO/C_EGO`), `bev_label.rc_of(x, y, spec)`, `bev_label.cell_centers(spec)`
- Produces:
  - `crop_mask(P_body: np.ndarray[(N,3)], spec: BevSpec) -> np.ndarray[(N,), bool]`
  - `ref_z(z: np.ndarray[(M,)], pct: float = 1.0) -> float`
  - `slab_mask(P_crop: np.ndarray[(M,3)], z_ref: float, thick: float = 0.8) -> np.ndarray[(M,), bool]`

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`calibration/bev_autolabel/test_slab_label.py` 를 새로 만든다:

```python
"""슬래브 라벨 순수 로직 테스트. 하드웨어·파일 불필요."""
import pathlib
import sys

import numpy as np
import pytest

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
from bev_label import BevSpec                                   # noqa: E402
import slab_label as sl                                         # noqa: E402


def test_crop_mask_keeps_inside_box():
    spec = BevSpec(XF=4.0, XR=2.0, YH=3.0)
    P = np.array([
        [0.0, 0.0, 0.0],       # 중앙 → 포함
        [4.0, 0.0, 99.0],      # x 상한 경계 → 포함 (z 는 무제한)
        [-2.0, 0.0, -99.0],    # x 하한 경계 → 포함
        [0.0, 3.0, 0.0],       # y 상한 경계 → 포함
        [0.0, -3.0, 0.0],      # y 하한 경계 → 포함
        [4.01, 0.0, 0.0],      # x 초과 → 제외
        [-2.01, 0.0, 0.0],     # x 미달 → 제외
        [0.0, 3.01, 0.0],      # y 초과 → 제외
    ])
    m = sl.crop_mask(P, spec)
    assert m.tolist() == [True, True, True, True, True, False, False, False]


def test_crop_mask_ignores_z_entirely():
    spec = BevSpec(XF=4.0, XR=2.0, YH=3.0)
    P = np.array([[1.0, 1.0, -1000.0], [1.0, 1.0, 1000.0]])
    assert sl.crop_mask(P, spec).all()


def test_crop_mask_empty_input():
    spec = BevSpec(XF=4.0, XR=2.0, YH=3.0)
    m = sl.crop_mask(np.empty((0, 3)), spec)
    assert m.shape == (0,) and m.dtype == bool


def test_ref_z_is_low_percentile_not_min():
    z = np.concatenate([[-5.0], np.linspace(0.0, 1.0, 100)])   # -5 는 이상치 1개
    r = sl.ref_z(z, pct=1.0)
    assert -5.0 < r < 0.05, r          # 이상치에 끌려가지 않는다


def test_ref_z_pct_zero_is_min():
    z = np.array([3.0, 1.0, 2.0])
    assert sl.ref_z(z, pct=0.0) == pytest.approx(1.0)


def test_slab_mask_band_boundaries_inclusive():
    P = np.array([
        [0.0, 0.0, 0.999],   # z_ref 바로 아래 → 제외
        [0.0, 0.0, 1.0],     # z_ref → 포함
        [0.0, 0.0, 1.8],     # z_ref+thick → 포함
        [0.0, 0.0, 1.801],   # 초과 → 제외
    ])
    m = sl.slab_mask(P, z_ref=1.0, thick=0.8)
    assert m.tolist() == [False, True, True, False]


def test_slab_mask_empty_input():
    m = sl.slab_mask(np.empty((0, 3)), z_ref=0.0, thick=0.8)
    assert m.shape == (0,) and m.dtype == bool
```

- [ ] **Step 2: 실패를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_label.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'slab_label'`

- [ ] **Step 3: 최소 구현을 쓴다**

`calibration/bev_autolabel/slab_label.py` 를 새로 만든다:

```python
"""BEV 슬래브 라벨 순수 로직. 하드웨어·파일 불필요.

좌표계: ego=body=LiDAR, x=전방·y=좌·z=상. BEV row=(XF-x)/RES, col=(YH-y)/RES.
라벨: occupancy 0=obstacle 1=drivable, visibility 0=unseen 1=visible.

LiDAR 가 바닥을 못 보고 자기 수평면 근처부터 돔으로 수집하므로, crop 안 최저 z 가
LiDAR 수평면(실제 지상 약 0.87m)이다. 슬래브 [z_ref, z_ref+thick] 은 로봇이 통과해야
하는 높이 구간을 뜻한다. 근거·실측: docs/superpowers/specs/2026-08-10-bev-slab-label-design.md
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))


def crop_mask(P_body, spec):
    """body 프레임 점 (N,3) → BEV 박스 안인지 (N,) bool. z 는 제한하지 않는다.

    z 를 여기서 자르지 않는 이유: z_ref 를 crop 안에서 구해야 전역 드리프트에 면역이 된다.
    """
    P = np.asarray(P_body, float)
    if len(P) == 0:
        return np.zeros(0, bool)
    return ((P[:, 0] <= spec.XF) & (P[:, 0] >= -spec.XR)
            & (np.abs(P[:, 1]) <= spec.YH))


def ref_z(z, pct=1.0):
    """슬래브 하단 = z 의 하위 pct 퍼센타일. min 이 아닌 이유는 이상치 flyer 때문."""
    return float(np.percentile(np.asarray(z, float), pct))


def slab_mask(P_crop, z_ref, thick=0.8):
    """crop 점 (M,3) → [z_ref, z_ref+thick] 밴드 안인지 (M,) bool. 양쪽 경계 포함."""
    P = np.asarray(P_crop, float)
    if len(P) == 0:
        return np.zeros(0, bool)
    return (P[:, 2] >= z_ref) & (P[:, 2] <= z_ref + thick)
```

- [ ] **Step 4: 테스트 통과를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_label.py -q`
Expected: PASS (7 passed)

- [ ] **Step 5: 커밋**

```bash
git add calibration/bev_autolabel/slab_label.py calibration/bev_autolabel/test_slab_label.py
git commit -m "feat(bev): 슬래브 라벨 crop·z_ref·슬래브 밴드 순수 로직"
```

---

### Task 2: occupancy 와 raycast visibility (순수 로직)

**Files:**
- Modify: `calibration/bev_autolabel/slab_label.py` (함수 추가)
- Modify: `calibration/bev_autolabel/test_slab_label.py` (테스트 추가)

**Interfaces:**
- Consumes: Task 1 의 `crop_mask`·`ref_z`·`slab_mask`. `bev_label.rc_of`. `bev_label.raycast_visible(obstacle, spec, step_deg)` — 기존 함수를 그대로 재사용한다(ego 셀에서 360° ray, 첫 obstacle 셀 포함까지 True).
- Produces:
  - `occupancy_counts(P_slab: np.ndarray[(M,3)], spec: BevSpec) -> np.ndarray[(NX,NY), int]`
  - `obstacle_from_counts(counts: np.ndarray, min_pts: int = 3) -> np.ndarray[(NX,NY), bool]`

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`test_slab_label.py` 끝에 추가한다:

```python
def test_occupancy_counts_bins_by_row_col():
    spec = BevSpec(XF=1.0, XR=0.0, YH=0.5)      # NX=20, NY=20
    # x=0.975,y=0.475 → row=floor((1.0-0.975)/0.05)=0, col=floor((0.5-0.475)/0.05)=0
    P = np.array([[0.975, 0.475, 0.0]] * 3 + [[0.025, -0.475, 0.0]])
    cnt = sl.occupancy_counts(P, spec)
    assert cnt.shape == (spec.NX, spec.NY)
    assert cnt[0, 0] == 3
    assert cnt[19, 19] == 1
    assert cnt.sum() == 4


def test_occupancy_counts_drops_out_of_range():
    spec = BevSpec(XF=1.0, XR=0.0, YH=0.5)
    P = np.array([[5.0, 0.0, 0.0], [0.5, 5.0, 0.0]])
    assert sl.occupancy_counts(P, spec).sum() == 0


def test_occupancy_counts_agrees_with_rc_of():
    from bev_label import rc_of
    spec = BevSpec(XF=4.0, XR=2.0, YH=3.0)
    rng = np.random.default_rng(0)
    P = np.column_stack([rng.uniform(-2, 4, 500), rng.uniform(-3, 3, 500),
                         np.zeros(500)])
    cnt = sl.occupancy_counts(P, spec)
    r, c = rc_of(P[:, 0], P[:, 1], spec)
    ok = (r >= 0) & (r < spec.NX) & (c >= 0) & (c < spec.NY)
    assert cnt.sum() == int(ok.sum())
    assert cnt[r[ok][0], c[ok][0]] >= 1


def test_obstacle_threshold_is_min_pts():
    counts = np.array([[0, 1, 2], [3, 4, 10]])
    obs = sl.obstacle_from_counts(counts, min_pts=3)
    assert obs.tolist() == [[False, False, False], [True, True, True]]


def test_raycast_empty_grid_is_all_visible():
    from bev_label import raycast_visible
    spec = BevSpec(XF=1.0, XR=1.0, YH=1.0)      # 40x40
    vis = raycast_visible(np.zeros((spec.NX, spec.NY), bool), spec, step_deg=0.25)
    assert vis.mean() > 0.95


def test_raycast_wall_blocks_cells_behind_it():
    from bev_label import raycast_visible
    spec = BevSpec(XF=1.0, XR=1.0, YH=1.0)      # NX=NY=40, ego=(20,20)
    obs = np.zeros((spec.NX, spec.NY), bool)
    obs[10, :] = True                            # ego 전방 0.5m 에 가로 벽
    vis = raycast_visible(obs, spec, step_deg=0.25)
    assert vis[10, spec.C_EGO]                   # 벽 표면은 보인다
    assert not vis[:10, :].any()                 # 벽 뒤는 전부 미관측
```

- [ ] **Step 2: 실패를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_label.py -q`
Expected: FAIL — `AttributeError: module 'slab_label' has no attribute 'occupancy_counts'`

- [ ] **Step 3: 최소 구현을 쓴다**

`slab_label.py` 의 import 절에 추가한다:

```python
from bev_label import rc_of  # noqa: E402
```

그리고 파일 끝에 추가한다:

```python
def occupancy_counts(P_slab, spec):
    """슬래브 점을 (row,col) 로 눌러 셀당 점 개수 (NX,NY) int.

    2D 기둥 누적이면 충분하다 — 슬래브 자르기가 flatten 앞에 오므로 로봇보다 높은
    장애물(열린 문·상인방·천장)은 이미 배제돼 있다.
    """
    cnt = np.zeros((spec.NX, spec.NY), int)
    P = np.asarray(P_slab, float)
    if len(P) == 0:
        return cnt
    r, c = rc_of(P[:, 0], P[:, 1], spec)
    ok = (r >= 0) & (r < spec.NX) & (c >= 0) & (c < spec.NY)
    np.add.at(cnt, (r[ok], c[ok]), 1)
    return cnt


def obstacle_from_counts(counts, min_pts=3):
    """count >= min_pts 인 셀만 obstacle. min_pts=3 은 실측으로 정한 값 —
    카트가 실제 지나간 궤적 셀이 obstacle 로 찍히는 비율이 N=1 에서 2.8~4.8%,
    N=3 에서 0.2~1.2% 다. N>=10 은 실구조물까지 지운다."""
    return np.asarray(counts) >= min_pts
```

- [ ] **Step 4: 테스트 통과를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_label.py -q`
Expected: PASS (13 passed)

- [ ] **Step 5: 커밋**

```bash
git add calibration/bev_autolabel/slab_label.py calibration/bev_autolabel/test_slab_label.py
git commit -m "feat(bev): occupancy 카운트·임계 + raycast 재사용 검증"
```

---

### Task 3: 비네팅 검출과 카메라 관측가능성 (순수 로직)

**Files:**
- Modify: `calibration/bev_autolabel/slab_label.py`
- Modify: `calibration/bev_autolabel/test_slab_label.py`

**Interfaces:**
- Consumes: `bev_label.cell_centers(spec)`, `chain.project(P_lidar, T_front_lidar, T_cam_front, cam)` → `(u, v, valid)`. `cam` 은 `ds_model.DoubleSphereCamera`(속성 `width`·`height`, 메서드 `project`).
- Produces:
  - `vignette_mask(frames_gray: np.ndarray[(F,H,W)], dark: int = 25, quantile: float = 0.90) -> np.ndarray[(H,W), bool]` (True=무효)
  - `camera_observable(spec, z_body: float, cams: dict, T_cam_front: dict, T_front_lidar: np.ndarray, invalid: dict, use_names=("front","left","right")) -> np.ndarray[(NX,NY), bool]`
  - `assemble(obstacle: np.ndarray, visible: np.ndarray, camera_ok: np.ndarray) -> tuple[np.ndarray, np.ndarray]` — `(occupancy uint8, visibility uint8)`

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`test_slab_label.py` 끝에 추가한다:

```python
def _fake_cam(width=100, height=80):
    """z>0 이면 (x,y) 를 그대로 픽셀로 쓰는 최소 카메라 스텁."""
    class Cam:
        def __init__(self):
            self.width, self.height = width, height

        def project(self, P):
            P = np.asarray(P, float)
            u = P[..., 0] + width / 2.0
            v = P[..., 1] + height / 2.0
            return u, v, P[..., 2] > 0
    return Cam()


def test_vignette_mask_finds_always_dark_pixels():
    F, H, W = 20, 40, 60
    frames = np.zeros((F, H, W), np.uint8)
    frames[:, 10:30, 15:45] = 200                 # 항상 밝은 유효원
    frames[3, 0, 0] = 255                         # 한 프레임만 튄 화소
    m = sl.vignette_mask(frames)
    assert m[0, 0]                                # 이상치 1개로는 유효가 되지 않는다
                                                  # (quantile=0.98 이면 여기서 실패한다)
    assert not m[20, 30]                          # 밝은 영역은 유효
    assert m.mean() > 0.5


def test_vignette_mask_all_bright_is_all_valid():
    frames = np.full((5, 20, 20), 180, np.uint8)
    assert not sl.vignette_mask(frames).any()


def test_camera_observable_rejects_masked_pixels():
    spec = BevSpec(XF=0.5, XR=0.5, YH=0.5)        # 20x20
    cam = _fake_cam()
    cams = {"front": cam}
    T = np.eye(4)
    # z_body=1.0 → 스텁의 z>0 조건 만족, u=x+50, v=y+40 → 전부 이미지 안
    free = sl.camera_observable(spec, 1.0, cams, {"front": T}, T, {}, ("front",))
    assert free.all()
    blocked = np.ones((cam.height, cam.width), bool)
    none = sl.camera_observable(spec, 1.0, cams, {"front": T}, T,
                                {"front": blocked}, ("front",))
    assert not none.any()


def test_camera_observable_ors_over_cameras():
    spec = BevSpec(XF=0.5, XR=0.5, YH=0.5)
    cam = _fake_cam()
    T = np.eye(4)
    blocked = np.ones((cam.height, cam.width), bool)
    got = sl.camera_observable(spec, 1.0, {"a": cam, "b": cam},
                               {"a": T, "b": T}, T,
                               {"a": blocked}, ("a", "b"))
    assert got.all()                              # b 가 보므로 OR 결과는 전부 True


def test_camera_observable_rejects_behind_camera():
    spec = BevSpec(XF=0.5, XR=0.5, YH=0.5)
    cam = _fake_cam()
    T = np.eye(4)
    # z_body=-1.0 → 스텁의 z>0 실패 → 전부 무효
    got = sl.camera_observable(spec, -1.0, {"front": cam}, {"front": T}, T,
                               {}, ("front",))
    assert not got.any()


def test_assemble_encodes_project_convention():
    obstacle = np.array([[True, False]])
    visible = np.array([[True, True]])
    camera_ok = np.array([[True, False]])
    occ, vis = sl.assemble(obstacle, visible, camera_ok)
    assert occ.tolist() == [[0, 1]]               # 0=obstacle, 1=drivable
    assert vis.tolist() == [[1, 0]]               # camera_ok=False → unseen
    assert occ.dtype == np.uint8 and vis.dtype == np.uint8
```

- [ ] **Step 2: 실패를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_label.py -q`
Expected: FAIL — `AttributeError: module 'slab_label' has no attribute 'vignette_mask'`

- [ ] **Step 3: 최소 구현을 쓴다**

`slab_label.py` 의 import 절을 다음으로 바꾼다:

```python
from bev_label import rc_of, cell_centers  # noqa: E402
from chain import project                  # noqa: E402
```

파일 끝에 추가한다:

```python
def vignette_mask(frames_gray, dark=25, quantile=0.90):
    """프레임 스택 (F,H,W) → '거의 항상 어두운' 화소 = 어안 유효원 바깥. True=무효.

    지면점이 1280x720 사각형 안에만 들어오면 보인다고 세면 틀린다 — 각 이미지의
    12~18%(좌우단 열에서는 60~66%)가 어안 원 바깥 검은 영역이다.

    max 가 아니라 퍼센타일을 쓰는 이유는 JPEG 노이즈로 한두 프레임만 튄 화소에 속지
    않기 위함이다. 0.98 은 너무 높다 — 프레임 40장이면 상위 1장이 그대로 결과를
    뒤집는다(20장·이상치 1개면 quantile(0.98)=158 로 유효 판정된다). 0.90 은 상위
    10% 를 버려 강건하다. 어안 원 바깥이 프레임의 10% 이상 밝아지는 일은 없다.
    """
    A = np.asarray(frames_gray)
    return np.quantile(A, quantile, axis=0) < dark


def camera_observable(spec, z_body, cams, T_cam_front, T_front_lidar, invalid,
                      use_names=("front", "left", "right")):
    """각 BEV 셀을 z_body 평면에 놓고 카메라별로 판정한 뒤 OR. (NX,NY) bool.

    판정 평면이 결정적이다. body z=0 은 카메라 높이(=수평선) 평면이라 그 위의 점은
    이미지 세로 중앙 근처에 찍혀 항상 유효해 보인다(측정 100%). 실제 지면
    z=-0.87 에서 재야 사각지대가 드러난다(96%, 반경 0.5m 완전 사각).

    invalid={name: (H,W) bool}. True 인 화소는 카트 자기 몸(상판·프레임·팔)이나
    어안 원 바깥이라 그 방향은 못 본 것으로 센다. 없는 이름은 마스크 없이 판정한다.
    """
    X, Y = cell_centers(spec)
    P = np.stack([X, Y, np.full_like(X, float(z_body))], axis=-1).reshape(-1, 3)
    ok_any = np.zeros(len(P), bool)
    for name in use_names:
        cam = cams[name]
        u, v, valid = project(P, T_front_lidar, T_cam_front[name], cam)
        ui = np.floor(np.nan_to_num(u, nan=-1.0)).astype(int)
        vi = np.floor(np.nan_to_num(v, nan=-1.0)).astype(int)
        good = (valid & (ui >= 0) & (ui < cam.width)
                & (vi >= 0) & (vi < cam.height))
        m = invalid.get(name)
        if m is not None:
            idx = np.flatnonzero(good)
            good[idx] = ~m[vi[idx], ui[idx]]
        ok_any |= good
    return ok_any.reshape(spec.NX, spec.NY)


def assemble(obstacle, visible, camera_ok):
    """(occupancy, visibility) uint8. occupancy 0=obstacle 1=drivable,
    visibility 0=unseen 1=visible.

    occupancy 에 unknown 클래스를 두지 않는다 — '한 번도 관측되지 않은 영역'은
    visibility=0 이 정확히 그 뜻이고, 두 채널의 네 조합이 각각 의미를 갖는다.
    """
    occupancy = np.where(np.asarray(obstacle), 0, 1).astype(np.uint8)
    visibility = (np.asarray(visible) & np.asarray(camera_ok)).astype(np.uint8)
    return occupancy, visibility
```

- [ ] **Step 4: 테스트 통과를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_label.py -q`
Expected: PASS (19 passed)

- [ ] **Step 5: 커밋**

```bash
git add calibration/bev_autolabel/slab_label.py calibration/bev_autolabel/test_slab_label.py
git commit -m "feat(bev): 비네팅 자동검출·카메라 관측가능성·라벨 조립"
```

---

### Task 4: PCD·마스크 IO

**Files:**
- Create: `calibration/bev_autolabel/slab_io.py`
- Test: `calibration/bev_autolabel/test_slab_label.py` (IO 테스트 추가 — tmp_path 사용)

**Interfaces:**
- Consumes: `mapping/pcd_denoise.read_pcd_raw(path) -> (head: list[bytes], arr: np.ndarray structured)`, `mapping/pcd_denoise.write_pcd_raw(path, head, arr)`. `arr` 는 `x/y/z/intensity/normal_x/normal_y/normal_z/curvature` 8필드 구조화 배열이다.
- Produces:
  - `load_map(map_dir: str) -> tuple[list, np.ndarray, np.ndarray, np.ndarray, list]` = `(head, arr, xyz(N,3) float64, times_ns, poses)`
  - `write_points(path, head, arr, idx: np.ndarray, xyz_body: np.ndarray)` — `arr[idx]` 를 복제하고 x/y/z 를 `xyz_body` 로 덮어 쓴다(나머지 필드 보존)
  - `load_labelmap(mask_dir) -> dict[str, tuple[int,int,int]]` (클래스명→RGB, `background` 제외)
  - `load_self_masks(mask_dir, use_names=("front","left","right"), classes=("table",), tol=10) -> dict[str, np.ndarray[bool]]` (True=무효). 폴더·파일이 없으면 그 이름을 건너뛴다
  - `sample_frames_gray(extract_dir, cam_idx: int, n: int = 40) -> np.ndarray[(F,H,W), uint8]`

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`test_slab_label.py` 끝에 추가한다:

```python
def _write_min_pcd(path, xyz, inten):
    """테스트용 최소 8필드 binary PCD 작성."""
    import struct
    n = len(xyz)
    head = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
        "SIZE 4 4 4 4 4 4 4 4\nTYPE F F F F F F F F\n"
        "COUNT 1 1 1 1 1 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(head.encode("ascii"))
        for i in range(n):
            f.write(struct.pack("<8f", xyz[i, 0], xyz[i, 1], xyz[i, 2],
                                inten[i], 0.0, 0.0, 0.0, 0.0))


def test_write_points_preserves_intensity_and_overwrites_xyz(tmp_path):
    import slab_io
    src = tmp_path / "map_clean.pcd"
    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
    _write_min_pcd(src, xyz, np.array([11.0, 22.0, 33.0]))
    head, arr = read_pcd_raw(str(src))
    out = tmp_path / "slab.pcd"
    idx = np.array([0, 2])
    body = np.array([[-1.0, -2.0, -3.0], [-7.0, -8.0, -9.0]])
    slab_io.write_points(str(out), head, arr, idx, body)
    _, got = read_pcd_raw(str(out))
    assert len(got) == 2
    assert got["intensity"].tolist() == [11.0, 33.0]        # 보존
    assert got["x"].tolist() == [-1.0, -7.0]                # body 좌표로 교체
    assert got["z"].tolist() == [-3.0, -9.0]


def test_load_self_masks_missing_dir_is_empty(tmp_path):
    import slab_io
    assert slab_io.load_self_masks(str(tmp_path / "nope")) == {}


_LABELMAP = ("# label:color_rgb:parts:actions\n"
             "background:0,0,0::\n"
             "handle:61,245,61::\n"
             "human:140,120,240::\n"
             "table:250,50,83::\n")


def _write_class_mask(path):
    """실측 마스크와 같은 클래스 색으로 3줄짜리 마스크를 만든다(BGR 로 씀)."""
    import cv2
    img = np.zeros((10, 20, 3), np.uint8)        # background=(0,0,0)
    img[0:3] = (83, 50, 250)                     # table  (RGB 250,50,83)
    img[3:5] = (240, 120, 140)                   # human  (RGB 140,120,240)
    img[5:6] = (61, 245, 61)                     # handle (RGB 61,245,61)
    cv2.imwrite(str(path), img)


def test_load_labelmap_excludes_background(tmp_path):
    import slab_io
    (tmp_path / "labelmap.txt").write_text(_LABELMAP)
    assert slab_io.load_labelmap(str(tmp_path)) == {
        "handle": (61, 245, 61), "human": (140, 120, 240), "table": (250, 50, 83)}


def test_load_self_masks_default_reads_table_only(tmp_path):
    import slab_io
    (tmp_path / "labelmap.txt").write_text(_LABELMAP)
    _write_class_mask(tmp_path / "mask_front.png")
    m = slab_io.load_self_masks(str(tmp_path), use_names=("front",))
    assert set(m) == {"front"}
    assert m["front"][:3].all()                  # table 만 무효
    assert not m["front"][3:6].any()             # human·handle 은 self 박스가 덮는다
    assert not m["front"][6:].any()              # background 는 유효


def test_load_self_masks_can_select_more_classes(tmp_path):
    import slab_io
    (tmp_path / "labelmap.txt").write_text(_LABELMAP)
    _write_class_mask(tmp_path / "mask_front.png")
    m = slab_io.load_self_masks(str(tmp_path), use_names=("front",),
                                classes=("table", "human", "handle"))
    assert m["front"][:6].all()
    assert not m["front"][6:].any()


def test_load_self_masks_skips_missing_file(tmp_path):
    import slab_io
    (tmp_path / "labelmap.txt").write_text(_LABELMAP)
    assert slab_io.load_self_masks(str(tmp_path), use_names=("front",)) == {}


def test_load_self_masks_rejects_unknown_class(tmp_path):
    import slab_io
    (tmp_path / "labelmap.txt").write_text(_LABELMAP)
    _write_class_mask(tmp_path / "mask_front.png")
    with pytest.raises(ValueError):
        slab_io.load_self_masks(str(tmp_path), use_names=("front",),
                                classes=("nope",))
```

`test_slab_label.py` 상단 `sys.path` 블록에 `mapping` 을 추가하고 `read_pcd_raw` 를 import 한다
(`_write_min_pcd` 로 쓴 PCD 를 되읽어 필드 보존을 확인하는 데 쓴다):

```python
sys.path.insert(0, str(_HERE.parent.parent / "mapping"))
```

기존 import 줄 아래에 추가:

```python
from pcd_denoise import read_pcd_raw  # noqa: E402
```

- [ ] **Step 2: 실패를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_label.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'slab_io'`

- [ ] **Step 3: 최소 구현을 쓴다**

`calibration/bev_autolabel/slab_io.py` 를 새로 만든다:

```python
"""슬래브 라벨 IO: 8필드 보존 PCD, self 마스크 PNG, 프레임 표본.

map.pcd 를 open3d 로 읽고 쓰면 intensity·curvature 가 사라진다. Point-LIO 산출은
FIELDS 가 8개라 mapping/pcd_denoise.py 의 바이너리 직접 IO 를 재사용한다.
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent.parent / "mapping"))
from cloud_io import load_tum                        # noqa: E402
from pcd_denoise import read_pcd_raw, write_pcd_raw  # noqa: E402

_MASK_FILES = {"front": "mask_front.png", "right": "mask_right.png",
               "left": "mask_left.png", "rear": "mask_rear.png"}


def load_map(map_dir):
    """map_clean.pcd + trajectory.tum → (head, arr, xyz(N,3), times_ns, poses)."""
    md = pathlib.Path(map_dir)
    head, arr = read_pcd_raw(str(md / "map_clean.pcd"))
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=-1).astype(np.float64)
    times_ns, poses = load_tum(str(md / "trajectory.tum"))
    return head, arr, xyz, times_ns, poses


def write_points(path, head, arr, idx, xyz_body):
    """arr[idx] 를 복제하고 x/y/z 를 body 좌표로 덮어 쓴 뒤 저장. 나머지 필드 보존."""
    sub = arr[np.asarray(idx)].copy()
    sub["x"] = xyz_body[:, 0].astype(sub["x"].dtype)
    sub["y"] = xyz_body[:, 1].astype(sub["y"].dtype)
    sub["z"] = xyz_body[:, 2].astype(sub["z"].dtype)
    write_pcd_raw(str(path), head, sub)


def load_labelmap(mask_dir):
    """labelmap.txt → {클래스명: (R,G,B)}. background 는 뺀다.

    형식(CVAT 내보내기): `# label:color_rgb:parts:actions` 주석 뒤로 `name:R,G,B::` 줄들.
    """
    out = {}
    for line in (pathlib.Path(mask_dir) / "labelmap.txt").read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        name, _, rest = line.partition(":")
        rgb = rest.split(":")[0]
        if name == "background" or not rgb:
            continue
        out[name] = tuple(int(v) for v in rgb.split(","))
    return out


def load_self_masks(mask_dir, use_names=("front", "left", "right"),
                    classes=("table",), tol=10):
    """{name: (H,W) bool} True=무효. 선택한 클래스 색의 화소만 무효로 본다.

    폴더가 없으면 {}, 개별 파일이 없으면 그 이름을 건너뛴다.

    기본이 table 만인 이유: handle·human 은 이미지에서의 위치가 프레임마다 달라진다
    (수집자가 화면을 확인하려 몸을 기울이고, 회전 구간에서 위치가 바뀌고, 턱에 걸려
    흔들린다). 정적 이미지 마스크는 없는 자리를 가리고(데이터 손실) 있는 자리를 놓친다
    (틀린 라벨). 물리적 위치는 body 프레임에서 늘 같으므로 slab_label.self_box_mask 가
    대신 덮는다.

    tol 은 색 비교 허용오차. 실측 마스크는 고유색이 2~4개뿐이라 정확히 일치하지만,
    나중에 안티에일리어싱된 내보내기가 와도 경계가 새지 않게 여유를 둔다.
    """
    import cv2
    d = pathlib.Path(mask_dir)
    out = {}
    if not d.is_dir():
        return out
    cmap = load_labelmap(d)
    want = [np.array(cmap[c], int) for c in classes if c in cmap]
    if not want:
        raise ValueError(f"labelmap.txt 에 {tuple(classes)} 중 아무 클래스도 없습니다: {d}")
    for name in use_names:
        p = d / _MASK_FILES[name]
        if not p.is_file():
            continue
        bgr = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        rgb = bgr[:, :, ::-1].astype(int)
        m = np.zeros(rgb.shape[:2], bool)
        for c in want:
            m |= np.abs(rgb - c).max(axis=2) <= tol
        out[name] = m
    return out


def sample_frames_gray(extract_dir, cam_idx, n=40):
    """추출 프레임에서 n 장을 균등 표본해 그레이스케일 (F,H,W) uint8 로."""
    import cv2
    ed = pathlib.Path(extract_dir)
    frames = sorted(ed.glob("frame_*"))
    if not frames:
        raise FileNotFoundError(f"frame_* 폴더가 없습니다: {ed}")
    sel = [frames[int(round(t))] for t in
           np.linspace(0, len(frames) - 1, min(n, len(frames)))]
    out = []
    for d in sel:
        img = cv2.imread(str(d / f"cam{cam_idx}.jpg"), cv2.IMREAD_GRAYSCALE)
        if img is not None:
            out.append(img)
    if not out:
        raise FileNotFoundError(f"cam{cam_idx}.jpg 를 못 읽었습니다: {ed}")
    return np.stack(out)
```

- [ ] **Step 4: 테스트 통과를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_label.py -q`
Expected: PASS (26 passed — Task 1~3 의 19개 + 새 7개)

- [ ] **Step 5: 커밋**

```bash
git add calibration/bev_autolabel/slab_io.py calibration/bev_autolabel/test_slab_label.py
git commit -m "feat(bev): 8필드 보존 PCD·self 마스크·프레임 표본 IO"
```

---

### Task 5: 라벨 PNG 와 검수뷰 렌더

**Files:**
- Create: `calibration/bev_autolabel/slab_render.py`
- Test: `calibration/bev_autolabel/test_slab_render.py`

**Interfaces:**
- Consumes: `bev_label.BevSpec`. PIL `Image`, cv2.
- Produces:
  - `save_indexed(path, arr: np.ndarray[uint8], palette: list[int])`
  - `PALETTE_OCC: list[int]` (0=obstacle 빨강, 1=drivable 초록), `PALETTE_VIS: list[int]` (0=unseen 검정, 1=visible 흰색)
  - `four_color(occupancy: np.ndarray, visibility: np.ndarray) -> np.ndarray[(NX,NY,3), uint8]` BGR
  - `review_png(occupancy, visibility, spec, scale: int = 6) -> np.ndarray[uint8]` BGR

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`calibration/bev_autolabel/test_slab_render.py` 를 새로 만든다:

```python
"""슬래브 라벨 렌더 테스트."""
import pathlib
import sys

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
from bev_label import BevSpec       # noqa: E402
import slab_render as sr            # noqa: E402


def test_save_indexed_roundtrip(tmp_path):
    from PIL import Image
    lab = np.array([[0, 1], [1, 0]], np.uint8)
    p = tmp_path / "occupancy.png"
    sr.save_indexed(str(p), lab, sr.PALETTE_OCC)
    got = np.array(Image.open(p))
    assert got.tolist() == lab.tolist()          # 인덱스값이 그대로 보존
    assert Image.open(p).mode == "P"


def test_four_color_distinguishes_all_combinations():
    occ = np.array([[0, 0], [1, 1]], np.uint8)   # 0=obstacle, 1=drivable
    vis = np.array([[1, 0], [1, 0]], np.uint8)
    img = sr.four_color(occ, vis)
    assert img.shape == (2, 2, 3)
    cols = {tuple(img[r, c]) for r in range(2) for c in range(2)}
    assert len(cols) == 4                         # 네 조합이 서로 다른 색


def test_review_png_scales_and_marks_ego():
    spec = BevSpec(XF=1.0, XR=1.0, YH=1.0)        # 40x40
    occ = np.ones((spec.NX, spec.NY), np.uint8)
    vis = np.ones((spec.NX, spec.NY), np.uint8)
    img = sr.review_png(occ, vis, spec, scale=4)
    assert img.shape == (spec.NX * 4, spec.NY * 4, 3)
    ey, ex = spec.R_EGO * 4, spec.C_EGO * 4
    # ego 색과 **정확히 일치**해야 한다. '기본색과 다르다'로 두면 무의미한 테스트가 된다 —
    # 이 spec 에서 ego 는 0.5m 경계에 정확히 놓여 격자선이 ego 픽셀을 지나가므로, 마커
    # 그리기를 통째로 지워도 격자 회색 때문에 '다르다'가 참이 된다(실측 확인).
    assert tuple(img[ey, ex]) == sr._C_EGO
```

- [ ] **Step 2: 실패를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_render.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'slab_render'`

- [ ] **Step 3: 최소 구현을 쓴다**

`calibration/bev_autolabel/slab_render.py` 를 새로 만든다:

```python
"""슬래브 라벨 렌더: 인덱스 팔레트 PNG + 4색 검수뷰."""
from __future__ import annotations

import numpy as np
import cv2
from PIL import Image

# occupancy: 0=obstacle 빨강, 1=drivable 초록 (기존 label.png 와 같은 약속)
PALETTE_OCC = [255, 0, 0, 0, 200, 0] + [0] * (256 * 3 - 6)
# visibility: 0=unseen 검정, 1=visible 흰색
PALETTE_VIS = [0, 0, 0, 255, 255, 255] + [0] * (256 * 3 - 6)

_C_VIS_DRIV = (0, 190, 0)      # BGR 초록: 보이는 drivable = 학습에 쓸 신뢰 영역
_C_VIS_OBS = (40, 40, 220)     # 빨강: 보이는 장애물 표면
_C_OCC_OBS = (60, 60, 110)     # 갈색: 가려진 장애물
_C_UNSEEN = (30, 30, 30)       # 검정: 미관측
_C_EGO = (0, 255, 255)


def save_indexed(path, arr, palette):
    """uint8 인덱스 배열을 팔레트 PNG 로. 화소값은 인덱스 그대로 보존된다."""
    im = Image.fromarray(np.asarray(arr, np.uint8), mode="P")
    im.putpalette(palette)
    im.save(str(path))


def four_color(occupancy, visibility):
    """(NX,NY,3) BGR. occ/vis 네 조합을 서로 다른 색으로."""
    occ = np.asarray(occupancy)
    vis = np.asarray(visibility).astype(bool)
    obstacle = occ == 0
    out = np.zeros(occ.shape + (3,), np.uint8)
    out[...] = _C_UNSEEN
    out[vis & ~obstacle] = _C_VIS_DRIV
    out[vis & obstacle] = _C_VIS_OBS
    out[~vis & obstacle] = _C_OCC_OBS
    return out


def review_png(occupancy, visibility, spec, scale=6):
    """4색 확대 + 0.5m 격자 + ego 마커 + 전방 화살표.

    격자선 위치는 미터좌표 기준으로 잡는다 — 격자 좌상단 기준이면 XF/YH 가 0.5 배수가
    아닐 때 선과 미터 라벨이 어긋나 거리를 잘못 읽는다.
    """
    img = cv2.resize(four_color(occupancy, visibility),
                     (spec.NY * scale, spec.NX * scale),
                     interpolation=cv2.INTER_NEAREST)
    H, W = img.shape[:2]
    step = 0.5
    for x_m in np.arange(np.floor(spec.XF / step) * step, -spec.XR - 1e-9, -step):
        gy = int(round((spec.XF - x_m) / spec.RES)) * scale
        if 0 <= gy < H:
            cv2.line(img, (0, gy), (W, gy), (70, 70, 70), 1)
    for y_m in np.arange(np.floor(spec.YH / step) * step, -spec.YH - 1e-9, -step):
        gx = int(round((spec.YH - y_m) / spec.RES)) * scale
        if 0 <= gx < W:
            cv2.line(img, (gx, 0), (gx, H), (70, 70, 70), 1)
    for r_m in range(-int(spec.XR), int(spec.XF) + 1):
        py = int(round((spec.XF - r_m) / spec.RES)) * scale
        cv2.putText(img, f"{r_m}m", (2, max(12, min(py, H - 2))),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1, cv2.LINE_AA)
    ex, ey = spec.C_EGO * scale, spec.R_EGO * scale
    cv2.circle(img, (ex, ey), max(2, scale // 2), _C_EGO, -1)
    cv2.arrowedLine(img, (ex, ey), (ex, max(0, ey - 6 * scale)), _C_EGO, 1,
                    tipLength=0.3)
    return img
```

- [ ] **Step 4: 테스트 통과를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_render.py -q`
Expected: PASS (3 passed)

- [ ] **Step 5: 커밋**

```bash
git add calibration/bev_autolabel/slab_render.py calibration/bev_autolabel/test_slab_render.py
git commit -m "feat(bev): 슬래브 라벨 PNG 저장·4색 검수뷰 렌더"
```

---

### Task 6: CLI 와 실기 단계 검증

**Files:**
- Create: `calibration/bev_autolabel/generate_slab.py`
- Modify: `calibration/bev_autolabel/slab_label.py` (`self_box_mask` 추가)
- Modify: `calibration/bev_autolabel/test_slab_label.py` (`self_box_mask` 테스트 추가)
- Create: `calibration/bev_autolabel/slab_sheet.py` (검수 시트·통계. 산출물은 `data/bev/slab_check/<name>/_sheet_review.png`·`_sheet_stack.png`·`_stats.txt`)

**Interfaces:**
- Consumes: Task 1~5 의 전부. `bev_io.load_stamps(extract_dir) -> {idx: stamp_ns}`, `bev_label.select_keyframes(stamps, times_ns, poses, kf_step) -> list[int]`, `cloud_io.pose_at(times_ns, poses, t_ns) -> (4,4)`, `chain.se3_inv`·`transform`, `ds_model.load_rig(calib, orient) -> CameraRig`(속성 `cams_by_name`·`T_cam_front`·`idx_to_name`), `calib_io.load_T_front_lidar(calib) -> (4,4)|None`
- Produces: `self_box_mask(spec, near: float = 0.4, far: float = 2.1, yh: float = 0.7) -> np.ndarray[(NX,NY), bool]` (True=무효), 샘플 폴더 산출물과 `dataset.csv`. 이 CLI 가 최종 사용자 진입점이다.

- [ ] **Step 1: `self_box_mask` 의 실패하는 테스트를 쓴다**

카트 손잡이와 수집자를 body 프레임 박스로 덮는다(이미지 마스크가 아니라). 근거는 설계 §2.8.1 —
이미지에서의 위치는 프레임마다 달라지지만(기울기·회전·턱 걸림) 물리적 위치는 body 프레임에서
늘 같다. 기본 박스(896셀, 6.2%)가 handle·human 이미지 마스크가 죽이던 143셀을 100% 포함하고
Point-LIO self mask 박스(x −1.5~−0.45, |y|<0.35)도 완전히 담는다.

`test_slab_label.py` 끝에 추가한다:

```python
def test_self_box_mask_bounds():
    from bev_label import cell_centers
    spec = BevSpec(XF=1.0, XR=1.0, YH=1.0)          # NX=NY=40
    m = sl.self_box_mask(spec, near=0.2, far=0.6, yh=0.1)
    X, Y = cell_centers(spec)
    assert not m[X > -0.2].any()                     # near 보다 가까우면 제외
    assert not m[X < -0.6].any()                     # far 보다 멀면 제외
    assert not m[np.abs(Y) > 0.1].any()
    assert m.sum() == 8 * 4                          # x 0.4m→8셀, |y|<=0.1→4셀


def test_self_box_default_contains_pointlio_box():
    from bev_label import cell_centers
    spec = BevSpec(XF=4.0, XR=2.0, YH=3.0)           # 120x120
    m = sl.self_box_mask(spec)
    X, Y = cell_centers(spec)
    pointlio = (X >= -1.5) & (X <= -0.45) & (np.abs(Y) < 0.35)
    assert not (pointlio & ~m).any()                 # Point-LIO 잔재 영역을 전부 담는다
    assert m.sum() == 896                            # 실측으로 정한 기본 박스 크기
```

- [ ] **Step 2: 실패를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_label.py -q`
Expected: FAIL — `AttributeError: module 'slab_label' has no attribute 'self_box_mask'`

- [ ] **Step 3: `self_box_mask` 를 구현한다**

`slab_label.py` 끝에 추가한다:

```python
def self_box_mask(spec, near=0.4, far=2.1, yh=0.7):
    """후방 self 박스: -far <= x <= -near 이고 |y| <= yh 인 셀 (NX,NY) bool. True=무효.

    카트 손잡이(폭 실측 46cm)와 이를 밀며 따라오는 수집자(LiDAR 기준 약 60cm 후방)를 덮어
    visibility 를 0 으로 만든다.

    이미지 마스크가 아니라 body 프레임 박스로 처리하는 이유: 수집자는 화면을 확인하려 몸을
    기울이고, 회전 구간에서 위치가 바뀌고, 턱에 걸려 흔들린다. 정적 이미지 마스크는 그 변동
    앞에서 없는 자리를 가리고(데이터 손실) 있는 자리를 놓친다(틀린 라벨). 물리적 위치는
    body 프레임에서 늘 같은 영역이다.

    기본값은 실측으로 정했다. 이 박스(896셀, 6.2%)는 (1) handle·human 이미지 마스크가
    죽이던 셀 143개(x −1.98~−0.98m, |y| 최대 0.68m)를 100% 포함하고, (2) Point-LIO
    self mask 박스(x −1.5~−0.45, |y|<0.35)도 완전히 담는다 — 후자는 map_clean.pcd 에 남은
    수집자 잔재(허위 obstacle)의 위치이며 이미지 마스크로는 고칠 수 없는 부분이다.
    """
    X, Y = cell_centers(spec)
    return (X >= -far) & (X <= -near) & (np.abs(Y) <= yh)
```

- [ ] **Step 4: 테스트 통과를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_label.py -q`
Expected: PASS (28 passed — Task 1~4 의 26개 + 새 2개)

- [ ] **Step 5: CLI 를 쓴다**

`calibration/bev_autolabel/generate_slab.py` 를 새로 만든다:

```python
#!/usr/bin/env python3
"""map_clean.pcd → 키프레임별 BEV occupancy + visibility 라벨.

파이프라인: pose 로 body 프레임 3D crop → 하위 pct z 부터 thick 만큼의 슬래브 →
2D 기둥 누적 count>=min_pts 로 occupancy → ego 셀 2D 360° raycast ∧ 카메라
관측가능성으로 visibility. 설계 근거는
docs/superpowers/specs/2026-08-10-bev-slab-label-design.md

사용:
  cd calibration/bev_autolabel
  python3 generate_slab.py \
    --map-dir ../../data/sj_bags/260722/maps_selfmask/raws3_mapping \
    --extract-dir ../../data/extracted/raws3 \
    --calib ../../data/calib_260723/calib.yaml \
    --orient ../../data/calib_260723/orientation.json \
    --self-mask-dir ../../data/calib_260723/self_mask \
    --out ../../data/bev/slab/raws3
"""
import argparse
import csv
import json
import pathlib
import sys

import numpy as np
import cv2

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
from calib_io import load_T_front_lidar                    # noqa: E402
from chain import se3_inv, transform                       # noqa: E402
from cloud_io import pose_at                               # noqa: E402
from ds_model import load_rig                              # noqa: E402
import bev_io                                              # noqa: E402
from bev_label import BevSpec, raycast_visible, select_keyframes  # noqa: E402
import slab_label as sl                                    # noqa: E402
import slab_io                                             # noqa: E402
import slab_render as sr                                   # noqa: E402

USE = ("front", "left", "right")


def build_invalid_masks(extract_dir, rig, self_mask_dir, classes=("table",),
                        use_names=USE):
    """{name: (H,W) bool True=무효} = 어안 원 바깥(자동) ∪ self 마스크(수작업).

    self 마스크가 없으면 어안 원만 쓰고 경고한다 — 그 경우 카트 상판이 가린 근거리가
    '보인다'고 나와 visibility 가 낙관적이다. 손잡이·수집자는 여기서 다루지 않는다
    (slab_label.self_box_mask 가 body 프레임에서 덮는다).
    """
    name2idx = {v: k for k, v in rig.idx_to_name.items()}
    self_masks = (slab_io.load_self_masks(self_mask_dir, use_names, classes)
                  if self_mask_dir else {})
    if not self_masks:
        print("[경고] self 마스크가 없습니다 — 카트 상판이 가린 근거리 visibility 가 "
              "낙관적으로 나옵니다.")
    out = {}
    for name in use_names:
        frames = slab_io.sample_frames_gray(extract_dir, name2idx[name])
        m = sl.vignette_mask(frames)
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN,
                             np.ones((5, 5), np.uint8)).astype(bool)
        sm = self_masks.get(name)
        if sm is not None:
            if sm.shape != m.shape:
                sys.exit(f"self 마스크 크기 불일치 {name}: {sm.shape} != {m.shape}")
            m |= sm
        out[name] = m
        print(f"  invalid[{name}]: 무효 화소 {m.mean()*100:.1f}% "
              f"(self 마스크 {'있음' if sm is not None else '없음'})")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-dir", required=True)
    ap.add_argument("--extract-dir", required=True)
    ap.add_argument("--calib", required=True)
    ap.add_argument("--orient", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--self-mask-dir", default=None)
    ap.add_argument("--xf", type=float, default=4.0)
    ap.add_argument("--xr", type=float, default=2.0)
    ap.add_argument("--yh", type=float, default=3.0)
    ap.add_argument("--thick", type=float, default=0.8,
                    help="슬래브 두께[m]. 로봇이 통과해야 하는 높이 구간")
    ap.add_argument("--pct", type=float, default=1.0,
                    help="z_ref 퍼센타일. 5 는 슬래브 바닥을 0.25m 들어올린다")
    ap.add_argument("--min-pts", type=int, default=3)
    ap.add_argument("--ray-step", type=float, default=0.25)
    ap.add_argument("--ground-offset", type=float, default=0.87,
                    help="z_ref 아래 실제 지면까지의 거리[m]. IPM cam_height 와 같은 평면")
    ap.add_argument("--self-mask-classes", default="table",
                    help="이미지 마스크에서 무효로 읽을 클래스(쉼표 구분). handle·human 은 "
                         "이미지 위치가 프레임마다 달라 self 박스로 덮는다(설계 §2.8.1)")
    ap.add_argument("--self-box-near", type=float, default=0.4,
                    help="후방 self 박스 근단[m]. 박스 = -far <= x <= -near, |y| <= yh")
    ap.add_argument("--self-box-far", type=float, default=2.1)
    ap.add_argument("--self-box-yh", type=float, default=0.7)
    ap.add_argument("--kf-step", type=float, default=0.4)
    ap.add_argument("--save-crop", action="store_true")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--review-scale", type=int, default=6)
    a = ap.parse_args()

    spec = BevSpec(XF=a.xf, XR=a.xr, YH=a.yh)
    print(f"BEV {spec.NX}x{spec.NY} (XF={spec.XF} XR={spec.XR} YH={spec.YH} "
          f"RES={spec.RES}) 슬래브 {a.thick}m @ p{a.pct}")

    rig = load_rig(a.calib, a.orient)
    T_front_lidar = load_T_front_lidar(a.calib)
    if T_front_lidar is None:
        sys.exit("calib.yaml 에 extrinsics.T_front_lidar 가 없습니다")
    classes = tuple(c.strip() for c in a.self_mask_classes.split(",") if c.strip())
    invalid = build_invalid_masks(a.extract_dir, rig, a.self_mask_dir, classes)

    head, arr, xyz, times_ns, poses = slab_io.load_map(a.map_dir)
    stamps = bev_io.load_stamps(a.extract_dir)
    kf = select_keyframes(stamps, times_ns, poses, kf_step=a.kf_step)
    if a.limit:
        kf = kf[:a.limit]
    print(f"map={len(xyz)} poses={len(poses)} keyframes={len(kf)}")

    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    # world bbox 사전 필터 반경: body 프레임 crop 의 가장 먼 모서리까지의 거리.
    # max(XF,XR,YH) 로는 부족하다 — 회전에 따라 (XF,YH) 모서리가 world 축에 정렬되면
    # hypot(4,3)=5.0m 가 필요한데 max()*1.5=4.5m 는 모서리를 잘라낸다.
    reach = float(np.hypot(max(spec.XF, spec.XR), spec.YH)) + spec.RES
    # 박스는 샘플과 무관하므로 루프 밖에서 한 번만 만든다.
    self_box = sl.self_box_mask(spec, a.self_box_near, a.self_box_far, a.self_box_yh)
    print(f"self 박스: x -{a.self_box_far}~-{a.self_box_near} |y|<={a.self_box_yh} "
          f"→ {int(self_box.sum())}셀 ({self_box.mean()*100:.1f}%)")
    rows = []
    for n, idx in enumerate(kf):
        t_ns = stamps[idx]
        T_wb = pose_at(times_ns, poses, t_ns)
        ctr = T_wb[:3, 3]
        near = np.flatnonzero((np.abs(xyz[:, 0] - ctr[0]) < reach)
                              & (np.abs(xyz[:, 1] - ctr[1]) < reach))
        P = transform(se3_inv(T_wb), xyz[near])
        cm = sl.crop_mask(P, spec)
        crop_idx, crop_P = near[cm], P[cm]
        if len(crop_P) == 0:
            print(f"skip (crop 비어 있음) frame_idx={idx}")
            continue
        z_ref = sl.ref_z(crop_P[:, 2], pct=a.pct)
        sm = sl.slab_mask(crop_P, z_ref, thick=a.thick)
        slab_idx, slab_P = crop_idx[sm], crop_P[sm]

        counts = sl.occupancy_counts(slab_P, spec)
        obstacle = sl.obstacle_from_counts(counts, min_pts=a.min_pts)
        visible = raycast_visible(obstacle, spec, step_deg=a.ray_step)
        cam_ok = sl.camera_observable(spec, z_ref - a.ground_offset,
                                      rig.cams_by_name, rig.T_cam_front,
                                      T_front_lidar, invalid, use_names=USE)
        # self 박스는 cam_ok 와 따로 둔다 — stats 의 camera_ok_pct 가 '기하+이미지 마스크
        # 커버리지' 진단값으로 남아야 박스가 그 수치를 가리지 않는다.
        occupancy, visibility = sl.assemble(obstacle, visible, cam_ok & ~self_box)

        sd = out / f"sample_{n:06d}"
        sd.mkdir(exist_ok=True)
        slab_io.write_points(sd / "slab.pcd", head, arr, slab_idx, slab_P)
        if a.save_crop:
            slab_io.write_points(sd / "crop.pcd", head, arr, crop_idx, crop_P)
        sr.save_indexed(sd / "occupancy.png", occupancy, sr.PALETTE_OCC)
        sr.save_indexed(sd / "visibility.png", visibility, sr.PALETTE_VIS)
        cv2.imwrite(str(sd / "review.png"),
                    sr.review_png(occupancy, visibility, spec, scale=a.review_scale))
        stats = {"crop_pts": int(len(crop_P)), "slab_pts": int(len(slab_P)),
                 "obstacle_pct": float(obstacle.mean() * 100),
                 "visible_pct": float((visibility == 1).mean() * 100),
                 "camera_ok_pct": float(cam_ok.mean() * 100)}
        meta = {
            "frame_idx": idx, "stamp_ns": t_ns,
            "world_T_body": T_wb.tolist(),
            "bev": {"XF": spec.XF, "XR": spec.XR, "YH": spec.YH, "RES": spec.RES,
                    "NX": spec.NX, "NY": spec.NY,
                    "R_EGO": spec.R_EGO, "C_EGO": spec.C_EGO},
            "z_ref": z_ref,
            "classes": {"occupancy": {"0": "obstacle", "1": "drivable"},
                        "visibility": {"0": "unseen", "1": "visible"}},
            "cameras": list(USE),
            "calib": str(pathlib.Path(a.calib).resolve()),
            "orient": str(pathlib.Path(a.orient).resolve()),
            "self_mask": (str(pathlib.Path(a.self_mask_dir).resolve())
                          if a.self_mask_dir else None),
            "self_mask_classes": list(classes),
            "self_box": {"near": a.self_box_near, "far": a.self_box_far,
                         "yh": a.self_box_yh, "cells": int(self_box.sum())},
            "params": {"thick": a.thick, "pct": a.pct, "min_pts": a.min_pts,
                       "ray_step": a.ray_step, "ground_offset": a.ground_offset,
                       "kf_step": a.kf_step},
            "stats": stats,
        }
        (sd / "meta.json").write_text(json.dumps(meta, indent=2))
        rows.append({"sample": sd.name, "frame_idx": idx, "stamp_ns": t_ns,
                     "z_ref": f"{z_ref:.4f}", **{k: f"{v:.2f}" if isinstance(v, float)
                                                 else v for k, v in stats.items()}})
        if n % 20 == 0:
            print(f"{n}/{len(kf)} {sd.name} z_ref={z_ref:+.3f} "
                  f"slab={len(slab_P)} obs={stats['obstacle_pct']:.1f}% "
                  f"vis={stats['visible_pct']:.1f}%")

    if not rows:
        sys.exit("샘플이 하나도 만들어지지 않았습니다")
    with open(out / "dataset.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    zr = np.array([float(r["z_ref"]) for r in rows])
    ob = np.array([float(r["obstacle_pct"]) for r in rows])
    vi = np.array([float(r["visible_pct"]) for r in rows])
    print(f"done: {len(rows)} samples → {out}")
    print(f"  z_ref {zr.min():+.3f}~{zr.max():+.3f} (중앙 {np.median(zr):+.3f})")
    print(f"  obstacle {ob.min():.1f}~{ob.max():.1f}% (중앙 {np.median(ob):.1f}%)")
    print(f"  visible  {vi.min():.1f}~{vi.max():.1f}% (중앙 {np.median(vi):.1f}%)")


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: 단계 1 검증 — crop/slab pcd**

Run:
```bash
cd calibration/bev_autolabel && python3 generate_slab.py \
  --map-dir ../../data/sj_bags/260722/maps_selfmask/raws3_mapping \
  --extract-dir ../../data/extracted/raws3 \
  --calib ../../data/calib_260723/calib.yaml \
  --orient ../../data/calib_260723/orientation.json \
  --out /tmp/slab_check/raws3 --limit 5 --save-crop
```
판정: 5개 샘플 생성. `z_ref` 가 +0.0~+0.1 범위. `crop_pts` 30~62만, `slab_pts` 7~14만.
`slab.pcd` 를 `read_pcd_raw` 로 다시 읽어 z 범위가 `[z_ref, z_ref+0.8]` 안이고
`intensity` 가 전부 0 이 아님을 확인한다:
```bash
python3 -c "
import sys,json,numpy as np,pathlib
sys.path.insert(0,'../../mapping')
from pcd_denoise import read_pcd_raw
d=pathlib.Path('/tmp/slab_check/raws3/sample_000000')
m=json.loads((d/'meta.json').read_text()); z0=m['z_ref']
_,a=read_pcd_raw(str(d/'slab.pcd'))
print('z', a['z'].min(), a['z'].max(), '기대', z0, z0+m['params']['thick'])
print('intensity nonzero', (a['intensity']!=0).mean())
print('필드', a.dtype.names)
"
```
Expected: z 가 `[z_ref, z_ref+0.8]` 안, intensity nonzero 비율 > 0, 필드 8개.

- [ ] **Step 7: 단계 2 검증 — occupancy**

Run:
```bash
cd calibration/bev_autolabel && python3 -c "
import sys,json,glob,numpy as np,pathlib
sys.path.insert(0,'.'); sys.path.insert(0,'../cam_lidar')
from PIL import Image
from chain import se3_inv, transform
from cloud_io import load_tum, pose_at
from bev_label import BevSpec, rc_of
md='../../data/sj_bags/260722/maps_selfmask/raws3_mapping'
times,poses=load_tum(md+'/trajectory.tum')
tpos=np.array([P[:3,3] for P in poses])
bad=[]
for d in sorted(glob.glob('/tmp/slab_check/raws3/sample_*')):
    m=json.loads(open(d+'/meta.json').read()); spec=BevSpec(m['bev']['XF'],m['bev']['XR'],m['bev']['YH'])
    occ=np.array(Image.open(d+'/occupancy.png'))
    T=np.array(m['world_T_body']); i=int(np.argmin(np.abs(times-m['stamp_ns'])))
    TE=transform(se3_inv(T),tpos[i:])
    TE=TE[(TE[:,0]<=spec.XF)&(TE[:,0]>=-spec.XR)&(np.abs(TE[:,1])<=spec.YH)]
    r,c=rc_of(TE[:,0],TE[:,1],spec); k=(r>=0)&(r<spec.NX)&(c>=0)&(c<spec.NY)
    bad.append((occ[r[k],c[k]]==0).mean()*100)
print('궤적셀이 obstacle 인 비율 %:', [f'{b:.2f}' for b in bad])
print('평균 %.2f%% 최대 %.2f%%'%(np.mean(bad),np.max(bad)))
"
```
판정: 평균 < 2%, 최대 < 9%. (설계 §2.3 실측: raws3 0.2%, rawos1 1.2%)

- [ ] **Step 8: 단계 3 검증 — visibility**

> **주의 — 아래 reach 스크립트는 결함이 있었고 고쳤다.** 원래는 ego 인접행부터 끊김 없는
> 사슬을 따라갔는데, 카메라가 수평을 봐서 반경 0.5m 는 아무 카메라도 못 본다. 그래서 ego
> 인접행의 visibility 가 0 이고 사슬이 즉시 끊겨 **항상 reach=0** 이 나왔다(실측: 가시 영역이
> 0.65~0.80m 에서 시작). 올바른 지표는 **중앙축에서 보이는 가장 먼 거리**다. 이 지표로 raws3
> 전 구간 98샘플 중 83개(85%)가 4.0m 에 도달하고 87%가 3.0m 이상, 1.0m 미만은 2개다.
> 계산은 `slab_sheet.py` 의 `center_reach()` 가 담당한다.

**표본이 공정해야 한다.** `--limit 5` 는 bag 앞부분(넓은 입구)만 뽑으므로 통로 형태가 아니다 —
이것으로 판정하면 안 된다. 전 구간을 돌린 뒤 통계와 컨택트시트로 본다.

```bash
cd calibration/bev_autolabel && python3 generate_slab.py \
  --map-dir ../../data/sj_bags/260722/maps_selfmask/raws3_mapping \
  --extract-dir ../../data/extracted/raws3 \
  --calib ../../data/calib_260723/calib.yaml \
  --orient ../../data/calib_260723/orientation.json \
  --self-mask-dir ../../data/calib_260723/self_mask \
  --out ../../data/bev/slab_check/raws3
python3 slab_sheet.py ../../data/bev/slab_check/raws3
```

판정:
- `reach_far` 중앙값이 XF(4.0m), 4.0m 도달이 80% 이상, 1.0m 미만이 소수(입구·출구 구간)
- `cam_ok` 는 두 숫자를 구분해야 한다. 설계 스펙 §2.7 의 **96.0%** 는 *기하만*(이미지
  마스크 없음, z=−0.87 평면) 잰 값이고, 이 CLI 실행(`--self-mask-dir` 로 table 마스크
  적용)은 그 기하값에서 마스크만큼 더 깎여 **~93~95%** 가 정상이다. 100% 에 가까우면
  투영 평면이 틀렸다는 신호다(body z=0 = 수평선 평면). `--ground-offset` 을 확인한다
- `_sheet_review.png` 에서 중간 구간이 **초록 통로가 전방으로 뻗고 좌우에 갈색 벽, 벽 표면에
  얇은 빨강**. ego 앞 검은 직사각형은 카메라 사각 반경 + 후방 self 박스로 정상이다

실측 결과(raws3 98샘플, table 마스크 적용): `reach_far` 중앙 4.00m·최소 0.75m,
4.0m 도달 83/98(85%), 3.0m 이상 87%, 1.0m 미만 2개. `cam_ok` 92.8~95.3%(기하만인
설계 스펙 §2.7 의 96.0% 보다 낮은 게 정상 — table 마스크가 추가로 가린 만큼).
obstacle 14.5~30.3%(중앙 22.2%). `z_ref` −0.073~+0.111(중앙 +0.048).

- [ ] **Step 9: 커밋**

```bash
git add calibration/bev_autolabel/generate_slab.py calibration/bev_autolabel/slab_label.py \
       calibration/bev_autolabel/test_slab_label.py calibration/bev_autolabel/slab_sheet.py
git commit -m "feat(bev): 슬래브 라벨 생성 CLI + self 박스 + 검수 시트"
```

---

### Task 7: bag 7개 일괄 실행과 문서

**Files:**
- Modify: `docs/BEV_AUTOLABEL.md`
- Modify: `CLAUDE.md`

**Interfaces:**
- Consumes: Task 6 의 CLI.
- Produces: `data/bev/slab/<name>/` 7개. 문서 §B.

- [ ] **Step 1: 7개 bag 을 돌린다**

```bash
cd calibration/bev_autolabel
for n in raws1 raws2 raws3 rawos1 rawos2 rawos3 rawos4; do
  echo "=== $n"
  python3 generate_slab.py \
    --map-dir ../../data/sj_bags/260722/maps_selfmask/${n}_mapping \
    --extract-dir ../../data/extracted/$n \
    --calib ../../data/calib_260723/calib.yaml \
    --orient ../../data/calib_260723/orientation.json \
    --self-mask-dir ../../data/calib_260723/self_mask \
    --out ../../data/bev/slab/$n 2>&1 | tail -5
  python3 slab_sheet.py ../../data/bev/slab/$n | tail -4
done
```
판정: 7개 전부 완주, 실패 샘플 0(`skip (missing image)` 줄이 없어야 한다). bag 간 일관성은
Task 6 이 raws3 로 세운 실측 기준선과 대조한다:

| 항목 | raws3 기준선(98샘플) |
|---|---|
| `z_ref` 중앙 | +0.048 (범위 −0.073~+0.111) |
| `obstacle` 중앙 | 22.2% (범위 14.5~30.3%) |
| `cam_ok` | 92.8~95.3% |
| `vis_start` | 0.65~0.80m |
| `reach_far` 중앙 | 4.00m, 4.0m 도달 85% |

`cam_ok` 가 100% 에 가까운 bag 이 있으면 투영 평면이 틀렸다는 신호다. rawos 계열은 world z
드리프트가 크지만 body 프레임 crop 이 이를 상쇄하므로 `z_ref` 는 raws 와 같은 범위여야 한다 —
벗어나면 그 bag 의 매핑을 의심한다.

각 bag 의 `_sheet_review.png` 를 눈으로 확인한다. 중간 구간이 초록 통로 + 좌우 갈색 벽 형태여야
하고, 입구·출구 구간(첫·마지막 몇 샘플)은 통로가 아니라 열린 공간이라 다르게 보이는 것이 정상이다.

- [ ] **Step 2: 요약표를 만든다**

```bash
cd calibration/bev_autolabel && python3 -c "
import csv,glob,numpy as np
for p in sorted(glob.glob('../../data/bev/slab/*/dataset.csv')):
    R=list(csv.DictReader(open(p)))
    f=lambda k: np.array([float(r[k]) for r in R])
    print('%-8s n=%3d z_ref중앙 %+.3f obs중앙 %4.1f%% vis중앙 %4.1f%% slab중앙 %6d'%(
        p.split('/')[-2], len(R), np.median(f('z_ref')), np.median(f('obstacle_pct')),
        np.median(f('visible_pct')), np.median(f('slab_pts'))))
"
```

- [ ] **Step 3: `docs/BEV_AUTOLABEL.md` 에 §B 를 추가한다**

문서 끝에 다음을 추가한다(§A 는 기존 IPM 경로 그대로 유지):

```markdown
## §B 슬래브 라벨 (LiDAR 라벨 현행판)

`map_clean.pcd` 에서 키프레임별 BEV occupancy + visibility 를 만든다. 기존 §A 의
`generate.py` LiDAR 라벨(`label.png`, 0/1/2)을 대체한다. IPM RGB 경로는 §A 를 그대로 쓴다.

설계 근거·실측값: `docs/superpowers/specs/2026-08-10-bev-slab-label-design.md`

### 실행

    cd calibration/bev_autolabel
    python3 generate_slab.py \
      --map-dir ../../data/sj_bags/260722/maps_selfmask/raws3_mapping \
      --extract-dir ../../data/extracted/raws3 \
      --calib ../../data/calib_260723/calib.yaml \
      --orient ../../data/calib_260723/orientation.json \
      --self-mask-dir ../../data/calib_260723/self_mask \
      --out ../../data/bev/slab/raws3

### 산출물

    sample_NNNNNN/{slab.pcd, crop.pcd(--save-crop), occupancy.png, visibility.png,
                   review.png, cam_{front,left,right}.jpg, meta.json} + dataset.csv

검수 산출물은 `slab_sheet.py` 로 따로 만든다(생성 CLI 가 자동 실행하지 않는다):

    cd calibration/bev_autolabel && python3 slab_sheet.py ../../data/bev/slab/raws3

→ 같은 폴더에 `_sheet_review.png`(궤적 전체 15장 격자)·`_sheet_stack.png`(occupancy/
visibility/review 나란히)·`_stats.txt`(샘플별 z_ref·obstacle·visible·cam_ok·reach).

`occupancy.png` = 0 obstacle / 1 drivable. `visibility.png` = 0 unseen / 1 visible.
학습에서 visibility 를 loss 마스크로 쓰면 미관측 영역이 자동 배제된다. occupancy 에
unknown 클래스를 두지 않는 이유가 이것이다.

`review.png` 는 **상단에 원본 3어안(left/front/right) 스트립** + 하단에 BEV 4색이다.
4색: 초록=보이는 drivable(신뢰 영역), 빨강=보이는 장애물 표면, 갈색=가려진 장애물, 검정=미관측.
원본과 BEV 를 한 장에서 대조할 수 있어야 라벨이 진짜 맞는지 사람이 판단할 수 있다.

ego 주변 검은 직사각형은 **정상**이다 — 카메라가 수평 바깥을 봐서 생기는 근거리 사각
(실측 가시 시작 0.65~0.80m)과 후방 self 박스가 합쳐진 것이다.

### 파이프라인

1. pose 로 body 프레임 3D crop (z 무제한) — rawos 의 world z 드리프트(−1.7m)가 여기서 상쇄된다
2. `z_ref` = crop z 하위 1% ≈ LiDAR 수평면 ≈ 실제 지상 0.87m. 슬래브 `[z_ref, z_ref+0.8]`
   = 로봇이 통과해야 하는 높이 구간. 로봇보다 높은 장애물(열린 문·천장·배관)은 자동 배제
3. occupancy: 슬래브를 2D 기둥으로 눌러 셀당 점 ≥ 3 이면 obstacle
4. visibility: ego 셀 2D 360° raycast(첫 obstacle 에서 정지) ∧ 카메라 관측가능성 ∧ ¬self 박스

### 카트 자기 가림 — 이미지 마스크 + self 박스

카메라가 수평 바깥을 보게 장착돼 아래를 못 내려다본다. **실제 지면(z=−0.87)에서 반경 0.5m
완전 사각, 1.0m 까지 부분 사각**이다(기하만으로 가려지는 셀 4.0%). 여기에 카트 자기 몸이
더해지는데, 두 종류를 **다른 방법으로** 처리한다.

**상판·LiDAR 받침판 → 이미지 마스크.** 카메라에 고정돼 위치가 변하지 않으므로 이미지 공간에
칠하는 게 정확하다. `data/calib_260723/self_mask/mask_{front,right,left}.png` + `labelmap.txt`,
1280×720 클래스 색 PNG(`table 250,50,83`). 기본으로 `table` 만 읽는다(`--self-mask-classes`).
어안 유효원 바깥 검은 영역(이미지의 12~18%)은 프레임 표본의 밝기 퍼센타일로 자동 검출한다.

**손잡이·수집자 → body 프레임 self 박스** (`--self-box-near/far/yh`, 기본 0.4/2.1/0.7 = 896셀,
6.2%). 마스크 파일에 `handle`·`human` 도 칠해져 있지만 쓰지 않는다 — 수집자가 화면을 확인하려
몸을 기울이고, 회전 구간에서 위치가 바뀌고, 턱에 걸려 흔들려서 **이미지에서의 위치가 프레임마다
달라진다**. 정적 이미지 마스크는 없는 자리를 가리고(데이터 손실) 있는 자리를 놓친다(틀린 라벨).
물리적 위치는 body 프레임에서 늘 같으므로 박스가 맞다. 기본 박스는 handle·human 이미지 마스크가
죽이던 셀 143개를 100% 포함하고, `map_clean.pcd` 에 남은 수집자 잔재 위치(Point-LIO self mask
박스 x −1.5~−0.45·|y|<0.35, 허위 obstacle 12~185셀)도 완전히 담는다.

이미지 마스크가 없어도 돌아가지만 근거리 visibility 가 낙관적이라는 경고가 찍히고
`meta.json` 의 `self_mask` 가 `null` 이 된다. self 박스는 마스크와 무관하게 항상 적용된다.

### 판정 기준

| 항목 | 기준 |
|---|---|
| `z_ref` | bag·위치와 무관하게 +0.0~+0.1 |
| obstacle 셀 | 20~30% |
| 궤적셀이 obstacle 인 비율 | < 2% (라벨이 실제 주행과 모순되지 않는지) |
| 중앙축 `reach_far` | 중앙값 XF 도달, 4.0m 도달 80% 이상 (`slab_sheet.py` 가 계산) |
| `cam_ok` | 93~95%. 100% 에 가까우면 투영 평면이 틀렸다는 신호 |

### 하지 말 것

- **3D raycast**: ray 원점(body z=0)이 슬래브 밑면에 붙어 있어 수평 ray 가 장애물 아래로
  빠져나간다. '열에 닿은 voxel 하나라도' 기준이면 visibility 가 거의 전역 1 이 된다.
- **`--min-pts` 를 10 이상으로**: 실구조물까지 지운다(raws3 obstacle 23.8%→14.9%).
- **`--pct` 를 5 로**: 슬래브 바닥이 최대 0.25m 들려 실제 하위 점을 잘라먹는다.
- **corridor prior 부활·self 점 추가 제거**: 측정으로 불필요함이 확인됐고, 넓게 **지우면**
  좌우 0.33~0.58m 의 실제 통로 벽을 갉아먹는다. self 박스는 점을 지우지 않고 visibility 만
  0 으로 두므로 이 금지에 걸리지 않는다.
- **`ground_offset` 를 0 으로**: body z=0 은 수평선 평면이라 FoV 가 100% 로 나오고
  사각지대가 전부 사라진다.
- **`handle`·`human` 을 `--self-mask-classes` 에 넣기**: 이미지에서의 위치가 프레임마다
  달라 정적 마스크로는 못 맞힌다. self 박스가 그 역할이다.
```

- [ ] **Step 4: `CLAUDE.md` 를 갱신한다**

`- **BEV auto-label**` 항목 끝에 다음 줄을 추가한다:

```markdown
  **슬래브 라벨(LiDAR 라벨 현행판)**: `slab_label.py`+`slab_io.py`+`slab_render.py`+`generate_slab.py`
  → `data/bev/slab/<name>/sample_NNNNNN/{slab.pcd,occupancy.png,visibility.png,review.png}`.
  map_clean.pcd 에서 body 프레임 3D crop → 하위1% z 부터 0.8m 슬래브 → 2D 기둥 count≥3
  occupancy + (2D raycast ∧ 카메라 관측가능성 ∧ ¬self박스) visibility. 옛 `label.png`(0/1/2) 대체.
  카메라가 수평을 봐서 실제 지면에서 반경 0.5m 완전 사각·1.0m 부분 사각이다. 카트 자기 가림은
  둘로 나눠 처리: **상판·받침판은 이미지 마스크**(`data/calib_260723/self_mask/`, 클래스 색 PNG,
  기본 `table` 만) + **손잡이·수집자는 body 프레임 self 박스**(기본 x −2.1~−0.4·|y|≤0.7, 896셀)
  — 사람의 이미지 위치가 프레임마다 달라 정적 마스크로는 못 맞히기 때문. 어안 원 바깥은 자동 검출.
  금지: 3D raycast·min_pts≥10·pct=5·corridor prior 부활·ground_offset=0·handle/human 을
  self-mask-classes 에 넣기. `docs/BEV_AUTOLABEL.md §B`.
```

- [ ] **Step 5: 전체 테스트를 돌리고 커밋한다**

```bash
cd src/econ_camera_ros && python3 -m pytest test/ -q
cd ../../calibration/bev_autolabel && python3 -m pytest -q
```
Expected: `src/econ_camera_ros` 기존 25개 PASS + `calibration/bev_autolabel` 35개 PASS

```bash
git add docs/BEV_AUTOLABEL.md CLAUDE.md
git commit -m "docs(bev): 슬래브 라벨 실행 가이드(§B)·CLAUDE.md 갱신"
```

---

### Task 8: IPM RGB 오버레이 + 알려진 한계 기록

**Files:**
- Modify: `calibration/bev_autolabel/slab_render.py` (`blend_slab` 추가, `review_png` 에 IPM 패널)
- Modify: `calibration/bev_autolabel/test_slab_render.py` (테스트 추가)
- Modify: `calibration/bev_autolabel/generate_slab.py` (IPM 캔버스·오버레이 산출)
- Modify: `docs/BEV_AUTOLABEL.md` (§B 에 IPM 산출물 + 알려진 한계)
- Modify: `docs/PIPELINE.md` (5단계에 슬래브 경로 안내)

**왜**: LiDAR 슬래브 라벨만으로는 사람이 맞는지 판단할 수 없다. 바닥 모습이 IPM 에만 있고,
통로가 진짜 막힌 것인지 잎이 슬래브 높이(실제 0.87~1.67m)로 튀어나온 것인지도 RGB 로만 갈린다.
694 샘플 중 45개(6.5%)가 `visibility` 전역 0 인데(§ 아래 한계), 이 45개는 사람이 그 영역을 직접
다시 그려야 하고 나머지도 검수·보정이 필요하다.

**Interfaces:**
- Consumes: `ipm.ipm_canvas(imgs, cams_by_name, T_cam_front, T_front_lidar, cam_height, spec, use_names=USE, blend="nearest") -> (NX,NY,3) BGR uint8`(빈 셀=0). `render.py` 의 `blend_label`·`label_overlay`·`review_overlay` 는 **읽고 패턴만 따르며 수정하지 않는다**(옛 0/1/2 라벨용).
- Produces: `blend_slab(ipm, occupancy, visibility, alpha=0.45) -> (NX,NY,3) uint8`

- [ ] **Step 1: `blend_slab` 의 실패하는 테스트를 쓴다**

오버레이 규약: `vis=1` 인 셀만 색을 얹고, **`vis=0` 셀은 IPM 원본을 그대로 보여준다**. 미관측
영역에 색을 얹으면 사람이 장면을 못 보고 보정할 수 없다 — 옛 `blend_label` 이 ignore 셀을
배경으로 남긴 것과 같은 이유다.

`test_slab_render.py` 끝에 추가한다:

```python
def test_blend_slab_tints_only_visible_cells():
    ipm = np.full((2, 3, 3), 100, np.uint8)
    occ = np.array([[0, 1, 0], [1, 0, 1]], np.uint8)     # 0=obstacle 1=drivable
    vis = np.array([[1, 1, 0], [0, 1, 1]], np.uint8)
    out = sr.blend_slab(ipm, occ, vis, alpha=0.5)
    assert (out[0, 2] == 100).all()                       # vis=0 → IPM 원본 그대로
    assert (out[1, 0] == 100).all()
    assert not (out[0, 0] == 100).all()                   # vis=1 → 색이 얹힌다
    assert not (out[0, 1] == 100).all()
    assert tuple(out[0, 0]) != tuple(out[0, 1])           # obstacle 과 drivable 은 다른 색


def test_blend_slab_alpha_zero_is_untouched_ipm():
    ipm = np.full((2, 2, 3), 77, np.uint8)
    occ = np.zeros((2, 2), np.uint8)
    vis = np.ones((2, 2), np.uint8)
    assert (sr.blend_slab(ipm, occ, vis, alpha=0.0) == 77).all()
```

- [ ] **Step 2: 실패를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_render.py -q`
Expected: FAIL — `AttributeError: module 'slab_render' has no attribute 'blend_slab'`

- [ ] **Step 3: `blend_slab` 을 구현하고 `review_png` 에 IPM 패널을 붙인다**

`slab_render.py` 에 추가한다:

```python
def blend_slab(ipm, occupancy, visibility, alpha=0.45):
    """IPM RGB 캔버스에 슬래브 라벨을 반투명 오버레이. 네이티브 해상도·장식 없음.

    CVAT 등에 올려 그 위에서 라벨을 보정하는 annotation base 다. 격자·미터축은 셀 크기와
    맞먹어 네이티브에선 실제 셀을 덮으므로 넣지 않는다(확대 검수는 review_png).

    vis=0 셀은 IPM 원본을 그대로 남긴다 — 미관측 영역에 색을 얹으면 사람이 장면을 못 보고
    보정할 수 없다. 45/694 샘플은 vis 가 전역 0 이어서 이 규약 덕에 IPM 이 온전히 보인다.
    """
    over = np.asarray(ipm).copy()
    occ = np.asarray(occupancy)
    m = np.asarray(visibility).astype(bool)
    tint = np.zeros(over.shape, np.uint8)
    tint[occ == 0] = _C_VIS_OBS
    tint[occ == 1] = _C_VIS_DRIV
    over[m] = (alpha * tint[m] + (1 - alpha) * over[m]).astype(np.uint8)
    return over
```

`review_png` 를 확장한다. 기존 4색 BEV 패널 **옆에** IPM+라벨 패널을 나란히 두고, 카메라 행은
그 위에 전체 폭으로 얹는다. IPM 이 주어지지 않으면 지금 동작(4색 단독)을 유지해야 한다 —
`ipm` 을 선택 인자로 두고, 기존 테스트가 그대로 통과해야 한다.

두 패널을 같은 `scale` 로 그리고 각 패널에 `_grid_axes` 상당의 격자·미터축·ego 마커를 넣는다.
패널 제목을 얹어 어느 쪽이 무엇인지 알 수 있게 한다(`4color(occ+vis)` / `ipm+label`).

- [ ] **Step 4: 테스트 통과를 확인한다**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_slab_label.py test_slab_render.py -q`
Expected: PASS (37 passed — 기존 35개 + 새 2개). 기존 `review_png` 테스트가 수정 없이 통과해야 한다.

- [ ] **Step 5: CLI 에 IPM 산출을 붙인다**

`generate_slab.py` 에 옵션을 추가한다:

```
--cam-height 0.87        # IPM 지면 평면용 카메라 렌즈 높이[m] 실측값. IPM 정확도의 핵심
--blend nearest          # nearest(셀별 최근접 1대, 기본)|average
--alpha 0.45             # 오버레이 불투명도
--no-ipm                 # IPM 생성을 끈다(LiDAR 라벨만 빠르게 뽑을 때)
```

샘플마다 추가 저장:
- `ipm_rgb.png` — `ipm.ipm_canvas(...)` 결과 그대로
- `overlay.png` — `sr.blend_slab(ipm_rgb, occupancy, visibility, alpha)`. **네이티브 해상도·장식 없음** = CVAT annotation base
- `review.png` — 카메라 행 + [4색 BEV | IPM+라벨] 두 패널

`meta.json` 의 `params` 에 `cam_height`·`blend`·`alpha` 를 기록한다.

- [ ] **Step 6: raws3 로 실기 확인**

```bash
cd calibration/bev_autolabel && python3 generate_slab.py \
  --map-dir ../../data/sj_bags/260722/maps_selfmask/raws3_mapping \
  --extract-dir ../../data/extracted/raws3 \
  --calib ../../data/calib_260723/calib.yaml \
  --orient ../../data/calib_260723/orientation.json \
  --self-mask-dir ../../data/calib_260723/self_mask \
  --out ../../data/bev/slab/raws3
python3 slab_sheet.py ../../data/bev/slab/raws3
```

판정: `ipm_rgb.png` 에 통로 바닥이 펴져 보이고, `overlay.png` 가 네이티브 해상도(NX×NY)이며
`review.png` 에 두 패널이 나란히 있다. **`visibility` 전역 0 샘플**(예: `raws1/sample_000067`)에서
`overlay.png` 가 색 없이 IPM 원본을 온전히 보여주는지 확인한다 — 사람이 그 위에 직접 그려야 하므로
이게 핵심이다. LiDAR 라벨 수치(`z_ref`·`obstacle`·`cam_ok`·`reach`)는 이전 실행과 동일해야 한다
(IPM 추가가 라벨을 바꾸면 안 된다).

- [ ] **Step 7: 알려진 한계를 문서에 기록한다**

`docs/BEV_AUTOLABEL.md` §B 에 하위 절을 추가한다. 실측값으로 쓴다:

```markdown
### 알려진 한계 — visibility 전역 0 샘플 (694 중 45개, 6.5%)

`visibility` 가 전역 0 이라 학습에서 통째로 마스킹되는 샘플이 있다. bag 별 분포:
raws1 14 / raws2 6 / raws3 **0** / rawos1 3 / rawos2 19 / rawos3 2 / rawos4 1.
그중 ego 셀 자체가 obstacle 인 경우가 25개(3.6%)다.

**메커니즘**: `raycast_visible` 은 ego 셀에서 출발해 첫 obstacle 셀에서 멈춘다. 그래서 ego 셀이
obstacle 이면 모든 ray 가 즉시 끊겨 visibility 가 전역 0 이 된다.

**원인은 셋이 겹친 것**이며 기하로 고칠 수 없다.
1. **좁은 통로**: 해당 bag 의 obstacle 셀 비율이 애초에 높다(raws1 26.9%·rawos2 28.1% vs raws3 22.2%).
2. **매핑 드리프트**: 낮은 reach 샘플이 연속 구간으로 뭉쳐 나온다(raws1 66~70·74·78).
3. **잎의 불규칙한 돌출**: 슬래브가 실제 0.87~1.67m 밴드라, 그 높이에서 잎이 통로로 넘어오면
   BEV 에서 통로가 실제보다 좁게 찍힌다. 바닥은 비어 있는데도 그렇다.

**ego 반경을 비우는 수정은 하지 않는다.** 실측: raycast 도달률이 0.9% → (r0.3 비움) 5.9%,
신뢰영역(r>0.7)만 보면 3.9% 다. 정상 샘플이 18~25% 이므로 회복이 미미하다. 지금처럼
`visibility=0` 으로 두면 라벨이 "여기서 학습하지 마라"를 정직하게 말하는데, 4% 짜리 어중간한
라벨로 바꾸면 걸러내기 어려워진다 — 명확한 실패가 애매한 성공이 되는 쪽이 QC 에 더 나쁘다.

**대응**: `overlay.png`(IPM+라벨) 위에서 사람이 보정한다. vis=0 셀은 색이 얹히지 않아 IPM 원본이
그대로 보이므로 그 영역을 직접 그릴 수 있다. 찾는 방법:

    awk '$4+0 < 1.0 {print $1, $4}' data/bev/slab/<name>/_stats.txt   # vis% < 1.0

**reach 가 낮은 bag 의 해석**: raws1 21%·rawos2 14% 는 4.0m 도달률이 낮지만, 그중 대부분이
이 현상이다(raws1 낮은 reach 20건 중 14건, rawos2 23건 중 19건). 통로가 좁은 것도 사실이고
그 위에 드리프트·잎 돌출이 겹친 결과다 — 어느 하나로 환원되지 않는다.
```

`docs/PIPELINE.md` 5단계에 슬래브 경로 안내를 추가한다(기존 `generate.py` 설명은 유지하고,
LiDAR 라벨 현행판이 `generate_slab.py` 이며 상세는 `BEV_AUTOLABEL.md §B` 임을 명시).

- [ ] **Step 8: 전체 테스트 후 커밋**

```bash
cd src/econ_camera_ros && python3 -m pytest test/ -q
cd ../../calibration/bev_autolabel && python3 -m pytest -q
git add calibration/bev_autolabel/slab_render.py calibration/bev_autolabel/test_slab_render.py \
        calibration/bev_autolabel/generate_slab.py docs/BEV_AUTOLABEL.md docs/PIPELINE.md
git commit -m "feat(bev): 슬래브 라벨에 IPM RGB 오버레이 + 알려진 한계 기록"
```

---

## 미확정 사항 (구현 중 사용자 확인 필요)

1. **`--thick 0.8`**: 로봇 높이 실측값으로 확정해야 한다. 슬래브 `[z_ref, z_ref+thick]` 은
   실세계 약 0.87~(0.87+thick)m 밴드다.
2. **`--ground-offset 0.87`**: IPM 의 `cam_height`(카메라 렌즈의 바닥 위 높이)에서 가져왔다.
   LiDAR 높이와 카메라 높이가 다르면 별도 실측값이 필요하다. 이 값은 카메라 관측가능성
   판정 평면에만 쓰인다.
3. **self 마스크**: 사용자가 제작 중. 없어도 Task 1~6 은 전부 진행·검증 가능하고, 준비되면
   Task 7 을 다시 돌리면 된다.
