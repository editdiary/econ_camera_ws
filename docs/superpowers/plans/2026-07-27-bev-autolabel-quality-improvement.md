# BEV Auto-Label 품질 개선 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** BEV auto-label의 obstacle 경계 정확도와 "보이는 곳만" 규칙을 개선한다(수직성 테스트 · 카메라 FoV observed · 정밀 ego 제거).

**Architecture:** 순수 라벨 로직(`bev_label.py`, numpy/cv2만, 하드웨어·파일 불필요)과 IO/렌더/CLI를 분리한다. 라벨 계산은 LIO 맵(월드 클라우드)+궤적+calib을 입력받아 80×80 라벨(0/1/2)을 만드는 순수 함수들로 구성하고, `calibration/cam_lidar`·`calibration/verify`의 기존 모듈을 재사용한다. 단계1(품질 검증용 합성 PNG)로 파라미터를 튜닝한 뒤 단계2(데이터셋 CLI)로 확장한다.

**Tech Stack:** Python3, numpy 1.21.5, scipy 1.8.0, opencv 4.5.4, open3d 0.18(user-site), Pillow 9.0.1. pytest(순수 로직).

## Global Constraints

- **환경 핀 절대 준수**: 시스템 scipy가 numpy<1.25 고정. user-site에 numpy≥2 / opencv-python≥4.10 설치 금지. PCD는 open3d 0.18(user-site) 사용. (spec §4, `BEV_AUTOLABEL.md §7-I`)
- **BEV 규격 고정**: `XF=3.0, XR=1.0, YH=2.0, RES=0.05` → `NX=NY=80`, ego셀 `(R_EGO=60, C_EGO=40)`. (spec §2)
- **좌표계**: ego=LIO body=LiDAR 프레임, `x=전방,y=좌,z=상`. TUM pose = `world_T_body`. BEV row=`(XF−x)/RES`, col=`(YH−y)/RES`. (spec §2, `BEV_AUTOLABEL.md §3`)
- **라벨 클래스**: `0=obstacle, 1=drivable, 2=ignore`. 입력 카메라 `front/left/right`(rear 제외). (spec §1)
- **라벨은 LiDAR+맵 기하로만 생성**. 이미지는 observed(FoV) 마스크와 사람 검수에만 사용. (spec §1)
- **모듈 재사용(복붙 금지)**: `calibration/cam_lidar/{chain,cloud_io,calib_io}.py`, `calibration/verify/ds_model.py`. sys.path 삽입 패턴은 `calibration/cam_lidar/test_cam_lidar.py`를 따른다.
- **커밋 규약**: 새 기능은 현재 브랜치 `feat/bev-autolabel`에서 작업. 커밋 메시지 말미에 `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. 병합·푸시는 사용자가 직접.

## File Structure

- `calibration/bev_autolabel/__init__.py` — 빈 파일(패키지 표식은 불필요하나 import 편의).
- `calibration/bev_autolabel/bev_label.py` — **순수 라벨 로직**. `BevSpec`, 좌표 헬퍼, `floor_grid`, `obstacle_mask`, `fov_mask`, `raycast_visible`, `compute_self_voxels`, `corridor_mask`, `keep_ego_connected`, `assemble_label`, `build_label`.
- `calibration/bev_autolabel/bev_io.py` — IO. 맵/궤적 로드, sets.csv stamp 로드, 이미지 로드. open3d/cv2 지연 import.
- `calibration/bev_autolabel/render.py` — `colorize`(라벨→RGB), `review_image`(미터축·격자·ego·화살표 + 3카메라 합성).
- `calibration/bev_autolabel/verify_labels.py` — **단계1 CLI**: 지정 프레임들의 합성 검증 PNG 생성.
- `calibration/bev_autolabel/generate.py` — **단계2 CLI**: 키프레임 샘플링 → 데이터셋 출력.
- `calibration/bev_autolabel/test_bev_label.py` — 순수 로직 pytest.

**공통 import 헤더**(bev_label.py, bev_io.py, render.py 상단, 필요한 것만):
```python
import sys, pathlib
_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
```

---

### Task 1: 패키지 스캐폴드 + BevSpec + 좌표 헬퍼

**Files:**
- Create: `calibration/bev_autolabel/__init__.py`
- Create: `calibration/bev_autolabel/bev_label.py`
- Test: `calibration/bev_autolabel/test_bev_label.py`

**Interfaces:**
- Produces:
  - `BevSpec` dataclass with fields `XF,XR,YH,RES` and properties `NX,NY,R_EGO,C_EGO` (all int).
  - `rc_of(x, y, spec) -> (row, col)` — x,y(scalar or ndarray) → floor 기반 int row/col(같은 shape).
  - `cell_centers(spec) -> (X, Y)` — 각 (NX,NY) 셀 중심의 ego x,y(float ndarray).
  - `_key3(C) -> ndarray[int64]` — (M,3) int 복셀좌표 → 유니크 해시키.

- [ ] **Step 1: Write the failing test**

`calibration/bev_autolabel/test_bev_label.py`:
```python
"""순수 로직 테스트(ROS·이미지·bag·calib 파일 불필요). 실행: cd calibration/bev_autolabel && python3 -m pytest -q"""
import numpy as np
from bev_label import BevSpec, rc_of, cell_centers, _key3


def test_bevspec_derived_shapes():
    s = BevSpec()
    assert (s.NX, s.NY) == (80, 80)
    assert (s.R_EGO, s.C_EGO) == (60, 40)


def test_rc_of_ego_origin():
    s = BevSpec()
    r, c = rc_of(0.0, 0.0, s)          # ego 원점(x=0,y=0)
    assert (int(r), int(c)) == (60, 40)


def test_rc_of_forward_is_up():
    s = BevSpec()
    r_fwd, _ = rc_of(2.0, 0.0, s)      # 전방 2m → row 작아짐(위)
    r_ego, _ = rc_of(0.0, 0.0, s)
    assert r_fwd < r_ego


def test_cell_centers_roundtrip():
    s = BevSpec()
    X, Y = cell_centers(s)
    r, c = rc_of(X, Y, s)
    rr, cc = np.meshgrid(np.arange(s.NX), np.arange(s.NY), indexing="ij")
    assert np.array_equal(r, rr) and np.array_equal(c, cc)


def test_key3_unique():
    C = np.array([[1, 2, 3], [1, 2, 3], [1, 2, 4]])
    k = _key3(C)
    assert k[0] == k[1] and k[0] != k[2]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'bev_label'`

- [ ] **Step 3: Write minimal implementation**

`calibration/bev_autolabel/__init__.py`: (빈 파일)

`calibration/bev_autolabel/bev_label.py`:
```python
"""BEV auto-label 순수 로직(LiDAR+맵 기하). 하드웨어·파일 불필요.

좌표계: ego=body=LiDAR, x=전방,y=좌,z=상. BEV row=(XF-x)/RES, col=(YH-y)/RES.
라벨: 0=obstacle, 1=drivable, 2=ignore.
"""
from __future__ import annotations

import sys
import pathlib
from dataclasses import dataclass

import numpy as np
import cv2

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
from chain import se3_inv, transform, project  # noqa: E402
from cloud_io import pose_at                     # noqa: E402


@dataclass
class BevSpec:
    XF: float = 3.0
    XR: float = 1.0
    YH: float = 2.0
    RES: float = 0.05

    @property
    def NX(self) -> int:
        return int(round((self.XF + self.XR) / self.RES))

    @property
    def NY(self) -> int:
        return int(round(2 * self.YH / self.RES))

    @property
    def R_EGO(self) -> int:
        return int(round(self.XF / self.RES))

    @property
    def C_EGO(self) -> int:
        return int(round(self.YH / self.RES))


def rc_of(x, y, spec):
    """ego x,y(스칼라/ndarray) → floor 기반 int row,col(같은 shape)."""
    r = np.floor((spec.XF - np.asarray(x, float)) / spec.RES).astype(int)
    c = np.floor((spec.YH - np.asarray(y, float)) / spec.RES).astype(int)
    return r, c


def cell_centers(spec):
    """각 (NX,NY) 셀 중심의 ego x,y."""
    r = np.arange(spec.NX)
    c = np.arange(spec.NY)
    x = spec.XF - (r + 0.5) * spec.RES
    y = spec.YH - (c + 0.5) * spec.RES
    return np.meshgrid(x, y, indexing="ij")


def _key3(C):
    """(M,3) int 복셀좌표 → 유니크 해시키(int64). PoC와 동일 규약."""
    C = np.asarray(C)
    return (C[:, 0] + 100) * 1_000_000 + (C[:, 1] + 100) * 1000 + (C[:, 2] + 100)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py -q`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
git add calibration/bev_autolabel/__init__.py calibration/bev_autolabel/bev_label.py calibration/bev_autolabel/test_bev_label.py
git commit -m "feat(bev): bev_autolabel 패키지 스캐폴드 + BevSpec·좌표 헬퍼

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: 국소 floor + 수직성 obstacle

**Files:**
- Modify: `calibration/bev_autolabel/bev_label.py` (append)
- Test: `calibration/bev_autolabel/test_bev_label.py` (append)

**Interfaces:**
- Consumes: `BevSpec`, `rc_of`.
- Produces:
  - `floor_grid(P, spec, win=1.0, pct=2.0) -> ndarray(NX,NY) float` — 국소 저-퍼센타일 바닥 격자(빈 윈도는 전역 저-퍼센타일로 채움).
  - `obstacle_mask(P, floor, spec, z_gate=0.3, min_extent=0.5, min_pts=2) -> ndarray(NX,NY) bool` — 수직성 테스트 + morph open/close. `P`는 ego 프레임 (M,3).

- [ ] **Step 1: Write the failing test**

`test_bev_label.py`에 추가:
```python
from bev_label import floor_grid, obstacle_mask


def _column(x, y, z0, z1, n=40):
    z = np.linspace(z0, z1, n)
    return np.stack([np.full(n, x), np.full(n, y), z], -1)


def test_floor_grid_flat_ground():
    s = BevSpec()
    # z=0 평면에 흩뿌린 점
    xy = np.random.RandomState(0).uniform(-1, 2, (500, 2))
    P = np.hstack([xy, np.zeros((500, 1))])
    f = floor_grid(P, s)
    assert f.shape == (s.NX, s.NY)
    assert abs(np.median(f)) < 0.05


def test_obstacle_column_detected():
    s = BevSpec()
    floor = np.zeros((s.NX, s.NY))
    P = _column(1.0, 0.0, 0.05, 1.5)          # 바닥까지 이어지는 수직 기둥
    obs = obstacle_mask(P, floor, s)
    r, c = rc_of(1.0, 0.0, s)
    assert obs[int(r), int(c)]


def test_ceiling_only_not_obstacle():
    s = BevSpec()
    floor = np.zeros((s.NX, s.NY))
    # 높이 2.0~2.3m 에만 떠 있는 천장 클러스터(바닥까지 안 이어짐)
    P = np.repeat(_column(0.5, 1.0, 2.0, 2.3, n=10), 1, axis=0)
    obs = obstacle_mask(P, floor, s)
    r, c = rc_of(0.5, 1.0, s)
    assert not obs[int(r), int(c)]


def test_floor_noise_speck_not_obstacle():
    s = BevSpec()
    floor = np.zeros((s.NX, s.NY))
    # 바닥 근처 단발 점 몇 개(수직 extent 없음)
    P = np.array([[0.5, 0.5, 0.02], [0.5, 0.5, 0.05], [0.5, 0.5, 0.08]])
    obs = obstacle_mask(P, floor, s)
    r, c = rc_of(0.5, 0.5, s)
    assert not obs[int(r), int(c)]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py -q`
Expected: FAIL — `ImportError: cannot import name 'floor_grid'`

- [ ] **Step 3: Write minimal implementation**

`bev_label.py`에 추가:
```python
def floor_grid(P, spec, win=1.0, pct=2.0):
    """국소 저-퍼센타일 바닥 격자. 빈 윈도는 전역 저-퍼센타일로 채움."""
    NX, NY = spec.NX, spec.NY
    floor = np.full((NX, NY), np.nan)
    if len(P) == 0:
        return np.zeros((NX, NY))
    r, c = rc_of(P[:, 0], P[:, 1], spec)
    inb = (r >= 0) & (r < NX) & (c >= 0) & (c < NY)
    r, c, z = r[inb], c[inb], P[inb, 2]
    wc = max(1, int(round(win / spec.RES)))
    gwr, gwc = r // wc, c // wc
    for wr in range(0, NX, wc):
        for wcol in range(0, NY, wc):
            m = (gwr == wr // wc) & (gwc == wcol // wc)
            if int(m.sum()) >= 3:
                floor[wr:wr + wc, wcol:wcol + wc] = np.percentile(z[m], pct)
    g = np.percentile(z, pct) if len(z) else 0.0
    floor[np.isnan(floor)] = g
    return floor


def obstacle_mask(P, floor, spec, z_gate=0.3, min_extent=0.5, min_pts=2):
    """수직성 테스트: 셀 점들이 바닥까지 이어지고(z_min<=floor+z_gate) 세로로 길면(z_max-z_min>=min_extent) obstacle."""
    NX, NY = spec.NX, spec.NY
    zmin = np.full((NX, NY), np.inf)
    zmax = np.full((NX, NY), -np.inf)
    cnt = np.zeros((NX, NY), int)
    if len(P):
        r, c = rc_of(P[:, 0], P[:, 1], spec)
        inb = (r >= 0) & (r < NX) & (c >= 0) & (c < NY)
        r, c, z = r[inb], c[inb], P[inb, 2]
        np.minimum.at(zmin, (r, c), z)
        np.maximum.at(zmax, (r, c), z)
        np.add.at(cnt, (r, c), 1)
    reaches = zmin <= (floor + z_gate)
    extent = (zmax - zmin) >= min_extent
    obs = (reaches & extent & (cnt >= min_pts)).astype(np.uint8)
    k = np.ones((3, 3), np.uint8)
    obs = cv2.morphologyEx(obs, cv2.MORPH_OPEN, k)
    obs = cv2.morphologyEx(obs, cv2.MORPH_CLOSE, k)
    return obs.astype(bool)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py -q`
Expected: PASS (9 passed)
주의: `test_obstacle_column_detected`는 morph OPEN(3×3)이 단일 셀을 지울 수 있으므로, 통과가 안 되면 컬럼을 3×3 셀 블록으로 만든다 — 아래 보강 참고.

보강(단일 셀이 OPEN에 지워지면 테스트를 이렇게 수정): 컬럼을 인접 셀로 퍼뜨린다.
```python
def test_obstacle_column_detected():
    s = BevSpec()
    floor = np.zeros((s.NX, s.NY))
    cols = [ _column(1.0 + dx, 0.0 + dy, 0.05, 1.5)
             for dx in (-0.05, 0, 0.05) for dy in (-0.05, 0, 0.05) ]
    P = np.vstack(cols)
    obs = obstacle_mask(P, floor, s)
    r, c = rc_of(1.0, 0.0, s)
    assert obs[int(r), int(c)]
```

- [ ] **Step 5: Commit**

```bash
git add calibration/bev_autolabel/bev_label.py calibration/bev_autolabel/test_bev_label.py
git commit -m "feat(bev): 국소 floor 격자 + 수직성(column) obstacle 판정

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: 카메라 FoV 마스크 + ray-cast 가림

**Files:**
- Modify: `calibration/bev_autolabel/bev_label.py` (append)
- Test: `calibration/bev_autolabel/test_bev_label.py` (append)

**Interfaces:**
- Consumes: `BevSpec`, `cell_centers`, `chain.project`, `ds_model.DoubleSphereCamera`.
- Produces:
  - `fov_mask(floor, spec, cams, T_cam_front, T_front_lidar, use_names=("front","left","right")) -> ndarray(NX,NY) bool` — 각 셀 지면점을 카메라로 투영해 이미지 안+DS 유효면 True. `cams`=name→DoubleSphereCamera, `T_cam_front`=name→4x4.
  - `raycast_visible(obstacle, spec, step_deg=0.5) -> ndarray(NX,NY) bool` — ego셀에서 광선 쏴 첫 obstacle까지 True.

- [ ] **Step 1: Write the failing test**

`test_bev_label.py`에 추가:
```python
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent.parent / "verify"))
from ds_model import DoubleSphereCamera
from bev_label import fov_mask, raycast_visible


def _front_cam():
    # 전방을 +z로 보는 표준 핀홀 유사 DS(alpha=0.5, xi=0): z>0 만 유효
    return DoubleSphereCamera(xi=0.0, alpha=0.5, fx=300, fy=300, cx=640, cy=360,
                              width=1280, height=720, name="front")


def test_fov_forward_visible_backward_not():
    s = BevSpec()
    floor = np.zeros((s.NX, s.NY))
    cams = {"front": _front_cam()}
    # front 카메라: LiDAR +x(전방)를 카메라 +z로 보내는 회전 T_cam_front
    Tcf = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]], float)
    fov = fov_mask(floor, s, cams, {"front": Tcf}, np.eye(4), use_names=("front",))
    r_f, c_f = rc_of(2.0, 0.0, s)     # 전방 2m → 보여야
    r_b, c_b = rc_of(-0.5, 0.0, s)    # 후방 → 안 보여야(카메라 뒤)
    assert fov[int(r_f), int(c_f)]
    assert not fov[int(r_b), int(c_b)]


def test_raycast_blocks_behind_wall():
    s = BevSpec()
    obs = np.zeros((s.NX, s.NY), bool)
    # ego 앞(전방 1m) 가로벽
    rw, _ = rc_of(1.0, 0.0, s)
    obs[int(rw), :] = True
    vis = raycast_visible(obs, s)
    r_near, c_near = rc_of(0.5, 0.0, s)   # 벽 앞: 보임
    r_far, c_far = rc_of(2.0, 0.0, s)     # 벽 뒤: 가려짐
    assert vis[int(r_near), int(c_near)]
    assert not vis[int(r_far), int(c_far)]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py -q`
Expected: FAIL — `ImportError: cannot import name 'fov_mask'`

- [ ] **Step 3: Write minimal implementation**

`bev_label.py`에 추가:
```python
def fov_mask(floor, spec, cams, T_cam_front, T_front_lidar,
             use_names=("front", "left", "right")):
    """각 BEV 셀 지면점을 카메라로 투영. 이미지 안 & DS 유효면 그 카메라 FoV."""
    X, Y = cell_centers(spec)
    P = np.stack([X, Y, floor], axis=-1).reshape(-1, 3)   # ego(=lidar) 프레임 지면점
    mask = np.zeros(spec.NX * spec.NY, bool)
    for name in use_names:
        cam = cams[name]
        u, v, ok = project(P, T_front_lidar, T_cam_front[name], cam)
        inimg = ok & (u >= 0) & (u < cam.width) & (v >= 0) & (v < cam.height)
        mask |= inimg
    return mask.reshape(spec.NX, spec.NY)


def raycast_visible(obstacle, spec, step_deg=0.5):
    """ego셀에서 0.5° 간격 광선 → 첫 obstacle까지 visible."""
    NX, NY = spec.NX, spec.NY
    vis = np.zeros((NX, NY), bool)
    for a in np.deg2rad(np.arange(0, 360, step_deg)):
        dr, dc = np.cos(a), np.sin(a)
        for rr in np.arange(0.0, NX + NY, 0.5):
            r = int(round(spec.R_EGO + dr * rr))
            c = int(round(spec.C_EGO + dc * rr))
            if not (0 <= r < NX and 0 <= c < NY):
                break
            vis[r, c] = True
            if obstacle[r, c]:
                break
    return vis
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py -q`
Expected: PASS (11 passed)

- [ ] **Step 5: Commit**

```bash
git add calibration/bev_autolabel/bev_label.py calibration/bev_autolabel/test_bev_label.py
git commit -m "feat(bev): 카메라 FoV 마스크 + ray-cast 가림

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: 정밀 ego self-mask + corridor + 라벨 조립

**Files:**
- Modify: `calibration/bev_autolabel/bev_label.py` (append)
- Test: `calibration/bev_autolabel/test_bev_label.py` (append)

**Interfaces:**
- Consumes: `BevSpec`, `rc_of`, `_key3`, `chain.se3_inv/transform`.
- Produces:
  - `compute_self_voxels(p, poses, radius=0.28, vox=0.15, z_abs=1.5, n_samples=150, persist=0.6, near_r=1.7) -> ndarray[int64]` — 카트 복셀키 집합(월드 클라우드 `p`(N,3), `poses`=list[4x4]).
  - `corridor_mask(tpos_ego, spec, r_traj=0.45, r_ego=0.3, x_min=-0.2) -> ndarray(NX,NY) bool` — 전방 궤적점 원 + ego 원.
  - `keep_ego_connected(drivable, spec) -> ndarray(NX,NY) bool` — ego셀과 연결된 성분만.
  - `assemble_label(observed, obs_rc, corridor, spec) -> ndarray(NX,NY) uint8` — 0/1/2 조립 + ego 연결 정리.

- [ ] **Step 1: Write the failing test**

`test_bev_label.py`에 추가:
```python
from bev_label import compute_self_voxels, corridor_mask, keep_ego_connected, assemble_label


def test_self_voxels_catches_close_not_far():
    # 모든 pose에서 body 원점 근처(0.1m)에 카트 점, 0.7m 옆에 기둥 점.
    # pose는 x축으로 전진(회전 없음).
    poses = []
    for i in range(20):
        T = np.eye(4); T[0, 3] = i * 0.3
        poses.append(T)
    cart = np.array([[i * 0.3 + 0.1, 0.0, 0.0] for i in range(20)])   # body +0.1m 지속
    pillar = np.array([[i * 0.3, 0.7, 0.0] for i in range(20)])       # body +0.7m 지속
    p = np.vstack([cart, pillar])
    SELF = compute_self_voxels(p, poses, radius=0.28)
    # 카트(0.1m)는 SELF, 기둥(0.7m)은 반경 밖이라 아님
    from bev_label import _key3
    cart_key = _key3(np.floor(np.array([[0.1, 0.0, 0.0]]) / 0.15).astype(int))[0]
    pillar_key = _key3(np.floor(np.array([[0.0, 0.7, 0.0]]) / 0.15).astype(int))[0]
    assert cart_key in set(SELF.tolist())
    assert pillar_key not in set(SELF.tolist())


def test_corridor_forward_only():
    s = BevSpec()
    TE = np.array([[1.0, 0.0, 0.0], [-0.5, 0.0, 0.0]])   # 전방 1m, 후방 0.5m
    corr = corridor_mask(TE, s)
    r_f, c_f = rc_of(1.0, 0.0, s)
    r_b, c_b = rc_of(-0.5, 0.0, s)
    assert corr[int(r_f), int(c_f)]        # 전방 마킹
    assert not corr[int(r_b), int(c_b)]    # 후방 미마킹(단, ego 원과 안 겹치는 위치)


def test_keep_ego_connected_drops_island():
    s = BevSpec()
    d = np.zeros((s.NX, s.NY), bool)
    d[s.R_EGO, s.C_EGO] = True
    d[s.R_EGO - 1, s.C_EGO] = True         # ego 연결
    d[0, 0] = True                         # 떨어진 섬
    out = keep_ego_connected(d, s)
    assert out[s.R_EGO, s.C_EGO] and not out[0, 0]


def test_assemble_label_values():
    s = BevSpec()
    observed = np.zeros((s.NX, s.NY), bool)
    observed[s.R_EGO - 5:s.R_EGO + 1, s.C_EGO] = True   # ego~전방 한 줄 관측
    obs_rc = np.zeros((s.NX, s.NY), bool)
    obs_rc[s.R_EGO - 5, s.C_EGO] = True                 # 관측 줄 끝에 장애물
    corridor = np.zeros((s.NX, s.NY), bool)
    lab = assemble_label(observed, obs_rc, corridor, s)
    assert lab[s.R_EGO - 5, s.C_EGO] == 0               # obstacle
    assert lab[s.R_EGO - 1, s.C_EGO] == 1               # drivable
    assert lab[0, 0] == 2                               # 미관측 ignore
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py -q`
Expected: FAIL — `ImportError: cannot import name 'compute_self_voxels'`

- [ ] **Step 3: Write minimal implementation**

`bev_label.py`에 추가:
```python
def compute_self_voxels(p, poses, radius=0.28, vox=0.15, z_abs=1.5,
                        n_samples=150, persist=0.6, near_r=1.7):
    """body 좌표에서 반경<radius·|z|<z_abs 로 >persist pose 지속 복셀 = 카트(self)."""
    from collections import Counter
    from scipy.spatial import cKDTree
    tree = cKDTree(p[:, :2])
    cnt = Counter()
    n = min(n_samples, len(poses))
    samp = np.linspace(0, len(poses) - 1, n).astype(int)
    for i in samp:
        Tbw = se3_inv(poses[i])
        idx = tree.query_ball_point(poses[i][:3, 3][:2], r=near_r)
        if not idx:
            continue
        Pb = transform(Tbw, p[idx, :3])
        b = (np.hypot(Pb[:, 0], Pb[:, 1]) < radius) & (np.abs(Pb[:, 2]) < z_abs)
        for k in set(_key3(np.floor(Pb[b, :3] / vox).astype(int)).tolist()):
            cnt[k] += 1
    return np.array([k for k, c in cnt.items() if c / len(samp) > persist], np.int64)


def corridor_mask(tpos_ego, spec, r_traj=0.45, r_ego=0.3, x_min=-0.2):
    """전방(x>=x_min) 궤적점 원 + ego 원을 drivable prior로."""
    corr = np.zeros((spec.NX, spec.NY), np.uint8)
    for x, y in np.asarray(tpos_ego)[:, :2]:
        if x < x_min:
            continue
        r, c = rc_of(x, y, spec)
        if 0 <= int(r) < spec.NX and 0 <= int(c) < spec.NY:
            cv2.circle(corr, (int(c), int(r)), max(1, int(r_traj / spec.RES)), 1, -1)
    cv2.circle(corr, (spec.C_EGO, spec.R_EGO), int(r_ego / spec.RES), 1, -1)
    return corr.astype(bool)


def keep_ego_connected(drivable, spec):
    """ego셀과 연결된 drivable 성분만 남김."""
    num, lbl = cv2.connectedComponents(drivable.astype(np.uint8))
    ego = lbl[spec.R_EGO, spec.C_EGO]
    if ego == 0:
        return np.zeros_like(drivable, bool)
    return lbl == ego


def assemble_label(observed, obs_rc, corridor, spec):
    """0=obstacle,1=drivable,2=ignore 조립 + ego 연결 drivable 정리."""
    label = np.full((spec.NX, spec.NY), 2, np.uint8)
    label[observed & ~obs_rc] = 1
    label[observed & obs_rc] = 0
    label[corridor & observed] = 1
    driv = keep_ego_connected(label == 1, spec)
    label[(label == 1) & ~driv] = 2
    return label
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py -q`
Expected: PASS (15 passed)
주의: `test_corridor_forward_only`에서 후방 셀이 ego 원(r_ego=0.3m=6셀)에 안 겹치도록 -0.5m(=10셀 뒤)로 두었다. 실패 시 후방점을 -1.0m로 더 멀리.

- [ ] **Step 5: Commit**

```bash
git add calibration/bev_autolabel/bev_label.py calibration/bev_autolabel/test_bev_label.py
git commit -m "feat(bev): 정밀 ego self-mask(반경0.28) + corridor + 라벨 조립·연결정리

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: build_label 오케스트레이션 + IO 로더

**Files:**
- Modify: `calibration/bev_autolabel/bev_label.py` (append `build_label`)
- Create: `calibration/bev_autolabel/bev_io.py`
- Test: `calibration/bev_autolabel/test_bev_label.py` (append `build_label` 통합 테스트)

**Interfaces:**
- Consumes: 위 모든 순수 함수, `cloud_io.pose_at`, `chain.se3_inv/transform`.
- Produces:
  - `build_label(p, times_ns, poses, tpos, self_voxels, cams, T_cam_front, T_front_lidar, t_ns, spec, near=6.0, vox=0.15, win_s=20.0) -> ndarray(NX,NY) uint8` — 한 키프레임(카메라 stamp `t_ns`) 라벨. `p`=월드 클라우드(N,3), `tpos`=pose 위치(T,3).
  - `bev_io.load_map(map_dir) -> (p(N,3) float64, times_ns(int64), poses list[4x4], tpos(T,3))`.
  - `bev_io.load_stamps(extract_dir) -> dict{idx:int -> stamp_ns:int}` (sets.csv의 stamp0).
  - `bev_io.load_cam_images(extract_dir, idx, idx_to_name, use_names) -> dict{name -> BGR ndarray}`.

- [ ] **Step 1: Write the failing test**

`test_bev_label.py`에 추가(합성 씬 — 파일 불필요):
```python
from bev_label import build_label


def test_build_label_synthetic_corridor():
    """직선 통로 합성: 좌우 벽(작물), 가운데 빈 길. front 카메라만. 라벨이 3값을 모두 포함."""
    s = BevSpec()
    # 월드=ego(pose=단위, 원점 정지). 좌우 y=±1.0 에 수직 벽, x 0~3m.
    walls = []
    for x in np.arange(0, 3, 0.05):
        for z in np.arange(0.05, 1.6, 0.05):
            walls.append([x, 1.0, z]); walls.append([x, -1.0, z])
    p = np.array(walls, float)
    times = np.array([0, int(1e9)], np.int64)
    poses = [np.eye(4), np.eye(4)]
    tpos = np.array([[0, 0, 0], [0.5, 0, 0]], float)
    cams = {"front": _front_cam()}
    Tcf = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]], float)
    lab = build_label(p, times, poses, tpos, np.array([], np.int64),
                      cams, {"front": Tcf}, np.eye(4), 0, s,
                      )
    assert lab.shape == (80, 80)
    assert (lab == 0).any()      # 벽 = obstacle
    assert (lab == 1).any()      # 통로 = drivable
    assert (lab == 2).any()      # 후방/밖 = ignore
    # 가운데 전방은 drivable, 좌우 벽 위치는 obstacle
    assert lab[s.R_EGO - 20, s.C_EGO] == 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py::test_build_label_synthetic_corridor -q`
Expected: FAIL — `ImportError: cannot import name 'build_label'`

- [ ] **Step 3: Write minimal implementation**

`bev_label.py`에 추가:
```python
def build_label(p, times_ns, poses, tpos, self_voxels, cams, T_cam_front,
                T_front_lidar, t_ns, spec, near=6.0, vox=0.15, win_s=20.0):
    """한 키프레임 라벨(0/1/2). p=월드 클라우드(N,3), tpos=pose 위치(T,3)."""
    T_wb = pose_at(times_ns, poses, t_ns)
    T_bw = se3_inv(T_wb)
    ctr = T_wb[:3, 3]
    near_m = (np.abs(p[:, 0] - ctr[0]) < near) & (np.abs(p[:, 1] - ctr[1]) < near)
    P = transform(T_bw, p[near_m])
    if len(self_voxels):
        keys = _key3(np.floor(P / vox).astype(int))
        P = P[~np.isin(keys, self_voxels)]
    crop = (P[:, 0] <= spec.XF) & (P[:, 0] >= -spec.XR) & (np.abs(P[:, 1]) <= spec.YH)
    P = P[crop]
    floor = floor_grid(P, spec)
    obstacle = obstacle_mask(P, floor, spec)
    tw = np.abs(times_ns - t_ns) < int(win_s * 1e9)
    TE = transform(T_bw, tpos[tw]) if tw.any() else np.empty((0, 3))
    corridor = corridor_mask(TE, spec)
    obs_rc = obstacle & ~corridor
    fov = fov_mask(floor, spec, cams, T_cam_front, T_front_lidar)
    visible = raycast_visible(obs_rc, spec)
    observed = fov & visible
    return assemble_label(observed, obs_rc, corridor, spec)
```

`calibration/bev_autolabel/bev_io.py`:
```python
"""BEV auto-label IO: 맵/궤적/stamp/이미지 로드. open3d·cv2 지연 import."""
from __future__ import annotations

import csv
import pathlib

import numpy as np

import sys
_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
from cloud_io import load_tum  # noqa: E402


def load_map(map_dir):
    """map.pcd + trajectory.tum → (p(N,3), times_ns, poses, tpos(T,3))."""
    import open3d as o3d
    md = pathlib.Path(map_dir)
    p = np.asarray(o3d.io.read_point_cloud(str(md / "map.pcd")).points, float)
    times_ns, poses = load_tum(str(md / "trajectory.tum"))
    tpos = np.array([P[:3, 3] for P in poses], float)
    return p, times_ns, poses, tpos


def load_stamps(extract_dir):
    """sets.csv → {idx: stamp_ns}. stamp0(=front 기준) 을 ns 로."""
    out = {}
    with open(pathlib.Path(extract_dir) / "sets.csv") as f:
        for r in csv.DictReader(f):
            out[int(r["idx"])] = int(round(float(r["stamp0"]) * 1e9))
    return out


def load_cam_images(extract_dir, idx, idx_to_name, use_names):
    """frame_NNNNNN/cam{k}.jpg → {name: BGR ndarray}. rig.idx_to_name 로 name 매핑."""
    import cv2
    ed = pathlib.Path(extract_dir)
    name2idx = {v: k for k, v in idx_to_name.items()}
    out = {}
    for name in use_names:
        k = name2idx[name]
        img = cv2.imread(str(ed / f"frame_{idx:06d}" / f"cam{k}.jpg"))
        out[name] = img
    return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py -q`
Expected: PASS (16 passed)

- [ ] **Step 5: Commit**

```bash
git add calibration/bev_autolabel/bev_label.py calibration/bev_autolabel/bev_io.py calibration/bev_autolabel/test_bev_label.py
git commit -m "feat(bev): build_label 오케스트레이션 + 맵/stamp/이미지 IO 로더

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 6: 렌더링 + 단계1 검증 CLI (품질 튜닝)

**Files:**
- Create: `calibration/bev_autolabel/render.py`
- Create: `calibration/bev_autolabel/verify_labels.py`
- Modify: `docs/superpowers/specs/2026-07-27-bev-autolabel-quality-improvement-design.md` (§6 튜닝값 갱신)
- Test: `calibration/bev_autolabel/test_bev_label.py` (append render 스모크)

**Interfaces:**
- Consumes: `build_label`, `bev_io.*`, `ds_model.load_rig`, `calib_io.load_T_front_lidar`, `BevSpec`.
- Produces:
  - `render.colorize(label) -> ndarray(NX,NY,3) uint8` — 0→(0,0,255)BGR빨강, 1→(0,200,0)초록, 2→(128,128,128)회색.
  - `render.review_image(label, spec, cam_imgs, scale=8) -> ndarray uint8` — BEV 확대 + 미터축·0.5m 격자·ego점·전방화살표, 상단에 3카메라(front/left/right) 가로 배치.

- [ ] **Step 1: Write the failing test (render 스모크)**

`test_bev_label.py`에 추가:
```python
def test_colorize_and_review_shapes():
    import render
    s = BevSpec()
    lab = np.full((s.NX, s.NY), 2, np.uint8); lab[30:40, 30:40] = 1; lab[10, 10] = 0
    rgb = render.colorize(lab)
    assert rgb.shape == (s.NX, s.NY, 3) and rgb.dtype == np.uint8
    dummy = {n: np.zeros((720, 1280, 3), np.uint8) for n in ("front", "left", "right")}
    rev = render.review_image(lab, s, dummy, scale=8)
    assert rev.ndim == 3 and rev.shape[2] == 3 and rev.shape[0] > s.NX
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py::test_colorize_and_review_shapes -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'render'`

- [ ] **Step 3: Write minimal implementation**

`calibration/bev_autolabel/render.py`:
```python
"""BEV 라벨 렌더: 색칠 + 미터축·격자 검수뷰(3카메라 합성)."""
from __future__ import annotations

import numpy as np
import cv2

_COLOR = {0: (0, 0, 255), 1: (0, 200, 0), 2: (128, 128, 128)}  # BGR


def colorize(label):
    out = np.zeros((label.shape[0], label.shape[1], 3), np.uint8)
    for v, col in _COLOR.items():
        out[label == v] = col
    return out


def review_image(label, spec, cam_imgs, scale=8):
    """BEV 확대 + 미터축·0.5m 격자·ego·전방화살표, 상단 3카메라 가로 배치."""
    bev = cv2.resize(colorize(label), (spec.NY * scale, spec.NX * scale),
                     interpolation=cv2.INTER_NEAREST)
    H, W = bev.shape[:2]
    # 0.5m 격자 + 미터 라벨
    step = int(round(0.5 / spec.RES)) * scale
    for gx in range(0, W, step):
        cv2.line(bev, (gx, 0), (gx, H), (60, 60, 60), 1)
    for gy in range(0, H, step):
        cv2.line(bev, (0, gy), (W, gy), (60, 60, 60), 1)
    # 축 라벨(전방 x: 위로 +, 좌우 y)
    for r_m in range(-int(spec.XR), int(spec.XF) + 1):
        py = int((spec.XF - r_m) / spec.RES) * scale
        cv2.putText(bev, f"{r_m}m", (2, max(12, py)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (255, 255, 0), 1, cv2.LINE_AA)
    # ego 위치 + 전방 화살표
    ex, ey = spec.C_EGO * scale, spec.R_EGO * scale
    cv2.circle(bev, (ex, ey), 4, (255, 255, 255), -1)
    cv2.arrowedLine(bev, (ex, ey), (ex, ey - 4 * scale), (255, 255, 255), 2, tipLength=0.3)
    # 상단 3카메라(front/left/right) 가로 배치, BEV 폭에 맞춤
    order = [n for n in ("front", "left", "right") if cam_imgs.get(n) is not None]
    if order:
        cw = W // len(order)
        row = []
        for n in order:
            im = cv2.resize(cam_imgs[n], (cw, int(cw * 0.5625)))  # 16:9
            cv2.putText(im, n, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            row.append(im)
        top = np.hstack(row)
        if top.shape[1] != W:
            top = cv2.resize(top, (W, top.shape[0]))
        return np.vstack([top, bev])
    return bev
```

`calibration/bev_autolabel/verify_labels.py`:
```python
#!/usr/bin/env python3
"""단계1 품질 검증: 지정 프레임들의 build_label 결과를 review PNG로 저장.

사용:
  cd calibration/bev_autolabel
  python3 verify_labels.py \
    --map-dir  ../../data/sj_bags/260722/raws3_mapping \
    --extract-dir ../../data/cam_out/extracted \
    --calib ../../data/calib_260723/calib.yaml \
    --orient ../../data/calib_260723/orientation.json \
    --frames 900 2000 2500 4850 \
    --out ../../data/sj_bags/260722/raws3_mapping
"""
import argparse
import pathlib
import sys

import cv2

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
from calib_io import load_T_front_lidar  # noqa: E402
from ds_model import load_rig            # noqa: E402
import bev_io                            # noqa: E402
import render                            # noqa: E402
from bev_label import BevSpec, build_label, compute_self_voxels  # noqa: E402

USE = ("front", "left", "right")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-dir", required=True)
    ap.add_argument("--extract-dir", required=True)
    ap.add_argument("--calib", required=True)
    ap.add_argument("--orient", required=True)
    ap.add_argument("--frames", type=int, nargs="+", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    spec = BevSpec()
    rig = load_rig(a.calib, a.orient)
    T_front_lidar = load_T_front_lidar(a.calib)
    p, times, poses, tpos = bev_io.load_map(a.map_dir)
    stamps = bev_io.load_stamps(a.extract_dir)
    print(f"map points={len(p)} poses={len(poses)}")
    self_vox = compute_self_voxels(p, poses)
    print(f"self voxels={len(self_vox)}")

    out = pathlib.Path(a.out)
    for idx in a.frames:
        t_ns = stamps[idx]
        lab = build_label(p, times, poses, tpos, self_vox,
                          rig.cams_by_name, rig.T_cam_front, T_front_lidar, t_ns, spec)
        imgs = bev_io.load_cam_images(a.extract_dir, idx, rig.idx_to_name, USE)
        rev = render.review_image(lab, spec, imgs)
        dst = out / f"bev_review_{idx:06d}.png"
        cv2.imwrite(str(dst), rev)
        print(f"saved {dst}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run render 스모크 테스트 통과 확인**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py -q`
Expected: PASS (17 passed)

- [ ] **Step 5: 실데이터 4프레임 렌더 + 육안 품질 검증(수동)**

Run:
```bash
cd calibration/bev_autolabel && python3 verify_labels.py \
  --map-dir ../../data/sj_bags/260722/raws3_mapping \
  --extract-dir ../../data/cam_out/extracted \
  --calib ../../data/calib_260723/calib.yaml \
  --orient ../../data/calib_260723/orientation.json \
  --frames 900 2000 2500 4850 \
  --out ../../data/sj_bags/260722/raws3_mapping
```
Expected: `bev_review_000900.png` 등 4개 저장. 이미지를 열어 **품질 판정 기준**(spec §3) 확인:
① obstacle이 작물행 경계를 잘 물음 ② 카메라 안 보이는 후방/가림이 회색 ③ 통로가 초록 ④ 가까운 기둥이 ego로 안 지워짐.
기준 미달 시 `bev_label.py`의 파라미터(`z_gate, min_extent, min_pts, floor win/pct, self radius, corridor r`)를 조정하고 재실행. **이 튜닝 루프가 이 태스크의 핵심.** 사용자와 함께 확인.

- [ ] **Step 6: 확정된 튜닝값을 spec §6 표에 기록 + 커밋**

`docs/superpowers/specs/2026-07-27-bev-autolabel-quality-improvement-design.md` §6의 값을 실제 확정치로 갱신(초기값과 달라진 것만).
```bash
git add calibration/bev_autolabel/render.py calibration/bev_autolabel/verify_labels.py calibration/bev_autolabel/test_bev_label.py docs/superpowers/specs/2026-07-27-bev-autolabel-quality-improvement-design.md
git commit -m "feat(bev): 렌더(미터축 검수뷰) + 단계1 검증 CLI, 튜닝값 확정

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 7: 단계2 데이터셋 생성 CLI

**Files:**
- Create: `calibration/bev_autolabel/generate.py`
- Test: `calibration/bev_autolabel/test_bev_label.py` (append 키프레임 선택 로직 테스트)

**Interfaces:**
- Consumes: 위 전체.
- Produces:
  - `select_keyframes(stamps, times_ns, poses, kf_step=0.4) -> list[int]` — 이동거리 kf_step(m) 간격 프레임 idx(오름차순). (bev_label.py 또는 generate.py에 위치 — 순수라 테스트 위해 `bev_label.py`에 둔다.)
  - `generate.py` CLI — 각 키프레임을 `sample_NNNNNN/`에 저장(`label.png` 인덱스 팔레트, `review.png`, `cam_{front,left,right}.jpg`, `meta.json`) + `dataset.csv`.

- [ ] **Step 1: Write the failing test**

`test_bev_label.py`에 추가:
```python
from bev_label import select_keyframes


def test_select_keyframes_by_displacement():
    # 0.1m 간격 11프레임 → 0.4m 스텝이면 idx 0,4,8 근처만 선택
    stamps = {i: int(i * 1e8) for i in range(11)}
    times = np.array([int(i * 1e8) for i in range(11)], np.int64)
    poses = [np.eye(4) for _ in range(11)]
    for i in range(11):
        poses[i][0, 3] = i * 0.1
    kf = select_keyframes(stamps, times, poses, kf_step=0.4)
    assert kf[0] == 0
    # 연속 선택 간 이동거리 >= 0.4m (마지막 제외)
    xs = [poses[i][0, 3] for i in kf]
    assert all(xs[j + 1] - xs[j] >= 0.4 - 1e-9 for j in range(len(xs) - 1))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py::test_select_keyframes_by_displacement -q`
Expected: FAIL — `ImportError: cannot import name 'select_keyframes'`

- [ ] **Step 3: Write minimal implementation**

`bev_label.py`에 추가:
```python
def select_keyframes(stamps, times_ns, poses, kf_step=0.4):
    """카메라 프레임 stamp를 pose 이동거리 kf_step(m) 간격으로 서브샘플. idx 오름차순 리스트."""
    order = sorted(stamps)
    kept, last = [], None
    for idx in order:
        T = pose_at(times_ns, poses, stamps[idx])
        pos = T[:3, 3]
        if last is None or np.linalg.norm(pos - last) >= kf_step:
            kept.append(idx)
            last = pos
    return kept
```

`calibration/bev_autolabel/generate.py`:
```python
#!/usr/bin/env python3
"""단계2: bag의 LIO 맵 + 추출 이미지 → BEV auto-label 데이터셋 일괄 생성.

샘플: sample_NNNNNN/{label.png(인덱스 팔레트 0/1/2), review.png, cam_{front,left,right}.jpg, meta.json}
      + dataset.csv

사용:
  cd calibration/bev_autolabel
  python3 generate.py \
    --map-dir ../../data/sj_bags/260722/raws3_mapping \
    --extract-dir ../../data/cam_out/extracted \
    --calib ../../data/calib_260723/calib.yaml \
    --orient ../../data/calib_260723/orientation.json \
    --out ../../data/bev_dataset/raws3 --kf-step 0.4
"""
import argparse
import csv
import json
import pathlib
import sys

import numpy as np
import cv2
from PIL import Image

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
from calib_io import load_T_front_lidar  # noqa: E402
from cloud_io import pose_at             # noqa: E402
from ds_model import load_rig            # noqa: E402
import bev_io                            # noqa: E402
import render                            # noqa: E402
from bev_label import BevSpec, build_label, compute_self_voxels, select_keyframes  # noqa: E402

USE = ("front", "left", "right")
_PALETTE = [255, 0, 0, 0, 200, 0, 128, 128, 128] + [0] * (256 * 3 - 9)  # RGB: 0빨강 1초록 2회색


def save_label_png(path, label):
    im = Image.fromarray(label, mode="P")
    im.putpalette(_PALETTE)
    im.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-dir", required=True)
    ap.add_argument("--extract-dir", required=True)
    ap.add_argument("--calib", required=True)
    ap.add_argument("--orient", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--kf-step", type=float, default=0.4)
    ap.add_argument("--limit", type=int, default=0, help="테스트용 최대 샘플 수(0=전체)")
    a = ap.parse_args()

    spec = BevSpec()
    rig = load_rig(a.calib, a.orient)
    T_front_lidar = load_T_front_lidar(a.calib)
    p, times, poses, tpos = bev_io.load_map(a.map_dir)
    stamps = bev_io.load_stamps(a.extract_dir)
    self_vox = compute_self_voxels(p, poses)
    kf = select_keyframes(stamps, times, poses, kf_step=a.kf_step)
    if a.limit:
        kf = kf[:a.limit]
    print(f"map={len(p)} poses={len(poses)} self_vox={len(self_vox)} keyframes={len(kf)}")

    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for n, idx in enumerate(kf):
        t_ns = stamps[idx]
        lab = build_label(p, times, poses, tpos, self_vox,
                          rig.cams_by_name, rig.T_cam_front, T_front_lidar, t_ns, spec)
        imgs = bev_io.load_cam_images(a.extract_dir, idx, rig.idx_to_name, USE)
        sd = out / f"sample_{n:06d}"
        sd.mkdir(exist_ok=True)
        save_label_png(sd / "label.png", lab)
        cv2.imwrite(str(sd / "review.png"), render.review_image(lab, spec, imgs))
        for name in USE:
            cv2.imwrite(str(sd / f"cam_{name}.jpg"), imgs[name])
        T_wb = pose_at(times, poses, t_ns)
        meta = {
            "frame_idx": idx, "stamp_ns": t_ns,
            "world_T_body": T_wb.tolist(),
            "bev": {"XF": spec.XF, "XR": spec.XR, "YH": spec.YH, "RES": spec.RES,
                    "NX": spec.NX, "NY": spec.NY, "R_EGO": spec.R_EGO, "C_EGO": spec.C_EGO},
            "classes": {"0": "obstacle", "1": "drivable", "2": "ignore"},
            "cameras": list(USE),
            "calib": str(pathlib.Path(a.calib).resolve()),
            "orient": str(pathlib.Path(a.orient).resolve()),
        }
        (sd / "meta.json").write_text(json.dumps(meta, indent=2))
        rows.append({"sample": f"sample_{n:06d}", "frame_idx": idx, "stamp_ns": t_ns})
        if n % 20 == 0:
            print(f"{n}/{len(kf)} {sd.name}")

    with open(out / "dataset.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["sample", "frame_idx", "stamp_ns"])
        w.writeheader()
        w.writerows(rows)
    print(f"done: {len(rows)} samples → {out}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd calibration/bev_autolabel && python3 -m pytest test_bev_label.py -q`
Expected: PASS (18 passed)

- [ ] **Step 5: 스모크 실행(소량 --limit)로 산출물 확인**

Run:
```bash
cd calibration/bev_autolabel && python3 generate.py \
  --map-dir ../../data/sj_bags/260722/raws3_mapping \
  --extract-dir ../../data/cam_out/extracted \
  --calib ../../data/calib_260723/calib.yaml \
  --orient ../../data/calib_260723/orientation.json \
  --out ../../data/bev_dataset/raws3_smoke --kf-step 0.4 --limit 3
```
Expected: `sample_000000..2/` 각각에 `label.png, review.png, cam_front.jpg, cam_left.jpg, cam_right.jpg, meta.json` + `dataset.csv`. `label.png`가 열었을 때 빨강/초록/회색으로 구분되고, `python3 -c "import numpy as np; from PIL import Image; print(np.unique(np.array(Image.open('../../data/bev_dataset/raws3_smoke/sample_000000/label.png'))))"` 가 `[0 1 2]` 부분집합.

- [ ] **Step 6: Commit**

```bash
git add calibration/bev_autolabel/generate.py calibration/bev_autolabel/bev_label.py calibration/bev_autolabel/test_bev_label.py
git commit -m "feat(bev): 단계2 데이터셋 생성 CLI(키프레임 샘플링·label/review/meta 저장)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## 최종 확인

- [ ] 전체 순수 로직 테스트: `cd calibration/bev_autolabel && python3 -m pytest -q` → 18 passed.
- [ ] `data/`는 gitignore이므로 산출 PNG·데이터셋은 커밋되지 않음(확인).
- [ ] `docs/BEV_AUTOLABEL.md`에 "구현 완료(§10 CLI = `calibration/bev_autolabel/`)" 한 줄 추가 여부는 사용자와 상의(선택).

## 자체 리뷰 메모(작성자 확인 완료)

- **Spec 커버리지**: §2-A 수직성/국소floor=Task2, §2-B FoV/가림=Task3, §2-C ego0.28=Task4, §2-D corridor=Task4/build, §2-E 조립·연결정리=Task4, §3 단계1=Task6·단계2=Task7, §4 pytest=전 태스크. 누락 없음.
- **타입 일관성**: `build_label` 인자(cams=name→cam, T_cam_front=name→4x4)가 `fov_mask` 시그니처와 일치. `rig.cams_by_name/T_cam_front/idx_to_name`은 `ds_model.CameraRig` 필드와 일치(확인).
- **플레이스홀더**: 모든 코드 스텝에 실제 코드 포함. Task6 Step5의 파라미터 튜닝은 값 조정 루프이며 코드 구조는 완비.
