# Cam–LiDAR Extrinsic Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 어안 4대(DS)와 Unitree L2 라이다 사이 6DoF extrinsic `T_front_lidar`를 수동 2D–3D 대응 + DS-PnP로 구해 `calib.yaml`에 기록하는 호스트-파이썬 도구 모음을 만든다.

**Architecture:** 순수 로직(SE3·투영 체인·PnP 잔차 = `chain.py`, 클라우드 파싱·누적·모션보정 = `cloud_io.py`)을 TDD로 먼저 만들고, 그 위에 얇은 CLI 4개(`accumulate_cloud`·`pick_correspondences`·`solve_extrinsic`·`overlay_verify`)를 얹는다. DS 투영은 `calibration/verify/ds_model.py`(`DoubleSphereCamera`, `CameraRig`, `load_rig`)를 재사용한다.

**Tech Stack:** Python3(호스트), numpy, scipy(`optimize.least_squares`, `spatial.transform.Rotation`), opencv-python(cv2, 2D 클릭·이미지), open3d(클라우드 IO·3D 점 클릭), rosbag2_py/rclpy(bag 읽기, 지연 import).

## Global Constraints

- **호스트 파이썬 전용.** Docker/ROS 노드 불필요. bag 읽기(`rosbag2_py`, `rclpy.serialization`)는 함수 내부 **지연 import**(순수 로직 테스트가 ROS 없이 돌게). 패턴: `mapping/check_lidar_bag.py`.
- **DS intrinsics 순서:** `[xi, alpha, fx, fy, cx, cy]` (calib.yaml/Kalibr 동일).
- **extrinsic 규약:** `T_<target>_<source>` = source 프레임 점 → target 프레임 점. 신규 키 `T_front_lidar` = lidar 점 → front 프레임. `ds_model.CameraRig.T_cam_front[name]` = front → name.
- **투영 체인:** `pixel = cam.project(T_cam_front · T_front_lidar · P_lidar)`.
- **테스트 실행:** `cd calibration/cam_lidar && python3 -m pytest -q`. 하드웨어·이미지·bag 불필요한 순수 로직만 테스트. GUI/CLI는 수동 스모크.
- **파일 위치:** 신규 코드 `calibration/cam_lidar/`, 문서 `docs/CAM_LIDAR_CALIBRATION.md`.
- **커밋:** `feat(calib): ...` / `test(calib): ...` / `docs(calib): ...`. 각 커밋 끝에
  `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. 브랜치 `feat/cam-lidar-calib`는 생성됨. **병합·푸시는 사용자가 직접**(Claude는 커밋까지만).
- **좌표/단위:** 거리 m, 각도는 CLI 인자에서 도(deg). 라이다 프레임 `unilidar_lidar`, cloud 토픽 `/unilidar/cloud`.

---

### Task 0: open3d arm64 설치 스파이크 (리스크 선차단)

pick_correspondences가 open3d의 3D 점 클릭에 의존한다. Jetson aarch64에서 설치·픽킹이 되는지 **먼저** 확인한다.

**Files:**
- Create: `calibration/cam_lidar/README.md` (스파이크 결과 1~2줄 기록)

- [ ] **Step 1: open3d 설치 시도**

Run:
```bash
python3 -c "import open3d" 2>/dev/null && echo "already" || pip install open3d
python3 -c "import open3d as o3d; print('open3d', o3d.__version__)"
```
Expected: 버전 출력. 실패(aarch64 휠 없음) 시 → 대체 경로 기록하고 아래 Step 2로.

- [ ] **Step 2: 픽킹 가용성 확인 및 결과 기록**

Run(디스플레이/ X-forwarding 있는 환경에서):
```bash
python3 - <<'PY'
import numpy as np, open3d as o3d
p = o3d.geometry.PointCloud()
p.points = o3d.utility.Vector3dVector(np.random.rand(100,3))
vis = o3d.visualization.VisualizerWithEditing()
print("VisualizerWithEditing OK (pick_points 사용 가능)")
PY
```
Expected: `VisualizerWithEditing OK`. 헤드리스면 import만 확인하고 픽킹은 실기에서.

`calibration/cam_lidar/README.md`에 결과 한 줄:
```markdown
# cam_lidar 스파이크 결과
- open3d: <버전> 설치됨 / 대체(<사유·대안>)
- 3D 픽킹: VisualizerWithEditing 가용 / 실기확인필요
```

- [ ] **Step 3: Commit**

```bash
git add calibration/cam_lidar/README.md
git commit -m "chore(calib): open3d arm64 스파이크 결과 기록

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

> **대체 경로(open3d 불가 시):** 클라우드 IO는 `.npy`(numpy)로 대체, 3D 픽킹은 matplotlib `Axes3D` 산점도 + `mpl_connect('pick_event')` 기반 경량 피커로 대체. 이 경우 Task 8에서 피커 백엔드만 교체(다른 Task 영향 없음 — 대응점 형식은 동일).

---

### Task 1: `chain.py` — SE3/회전 헬퍼

6DoF를 다루는 순수 로직. 이후 모든 투영·최적화의 토대.

**Files:**
- Create: `calibration/cam_lidar/chain.py`
- Create: `calibration/cam_lidar/test_cam_lidar.py`

**Interfaces:**
- Consumes: (없음 — scipy만)
- Produces:
  - `se3(rvec, tvec) -> np.ndarray(4,4)` — `rvec`=회전벡터(축각, rad, shape(3,)), `tvec` shape(3,)
  - `se3_inv(T) -> np.ndarray(4,4)`
  - `se3_to_rvec_tvec(T) -> (np.ndarray(3,), np.ndarray(3,))`
  - `se3_from_rpy_xyz(roll, pitch, yaw, x, y, z, degrees=True) -> np.ndarray(4,4)` — euler 'xyz'
  - `transform(T, P) -> np.ndarray(...,3)` — `P` shape(...,3)

- [ ] **Step 1: Write the failing tests**

`calibration/cam_lidar/test_cam_lidar.py`:
```python
"""순수 로직 테스트(ROS·이미지·bag 불필요).

실행: cd calibration/cam_lidar && python3 -m pytest -q
"""
import numpy as np

from chain import se3, se3_inv, se3_to_rvec_tvec, se3_from_rpy_xyz, transform


def test_se3_identity_roundtrip():
    T = se3(np.zeros(3), np.zeros(3))
    assert np.allclose(T, np.eye(4))


def test_se3_inv_is_inverse():
    T = se3_from_rpy_xyz(10, -20, 30, 0.1, -0.2, 0.3)
    assert np.allclose(T @ se3_inv(T), np.eye(4), atol=1e-12)


def test_se3_to_rvec_tvec_roundtrip():
    rvec = np.array([0.2, -0.5, 0.1])
    tvec = np.array([1.0, 2.0, -3.0])
    T = se3(rvec, tvec)
    r2, t2 = se3_to_rvec_tvec(T)
    assert np.allclose(se3(r2, t2), T, atol=1e-12)


def test_transform_translation():
    T = se3(np.zeros(3), np.array([1.0, 2.0, 3.0]))
    P = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]])
    out = transform(T, P)
    assert np.allclose(out, [[1, 2, 3], [2, 3, 4]])


def test_se3_from_rpy_90deg_z():
    T = se3_from_rpy_xyz(0, 0, 90, 0, 0, 0)
    out = transform(T, np.array([[1.0, 0.0, 0.0]]))
    assert np.allclose(out[0], [0, 1, 0], atol=1e-9)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd calibration/cam_lidar && python3 -m pytest test_cam_lidar.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'chain'`

- [ ] **Step 3: Implement `chain.py` helpers**

`calibration/cam_lidar/chain.py`:
```python
"""라이다→카메라 투영 체인과 DS-PnP 최적화(순수 로직).

extrinsic 규약: T_<target>_<source> = source 점 → target 점.
투영: pixel = cam.project(T_cam_front · T_front_lidar · P_lidar).
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def se3(rvec, tvec):
    """rvec(축각,rad,(3,)) + tvec((3,)) → 4x4."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(np.asarray(rvec, float)).as_matrix()
    T[:3, 3] = np.asarray(tvec, float)
    return T


def se3_inv(T):
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def se3_to_rvec_tvec(T):
    rvec = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    return rvec, T[:3, 3].copy()


def se3_from_rpy_xyz(roll, pitch, yaw, x, y, z, degrees=True):
    R = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def transform(T, P):
    """T(4x4) 로 P(...,3) 변환 → (...,3)."""
    P = np.asarray(P, float)
    return P @ T[:3, :3].T + T[:3, 3]
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd calibration/cam_lidar && python3 -m pytest test_cam_lidar.py -q`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
git add calibration/cam_lidar/chain.py calibration/cam_lidar/test_cam_lidar.py
git commit -m "feat(calib): chain.py SE3/회전 헬퍼 + 테스트

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: `chain.py` — 투영 체인 · PnP 잔차 · solve

DS 재사용해 라이다 점을 카메라 픽셀로 투영하고, 대응점 재투영 오차를 최소화하는 6DoF 해를 푼다.

**Files:**
- Modify: `calibration/cam_lidar/chain.py` (함수 추가)
- Modify: `calibration/cam_lidar/test_cam_lidar.py` (테스트 추가)

**Interfaces:**
- Consumes: Task 1(`se3`,`se3_to_rvec_tvec`,`transform`); `ds_model.DoubleSphereCamera`(`.project(P)->(u,v,valid)`), `ds_model.CameraRig`(`.cams_by_name`,`.T_cam_front`)
- Produces:
  - `project(P_lidar, T_front_lidar, T_cam_front, cam) -> (u, v, valid)` — `P_lidar` shape(...,3)
  - `residuals(params, corrs, rig) -> np.ndarray(2N,)` — `params`=(6,)=rvec+tvec; `corrs`=list[dict{"cam":str,"uv":[u,v],"xyz":[x,y,z]}]; `rig`=CameraRig
  - `solve(corrs, rig, init_T, huber=2.0) -> (T_front_lidar(4,4), rms_px(float), per_point_px(np.ndarray(N,)))`

- [ ] **Step 1: Write the failing tests**

`test_cam_lidar.py` 에 추가(상단 import 확장):
```python
from chain import project, residuals, solve

# ds_model 재사용(형제 디렉터리 verify/)
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "verify"))
from ds_model import DoubleSphereCamera, CameraRig


def _toy_rig():
    """front·right 2대 합성 리그(파일 불필요)."""
    front = DoubleSphereCamera(xi=-0.19, alpha=0.606, fx=295.0, fy=295.0,
                               cx=640.0, cy=360.0, width=1280, height=720, name="front")
    right = DoubleSphereCamera(xi=-0.20, alpha=0.604, fx=293.0, fy=293.0,
                               cx=635.0, cy=350.0, width=1280, height=720, name="right")
    # right 는 front 대비 -90도 yaw (앞→오른쪽). front→cam 변환.
    T_front_front = np.eye(4)
    T_right_front = se3_from_rpy_xyz(0, -90, 0, 0.0, 0.0, 0.0)
    return CameraRig({"front": front, "right": right},
                     {"front": T_front_front, "right": T_right_front},
                     {0: "front", 1: "right"})


def _make_corrs(rig, T_true, n_per_cam=8):
    """알려진 T_front_lidar로 라이다 점→픽셀 합성 대응점 생성."""
    rng = np.random.default_rng(0)
    corrs = []
    for name in ("front", "right"):
        cam = rig.cams_by_name[name]
        # 카메라 정면(+z, cam 프레임)에 점 배치 → front 프레임 → lidar 프레임으로 역변환
        for _ in range(n_per_cam):
            z = rng.uniform(1.0, 8.0)
            x = rng.uniform(-0.6, 0.6) * z
            y = rng.uniform(-0.4, 0.4) * z
            P_cam = np.array([x, y, z])
            u, v, ok = cam.project(P_cam)
            if not ok:
                continue
            P_front = transform(se3_inv(rig.T_cam_front[name]), P_cam)
            P_lidar = transform(se3_inv(T_true), P_front)
            corrs.append({"cam": name, "uv": [float(u), float(v)],
                          "xyz": [float(P_lidar[0]), float(P_lidar[1]), float(P_lidar[2])]})
    return corrs


def test_project_matches_ds_directly():
    rig = _toy_rig()
    cam = rig.cams_by_name["front"]
    P_cam = np.array([0.1, -0.2, 3.0])
    u0, v0, ok0 = cam.project(P_cam)
    # T_front_lidar = 항등, T_cam_front(front)=항등 → project 는 ds 와 동일해야
    u, v, ok = project(P_cam, np.eye(4), np.eye(4), cam)
    assert ok == ok0 and np.allclose([u, v], [u0, v0])


def test_solve_recovers_known_extrinsic():
    rig = _toy_rig()
    # 라이다가 카메라 대비 z-up(≈ +90도 pitch) + 오프셋: 큰 회전
    T_true = se3_from_rpy_xyz(-90, 0, 0, 0.05, -0.03, 0.10)
    corrs = _make_corrs(rig, T_true)
    assert len(corrs) >= 12
    # 초기값: 진값에 15도/5cm 섭동(대략 장착값이 알려진 실사용 상황 모사)
    perturb = se3_from_rpy_xyz(15, -10, 8, 0.03, 0.03, -0.03)
    init_T = perturb @ T_true
    T_est, rms, per = solve(corrs, rig, init_T)
    assert rms < 1e-3
    assert np.allclose(T_est, T_true, atol=1e-4)


def test_residuals_length():
    rig = _toy_rig()
    T_true = se3_from_rpy_xyz(-90, 0, 0, 0, 0, 0)
    corrs = _make_corrs(rig, T_true)
    r = residuals(np.zeros(6), corrs, rig)
    assert r.shape == (2 * len(corrs),)
```
(파일 상단의 `from chain import ...` 줄에 `se3_inv, se3_from_rpy_xyz` 가 이미 포함돼 있어야 함 — Task 1 테스트에서 import 했으니 병합.)

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd calibration/cam_lidar && python3 -m pytest test_cam_lidar.py -q`
Expected: FAIL — `ImportError: cannot import name 'project'`

- [ ] **Step 3: Implement projection/residuals/solve in `chain.py`**

`chain.py` 하단에 추가:
```python
from scipy.optimize import least_squares

_INVALID_PENALTY = 1e3


def project(P_lidar, T_front_lidar, T_cam_front, cam):
    """라이다 점 → 카메라 픽셀. cam=DoubleSphereCamera. 반환 (u, v, valid)."""
    P_front = transform(T_front_lidar, P_lidar)
    P_cam = transform(T_cam_front, P_front)
    return cam.project(P_cam)


def residuals(params, corrs, rig):
    """params=(6,) rvec+tvec → 대응점별 (u,v) 재투영 잔차 평탄화(2N,)."""
    T = se3(params[:3], params[3:])
    out = np.empty(2 * len(corrs), float)
    for i, c in enumerate(corrs):
        cam = rig.cams_by_name[c["cam"]]
        u, v, ok = project(np.asarray(c["xyz"], float), T, rig.T_cam_front[c["cam"]], cam)
        if ok:
            out[2 * i] = u - c["uv"][0]
            out[2 * i + 1] = v - c["uv"][1]
        else:
            out[2 * i] = out[2 * i + 1] = _INVALID_PENALTY
    return out


def solve(corrs, rig, init_T, huber=2.0):
    """대응점→T_front_lidar. 반환 (T(4,4), rms_px, per_point_px(N,))."""
    r0, t0 = se3_to_rvec_tvec(init_T)
    x0 = np.concatenate([r0, t0])
    res = least_squares(residuals, x0, args=(corrs, rig), loss="huber", f_scale=huber)
    T = se3(res.x[:3], res.x[3:])
    r = res.fun.reshape(-1, 2)
    per = np.hypot(r[:, 0], r[:, 1])
    rms = float(np.sqrt(np.mean(per ** 2)))
    return T, rms, per
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd calibration/cam_lidar && python3 -m pytest test_cam_lidar.py -q`
Expected: PASS (전체 통과, 8+ passed)

- [ ] **Step 5: Commit**

```bash
git add calibration/cam_lidar/chain.py calibration/cam_lidar/test_cam_lidar.py
git commit -m "feat(calib): DS 투영 체인·PnP 잔차·solve + 합성 복원 테스트

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: `ds_model.load_rig` 하드닝 + `T_front_lidar` 로더

`load_rig`가 `T_front_lidar` 같은 비-카메라 extrinsic 키를 만나면 잘못 슬라이스해 `front`를 덮어쓴다. 카메라 키만 취하도록 고치고, 라이다 extrinsic 전용 로더를 더한다.

**Files:**
- Modify: `calibration/verify/ds_model.py:120-124` (extrinsic 파싱 루프)
- Create: `calibration/cam_lidar/calib_io.py`
- Modify: `calibration/cam_lidar/test_cam_lidar.py`

**Interfaces:**
- Consumes: `ds_model.load_rig`
- Produces:
  - `calib_io.load_T_front_lidar(calib_path) -> np.ndarray(4,4) | None`
  - `calib_io.write_extrinsic(calib_path, T_front_lidar, meta) -> None` — `meta`=dict, `verification.cam_lidar`에 병합

- [ ] **Step 1: Write the failing tests**

`test_cam_lidar.py` 에 추가:
```python
import yaml
from calib_io import load_T_front_lidar, write_extrinsic

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "verify"))
from ds_model import load_rig

_MIN_CALIB = {
    "model_chosen": "ds",
    "cameras": {
        "front": {"camera_model": "ds", "intrinsics": [-0.19, 0.606, 295.0, 295.0, 640.0, 360.0],
                  "distortion_model": "none", "distortion_coeffs": [], "resolution": [1280, 720]},
        "right": {"camera_model": "ds", "intrinsics": [-0.20, 0.604, 293.0, 293.0, 635.0, 350.0],
                  "distortion_model": "none", "distortion_coeffs": [], "resolution": [1280, 720]},
    },
    "extrinsics": {
        "T_front_front": np.eye(4).tolist(),
        "T_right_front": se3_from_rpy_xyz(0, -90, 0, 0, 0, 0).tolist(),
    },
    "verification": {"reproj_rms_px": {}},
}
_ORIENT = {"cam0": "front", "cam1": "right"}


def _write(tmp_path, calib=_MIN_CALIB, orient=_ORIENT):
    cp = tmp_path / "calib.yaml"
    op = tmp_path / "orientation.json"
    cp.write_text(yaml.safe_dump(calib))
    op.write_text(__import__("json").dumps(orient))
    return cp, op


def test_load_rig_ignores_lidar_extrinsic(tmp_path):
    calib = {**_MIN_CALIB, "extrinsics": {**_MIN_CALIB["extrinsics"],
             "T_front_lidar": se3_from_rpy_xyz(-90, 0, 0, 0.05, 0, 0.1).tolist()}}
    cp, op = _write(tmp_path, calib)
    rig = load_rig(str(cp), str(op))
    # front extrinsic 은 여전히 항등(라이다 키에 덮이지 않음)
    assert np.allclose(rig.T_cam_front["front"], np.eye(4))
    assert set(rig.cams_by_name) == {"front", "right"}
    assert "lidar" not in rig.T_cam_front and "front_lidar" not in rig.T_cam_front


def test_write_then_load_T_front_lidar(tmp_path):
    cp, _ = _write(tmp_path)
    assert load_T_front_lidar(str(cp)) is None
    T = se3_from_rpy_xyz(-90, 0, 0, 0.05, -0.03, 0.1)
    write_extrinsic(str(cp), T, {"method": "manual-pnp-ds", "reproj_rms_px": 0.8, "stage": 1})
    back = load_T_front_lidar(str(cp))
    assert back is not None and np.allclose(back, T, atol=1e-12)
    doc = yaml.safe_load(cp.read_text())
    assert doc["verification"]["cam_lidar"]["method"] == "manual-pnp-ds"
    # 기존 카메라 검증 블록 보존
    assert "reproj_rms_px" in doc["verification"]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd calibration/cam_lidar && python3 -m pytest test_cam_lidar.py -k "lidar_extrinsic or T_front_lidar" -q`
Expected: FAIL — `test_load_rig_ignores_lidar_extrinsic`(front가 라이다행렬로 덮여 assert 실패) + `ImportError: calib_io`

- [ ] **Step 3a: 하드닝 — `ds_model.py` extrinsic 루프**

`calibration/verify/ds_model.py` 의 `load_rig` 내부, 기존:
```python
    extr = {}
    for key, mat in calib["extrinsics"].items():
        # key = "T_<name>_front"
        name = key[len("T_"):-len("_front")]
        extr[name] = np.array(mat, dtype=np.float64)
```
를 다음으로 교체:
```python
    extr = {}
    for key, mat in calib["extrinsics"].items():
        # 카메라 extrinsic 키만(T_<cam>_front). T_front_lidar 등 비-카메라 키는 무시.
        if not (key.startswith("T_") and key.endswith("_front")):
            continue
        name = key[len("T_"):-len("_front")]
        if name not in cams:
            continue
        extr[name] = np.array(mat, dtype=np.float64)
```

- [ ] **Step 3b: Implement `calib_io.py`**

`calibration/cam_lidar/calib_io.py`:
```python
"""calib.yaml 의 라이다 extrinsic 읽기/쓰기(카메라 블록은 건드리지 않음)."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

_KEY = "T_front_lidar"


def load_T_front_lidar(calib_path):
    doc = yaml.safe_load(Path(calib_path).read_text())
    mat = (doc.get("extrinsics") or {}).get(_KEY)
    return None if mat is None else np.array(mat, dtype=np.float64)


def write_extrinsic(calib_path, T_front_lidar, meta):
    """T_front_lidar(4x4) 를 extrinsics 에, meta 를 verification.cam_lidar 에 병합 저장."""
    p = Path(calib_path)
    doc = yaml.safe_load(p.read_text())
    doc.setdefault("extrinsics", {})[_KEY] = np.asarray(T_front_lidar, float).tolist()
    ver = doc.setdefault("verification", {})
    ver["cam_lidar"] = {**ver.get("cam_lidar", {}), **meta}
    p.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd calibration/cam_lidar && python3 -m pytest test_cam_lidar.py -q`
그리고 기존 verify 테스트 회귀 확인: `cd calibration/verify && python3 -m pytest -q`
Expected: 둘 다 PASS

- [ ] **Step 5: Commit**

```bash
git add calibration/verify/ds_model.py calibration/cam_lidar/calib_io.py calibration/cam_lidar/test_cam_lidar.py
git commit -m "feat(calib): load_rig 비-카메라 extrinsic 무시 + T_front_lidar 읽기/쓰기

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: `cloud_io.py` — PointCloud2 파싱 · voxel · 정지 누적

라이다 클라우드를 numpy로 읽고 다운샘플·정지 누적하는 순수 로직.

**Files:**
- Create: `calibration/cam_lidar/cloud_io.py`
- Modify: `calibration/cam_lidar/test_cam_lidar.py`

**Interfaces:**
- Consumes: (없음)
- Produces:
  - `cloud_to_xyzi(msg) -> np.ndarray(N,4)` — msg=PointCloud2 유사(`.fields[*].name/.offset`, `.point_step`, `.width`, `.height`, `.data`)
  - `voxel_downsample(xyzi, voxel) -> np.ndarray(M,4)`
  - `accumulate_static(clouds, ref_ns=None, window_s=2.0) -> np.ndarray(K,4)` — `clouds`=list[(stamp_ns:int, xyzi(N,4))]
  - `read_clouds(bag_path, topic="/unilidar/cloud") -> list[(stamp_ns, xyzi)]` (지연 import)

- [ ] **Step 1: Write the failing tests**

`test_cam_lidar.py` 에 추가:
```python
from cloud_io import cloud_to_xyzi, voxel_downsample, accumulate_static


class _Field:
    def __init__(self, name, offset):
        self.name, self.offset = name, offset


class _FakeMsg:
    """PointCloud2 유사(x,y,z,intensity float32, point_step=16)."""
    def __init__(self, xyzi):
        self.fields = [_Field("x", 0), _Field("y", 4), _Field("z", 8), _Field("intensity", 12)]
        self.point_step = 16
        self.width, self.height = len(xyzi), 1
        self.data = np.asarray(xyzi, np.float32).tobytes()


def test_cloud_to_xyzi_parses_fields():
    pts = np.array([[1, 2, 3, 0.5], [4, 5, 6, 0.9]], np.float32)
    out = cloud_to_xyzi(_FakeMsg(pts))
    assert out.shape == (2, 4)
    assert np.allclose(out, pts, atol=1e-6)


def test_cloud_to_xyzi_drops_nonfinite():
    pts = np.array([[1, 2, 3, 0.5], [np.nan, 0, 0, 0.1]], np.float32)
    out = cloud_to_xyzi(_FakeMsg(pts))
    assert out.shape == (1, 4)


def test_voxel_downsample_reduces():
    xyzi = np.array([[0.01, 0, 0, 1], [0.02, 0, 0, 1], [5, 5, 5, 1]], float)
    out = voxel_downsample(xyzi, voxel=0.1)
    assert out.shape[0] == 2  # 앞 두 점은 같은 셀


def test_accumulate_static_windows_by_time():
    c = [(0, np.array([[0, 0, 0, 1]], float)),
         (1_000_000_000, np.array([[1, 1, 1, 1]], float)),   # +1.0s
         (5_000_000_000, np.array([[9, 9, 9, 1]], float))]   # +5.0s (창 밖)
    out = accumulate_static(c, ref_ns=0, window_s=2.0)
    assert out.shape[0] == 2
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd calibration/cam_lidar && python3 -m pytest test_cam_lidar.py -k cloud_ -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'cloud_io'`

- [ ] **Step 3: Implement `cloud_io.py`**

`calibration/cam_lidar/cloud_io.py`:
```python
"""라이다 클라우드 IO·누적(순수 로직 + bag 읽기 지연 import).

PointCloud2 파싱은 mapping/check_lidar_bag.py 패턴을 따른다.
"""
from __future__ import annotations

import numpy as np

CLOUD_TOPIC = "/unilidar/cloud"


def cloud_to_xyzi(msg):
    """PointCloud2 유사 → (N,4) x,y,z,intensity(float64). 비유한 xyz 제거."""
    off = {f.name: f.offset for f in msg.fields}
    for k in ("x", "y", "z"):
        if k not in off:
            return np.empty((0, 4))
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 4))
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)

    def col(name):
        o = off[name]
        return buf[:, o:o + 4].copy().view(np.float32).reshape(n).astype(np.float64)

    inten = col("intensity") if "intensity" in off else np.zeros(n)
    xyzi = np.stack([col("x"), col("y"), col("z"), inten], -1)
    return xyzi[np.isfinite(xyzi[:, :3]).all(1)]


def voxel_downsample(xyzi, voxel):
    """voxel(m) 격자 양자화 후 셀당 첫 점만."""
    if len(xyzi) == 0 or voxel <= 0:
        return xyzi
    keys = np.floor(xyzi[:, :3] / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyzi[np.sort(idx)]


def accumulate_static(clouds, ref_ns=None, window_s=2.0):
    """정지 가정: 기준 시각 ±window 안의 프레임 단순 병합. clouds=[(stamp_ns, xyzi)]."""
    if not clouds:
        return np.empty((0, 4))
    if ref_ns is None:
        ref_ns = clouds[0][0]
    w = int(window_s * 1e9)
    sel = [xyzi for (s, xyzi) in clouds if abs(s - ref_ns) <= w]
    return np.vstack(sel) if sel else np.empty((0, 4))


def read_clouds(bag_path, topic=CLOUD_TOPIC):
    """(stamp_ns, xyzi) 리스트를 시간순 반환(bag 읽기). 지연 import."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from sensor_msgs.msg import PointCloud2

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id="mcap"),
                ConverterOptions("", ""))
    out = []
    while reader.has_next():
        t, data, stamp = reader.read_next()
        if t != topic:
            continue
        out.append((stamp, cloud_to_xyzi(deserialize_message(data, PointCloud2))))
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd calibration/cam_lidar && python3 -m pytest test_cam_lidar.py -q`
Expected: PASS (전체 통과)

- [ ] **Step 5: Commit**

```bash
git add calibration/cam_lidar/cloud_io.py calibration/cam_lidar/test_cam_lidar.py
git commit -m "feat(calib): cloud_io PointCloud2 파싱·voxel·정지누적 + 테스트

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: `cloud_io.py` — TUM 궤적 로드 · 자세 보간 · 모션 보정 누적

2단계(이동 현장 bag)용. LIO 궤적으로 각 프레임을 기준 시각 라이다 프레임에 정렬해 누적.

**Files:**
- Modify: `calibration/cam_lidar/cloud_io.py` (함수 추가)
- Modify: `calibration/cam_lidar/test_cam_lidar.py`

**Interfaces:**
- Consumes: Task 1(`chain.se3`,`chain.se3_inv`,`chain.transform`)
- Produces:
  - `load_tum(path) -> (times_ns np.ndarray(T,) int64, poses list[np.ndarray(4,4)])` — TUM: `t tx ty tz qx qy qz qw`, t초
  - `pose_at(times_ns, poses, t_ns) -> np.ndarray(4,4)` — 브래킷 선형보간(위치)+slerp(회전), 양끝 클램프
  - `accumulate_motion(clouds, times_ns, poses, at_ns, window_s=2.0) -> np.ndarray(K,4)`

- [ ] **Step 1: Write the failing tests**

`test_cam_lidar.py` 에 추가:
```python
from cloud_io import load_tum, pose_at, accumulate_motion


def test_load_tum_roundtrip(tmp_path):
    f = tmp_path / "t.tum"
    f.write_text("0.0 0 0 0 0 0 0 1\n1.0 1 0 0 0 0 0 1\n")
    times, poses = load_tum(str(f))
    assert times.tolist() == [0, 1_000_000_000]
    assert np.allclose(poses[1][:3, 3], [1, 0, 0])


def test_pose_at_interpolates_translation():
    times = np.array([0, 2_000_000_000], np.int64)
    poses = [np.eye(4), se3_from_rpy_xyz(0, 0, 0, 2, 0, 0)]
    T = pose_at(times, poses, 1_000_000_000)   # 중간
    assert np.allclose(T[:3, 3], [1, 0, 0], atol=1e-9)


def test_pose_at_clamps_ends():
    times = np.array([0, 1_000_000_000], np.int64)
    poses = [np.eye(4), se3_from_rpy_xyz(0, 0, 0, 1, 0, 0)]
    assert np.allclose(pose_at(times, poses, -5)[:3, 3], [0, 0, 0])
    assert np.allclose(pose_at(times, poses, 9_000_000_000)[:3, 3], [1, 0, 0])


def test_accumulate_motion_compensates():
    # 라이다가 +x로 1m 이동. 세상 같은 점(원점)을 두 시각에 관측 → 각 프레임의 로컬 좌표는 다름.
    times = np.array([0, 1_000_000_000], np.int64)
    poses = [np.eye(4), se3_from_rpy_xyz(0, 0, 0, 1, 0, 0)]
    # t0: 세상원점이 로컬(0,0,0); t1: 라이다가 +x1 이동 → 로컬(-1,0,0)
    clouds = [(0, np.array([[0.0, 0, 0, 1]])), (1_000_000_000, np.array([[-1.0, 0, 0, 1]]))]
    out = accumulate_motion(clouds, times, poses, at_ns=0, window_s=2.0)
    # 기준=t0 라이다 프레임 → 둘 다 (0,0,0) 근처로 정렬돼야
    assert out.shape[0] == 2
    assert np.allclose(out[:, :3], 0.0, atol=1e-9)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd calibration/cam_lidar && python3 -m pytest test_cam_lidar.py -k "tum or pose_at or motion" -q`
Expected: FAIL — `ImportError: cannot import name 'load_tum'`

- [ ] **Step 3: Implement in `cloud_io.py`**

`cloud_io.py` 상단 import 에 추가: `from scipy.spatial.transform import Rotation, Slerp` 및 `from chain import se3, se3_inv, transform`. 하단에 함수 추가:
```python
def load_tum(path):
    """TUM(t tx ty tz qx qy qz qw, t=초) → (times_ns int64(T,), poses list[4x4])."""
    times, poses = [], []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        t, x, y, z, qx, qy, qz, qw = (float(v) for v in line.split()[:8])
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        T[:3, 3] = [x, y, z]
        times.append(int(round(t * 1e9)))
        poses.append(T)
    return np.array(times, np.int64), poses


def pose_at(times_ns, poses, t_ns):
    """브래킷 선형보간(위치)+slerp(회전). 범위 밖은 끝값 클램프."""
    if t_ns <= times_ns[0]:
        return poses[0].copy()
    if t_ns >= times_ns[-1]:
        return poses[-1].copy()
    j = int(np.searchsorted(times_ns, t_ns))
    t0, t1 = times_ns[j - 1], times_ns[j]
    a = (t_ns - t0) / (t1 - t0)
    rots = Rotation.from_matrix([poses[j - 1][:3, :3], poses[j][:3, :3]])
    R = Slerp([0.0, 1.0], rots)([a]).as_matrix()[0]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (1 - a) * poses[j - 1][:3, 3] + a * poses[j][:3, 3]
    return T


def accumulate_motion(clouds, times_ns, poses, at_ns, window_s=2.0):
    """각 프레임을 세상→기준(at) 라이다 프레임으로 정렬 후 병합. clouds=[(stamp_ns, xyzi)]."""
    if not clouds:
        return np.empty((0, 4))
    T_ref_inv = se3_inv(pose_at(times_ns, poses, at_ns))
    w = int(window_s * 1e9)
    out = []
    for s, xyzi in clouds:
        if abs(s - at_ns) > w or len(xyzi) == 0:
            continue
        T = T_ref_inv @ pose_at(times_ns, poses, s)   # lidar@s → lidar@ref
        P = transform(T, xyzi[:, :3])
        out.append(np.hstack([P, xyzi[:, 3:4]]))
    return np.vstack(out) if out else np.empty((0, 4))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd calibration/cam_lidar && python3 -m pytest test_cam_lidar.py -q`
Expected: PASS (전체 통과)

- [ ] **Step 5: Commit**

```bash
git add calibration/cam_lidar/cloud_io.py calibration/cam_lidar/test_cam_lidar.py
git commit -m "feat(calib): cloud_io TUM 로드·자세보간·모션보정 누적 + 테스트

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 6: `accumulate_cloud.py` CLI

bag → 밀집 클라우드(`.npy`) + 대표 시각 출력. 순수 로직은 Task 4·5, 여기선 얇은 CLI.

**Files:**
- Create: `calibration/cam_lidar/accumulate_cloud.py`

**Interfaces:**
- Consumes: `cloud_io.read_clouds`, `voxel_downsample`, `accumulate_static`, `accumulate_motion`, `load_tum`
- Produces: (CLI) 출력 `<out>.npy` (N,4) + stdout에 대표 stamp_ns·점수·범위

- [ ] **Step 1: Implement `accumulate_cloud.py`**

`calibration/cam_lidar/accumulate_cloud.py`:
```python
#!/usr/bin/env python3
"""bag → 밀집 라이다 클라우드(.npy, x,y,z,intensity).

정지(1단계): --window 안의 프레임 병합. 이동(2단계): --trajectory 로 모션 보정.
대표 카메라 이미지는 별도로 bag_extract 로 뽑는다(중복 구현 안 함).

사용:
  source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash
  # 1단계(정지)
  python3 accumulate_cloud.py <bag> -o cloud.npy --window 3
  # 2단계(이동)
  python3 accumulate_cloud.py <bag> -o cloud.npy --trajectory traj.tum --at 12.5 --window 2
"""
import argparse
import sys

import numpy as np

from cloud_io import (accumulate_motion, accumulate_static, load_tum,
                      read_clouds, voxel_downsample)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--topic", default="/unilidar/cloud")
    ap.add_argument("--window", type=float, default=3.0, help="누적 시간창(초)")
    ap.add_argument("--voxel", type=float, default=0.02, help="다운샘플 격자(m), 0이면 끔")
    ap.add_argument("--trajectory", help="TUM 궤적(주면 모션 보정 누적)")
    ap.add_argument("--at", type=float, help="기준 시각(초, bag 시작 기준). 미지정시 첫 스캔")
    args = ap.parse_args()

    clouds = read_clouds(args.bag, args.topic)
    if not clouds:
        print(f"[에러] {args.topic} 스캔 없음", file=sys.stderr)
        sys.exit(1)
    t0 = clouds[0][0]
    at_ns = t0 + int((args.at or 0.0) * 1e9) if args.at is not None else t0

    if args.trajectory:
        times, poses = load_tum(args.trajectory)
        xyzi = accumulate_motion(clouds, times, poses, at_ns, args.window)
        mode = "motion"
    else:
        xyzi = accumulate_static(clouds, at_ns, args.window)
        mode = "static"

    xyzi = voxel_downsample(xyzi, args.voxel)
    np.save(args.out, xyzi)
    lo, hi = xyzi[:, :3].min(0), xyzi[:, :3].max(0)
    print(f"[{mode}] {len(xyzi)} pts → {args.out}  ref_stamp_ns={at_ns}")
    print(f"  범위 x[{lo[0]:.1f},{hi[0]:.1f}] y[{lo[1]:.1f},{hi[1]:.1f}] z[{lo[2]:.1f},{hi[2]:.1f}]")
    print(f"  대표 이미지: ros2 run econ_camera_ros bag_extract {args.bag} -o extracted  (stamp≈{at_ns})")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 수동 스모크 (bag 있는 환경)**

Run:
```bash
cd calibration/cam_lidar
source /opt/ros/humble/setup.bash && source ../../install/setup.bash
python3 accumulate_cloud.py ../../data/sj_bags/260722/<정지bag> -o /tmp/cloud.npy --window 3
python3 -c "import numpy as np; a=np.load('/tmp/cloud.npy'); print(a.shape, a[:2])"
```
Expected: `[static] N pts → ...` + `(N, 4)` 배열. bag 없으면 이 스텝은 실기에서.

- [ ] **Step 3: Commit**

```bash
git add calibration/cam_lidar/accumulate_cloud.py
git commit -m "feat(calib): accumulate_cloud CLI(정지/모션 누적 → .npy)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 7: `solve_extrinsic.py` CLI

대응점 → `T_front_lidar` → `calib.yaml` 기록. 초기값은 대략 장착(rpy/xyz) 또는 이전 단계 값.

**Files:**
- Create: `calibration/cam_lidar/solve_extrinsic.py`

**Interfaces:**
- Consumes: `chain.solve`, `chain.se3_from_rpy_xyz`; `calib_io.load_T_front_lidar`, `write_extrinsic`; `ds_model.load_rig`
- Produces: (CLI) 갱신된 `calib.yaml`, stdout에 RMS·홀드아웃 RMS

- [ ] **Step 1: Implement `solve_extrinsic.py`**

`calibration/cam_lidar/solve_extrinsic.py`:
```python
#!/usr/bin/env python3
"""대응점(correspondences.json) → T_front_lidar → calib.yaml 기록.

correspondences.json = [{"cam":"front","uv":[u,v],"xyz":[x,y,z]}, ...]
초기값: --init-rpy/--init-xyz(대략 장착, deg·m) 또는 calib.yaml 의 기존 T_front_lidar.

사용:
  python3 solve_extrinsic.py correspondences.json \
    --calib ../../data/calib_260723/calib.yaml --orientation orientation.json \
    --init-rpy -90 0 0 --init-xyz 0.05 0 0.1 --holdout 3 --stage 1
"""
import argparse
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "verify"))
from ds_model import load_rig  # noqa: E402

from calib_io import load_T_front_lidar, write_extrinsic  # noqa: E402
from chain import residuals, se3_from_rpy_xyz, se3_to_rvec_tvec, solve  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("corrs")
    ap.add_argument("--calib", required=True)
    ap.add_argument("--orientation", required=True)
    ap.add_argument("--init-rpy", type=float, nargs=3, metavar=("R", "P", "Y"))
    ap.add_argument("--init-xyz", type=float, nargs=3, metavar=("X", "Y", "Z"), default=[0, 0, 0])
    ap.add_argument("--holdout", type=int, default=0, help="검증용으로 제외할 대응점 수")
    ap.add_argument("--huber", type=float, default=2.0)
    ap.add_argument("--stage", type=int, default=1)
    ap.add_argument("--no-write", action="store_true", help="calib.yaml 기록 생략(미리보기)")
    args = ap.parse_args()

    corrs = json.loads(pathlib.Path(args.corrs).read_text())
    rig = load_rig(args.calib, args.orientation)

    if args.init_rpy is not None:
        init_T = se3_from_rpy_xyz(*args.init_rpy, *args.init_xyz)
    else:
        init_T = load_T_front_lidar(args.calib)
        if init_T is None:
            print("[에러] --init-rpy 또는 calib.yaml 의 기존 T_front_lidar 필요", file=sys.stderr)
            sys.exit(1)

    hold = corrs[len(corrs) - args.holdout:] if args.holdout else []
    train = corrs[:len(corrs) - args.holdout] if args.holdout else corrs
    T, rms, per = solve(train, rig, init_T, huber=args.huber)
    print(f"T_front_lidar =\n{np.array2string(T, precision=6)}")
    print(f"train RMS = {rms:.3f} px (n={len(train)}, max={per.max():.2f})")

    hold_rms = None
    if hold:
        # 홀드아웃은 재최적화 없이 학습된 T로 재투영만.
        params = np.concatenate(se3_to_rvec_tvec(T))
        r = residuals(params, hold, rig).reshape(-1, 2)
        hold_rms = float(np.sqrt(np.mean((r ** 2).sum(1))))
        print(f"holdout RMS = {hold_rms:.3f} px (n={len(hold)})")

    if not args.no_write:
        meta = {"method": "manual-pnp-ds", "reproj_rms_px": round(rms, 3), "stage": args.stage}
        if hold_rms is not None:
            meta["holdout_rms_px"] = round(hold_rms, 3)
        write_extrinsic(args.calib, T, meta)
        print(f"→ {args.calib} 에 T_front_lidar + verification.cam_lidar 기록")


if __name__ == "__main__":
    main()
```

> 홀드아웃 RMS는 학습된 `T`로 재투영만 계산한다(재최적화 아님) — 과적합·오클릭 감시용.

- [ ] **Step 2: 수동 스모크 (합성 대응점으로 파이프 확인)**

Run:
```bash
cd calibration/cam_lidar
python3 - <<'PY'
# 합성 대응점 생성 → solve CLI 로 복원(테스트의 _toy 대신 실제 calib 사용)
import json, numpy as np, pathlib, sys
sys.path.insert(0, str(pathlib.Path("../verify").resolve()))
from ds_model import load_rig
from chain import se3_from_rpy_xyz, se3_inv, transform
calib="../../data/calib_260723/calib.yaml"
orient="../../data/calib_260723/orientation.json"  # 없으면 임시 생성
rig=load_rig(calib, orient)
T=se3_from_rpy_xyz(-90,0,0,0.05,-0.03,0.1)
corrs=[]
for name,cam in rig.cams_by_name.items():
    for _ in range(6):
        P=np.array([np.random.uniform(-1,1),np.random.uniform(-.5,.5),np.random.uniform(1,6)])
        u,v,ok=cam.project(P)
        if not ok: continue
        Pf=transform(se3_inv(rig.T_cam_front[name]),P); Pl=transform(se3_inv(T),Pf)
        corrs.append({"cam":name,"uv":[float(u),float(v)],"xyz":Pl.tolist()})
json.dump(corrs, open("/tmp/corrs.json","w"))
print(len(corrs),"corrs")
PY
python3 solve_extrinsic.py /tmp/corrs.json --calib ../../data/calib_260723/calib.yaml \
  --orientation ../../data/calib_260723/orientation.json --init-rpy -80 5 -5 --init-xyz 0.08 0 0.08 --no-write
```
Expected: `train RMS` ≲ 0.01 px, 출력 `T_front_lidar` ≈ 진값. (orientation.json 없으면 `{"cam0":"front",...}` 임시 생성)

- [ ] **Step 3: Commit**

```bash
git add calibration/cam_lidar/solve_extrinsic.py
git commit -m "feat(calib): solve_extrinsic CLI(DS-PnP → calib.yaml 기록)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 8: `pick_correspondences.py` CLI (상호작용)

이미지 2D 클릭 + open3d 3D 클릭 → `correspondences.json`. 상호작용이라 단위 테스트 없음(수동 스모크).

**Files:**
- Create: `calibration/cam_lidar/pick_correspondences.py`

**Interfaces:**
- Consumes: cloud `.npy`(Task 6 산출), 카메라 이미지(들)
- Produces: `correspondences.json` = `[{"cam":str,"uv":[u,v],"xyz":[x,y,z]}, ...]` (Task 7이 소비)

- [ ] **Step 1: Implement `pick_correspondences.py`**

`calibration/cam_lidar/pick_correspondences.py`:
```python
#!/usr/bin/env python3
"""수동 2D–3D 대응점 수집 → correspondences.json.

흐름(카메라 1대씩): (1) 이미지 창에서 물리 점 픽셀 클릭 → (2) open3d 창에서 같은 점 클릭.
open3d VisualizerWithEditing 은 [shift+좌클릭]으로 점 선택, 창 닫으면 인덱스 반환.

사용:
  python3 pick_correspondences.py --cloud cloud.npy --image extracted/frame_000800/cam2.jpg \
    --cam front --out correspondences.json --append
"""
import argparse
import json
import pathlib

import cv2
import numpy as np
import open3d as o3d


def pick_pixels(image_path):
    """이미지 창 클릭 → [(u,v), ...]. 아무키나 누르면 종료."""
    img = cv2.imread(image_path)
    pts = []

    def on_click(ev, x, y, flags, _):
        if ev == cv2.EVENT_LBUTTONDOWN:
            pts.append((float(x), float(y)))
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(img, str(len(pts)), (x + 6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow("image (click points, any key=done)", img)

    cv2.imshow("image (click points, any key=done)", img)
    cv2.setMouseCallback("image (click points, any key=done)", on_click)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return pts


def pick_points_3d(xyzi):
    """open3d 편집 뷰 → 선택 점 인덱스 리스트(shift+좌클릭, 창 닫기=종료)."""
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyzi[:, :3])
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window("cloud (shift+click same points, close=done)")
    vis.add_geometry(pc)
    vis.run()
    vis.destroy_window()
    return vis.get_picked_points()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", required=True)
    ap.add_argument("--image", required=True)
    ap.add_argument("--cam", required=True, help="front/right/rear/left")
    ap.add_argument("--out", required=True)
    ap.add_argument("--append", action="store_true", help="기존 out에 이어붙임")
    args = ap.parse_args()

    xyzi = np.load(args.cloud)
    uv = pick_pixels(args.image)
    idx = pick_points_3d(xyzi)
    n = min(len(uv), len(idx))
    if len(uv) != len(idx):
        print(f"[경고] 2D {len(uv)}개 ≠ 3D {len(idx)}개 → 앞 {n}쌍만 사용")

    corrs = []
    if args.append and pathlib.Path(args.out).exists():
        corrs = json.loads(pathlib.Path(args.out).read_text())
    for i in range(n):
        corrs.append({"cam": args.cam, "uv": list(uv[i]),
                      "xyz": xyzi[idx[i], :3].tolist()})
    pathlib.Path(args.out).write_text(json.dumps(corrs, indent=1))
    print(f"{n}쌍 저장(누적 {len(corrs)}) → {args.out}")


if __name__ == "__main__":
    main()
```

> **클릭 순서 규약:** 2D와 3D를 **같은 순서**로 클릭해야 짝이 맞는다. 카메라별로 실행하고 `--append`로 4대 누적. 회전 관측을 위해 가까운/먼·좌우/상하로 분산해 ~8쌍(+홀드아웃 2~3).
> Task 0에서 open3d 불가로 판정됐다면 `pick_points_3d`를 matplotlib 3D 피커로 교체(대응점 형식 동일).

- [ ] **Step 2: 수동 스모크 (디스플레이 환경)**

Run:
```bash
cd calibration/cam_lidar
python3 pick_correspondences.py --cloud /tmp/cloud.npy \
  --image ../../data/calib_260723/extracted/frame_000800/cam2.jpg \
  --cam front --out /tmp/corrs.json
python3 -c "import json; print(json.load(open('/tmp/corrs.json'))[:2])"
```
Expected: 이미지 창·클라우드 창이 뜨고, 종료 후 `[{'cam':'front','uv':[...],'xyz':[...]}]` 출력.

- [ ] **Step 3: Commit**

```bash
git add calibration/cam_lidar/pick_correspondences.py
git commit -m "feat(calib): pick_correspondences CLI(2D+open3d 3D 클릭 → json)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 9: `overlay_verify.py` CLI

라이다 점을 4대 이미지에 투영 오버레이(깊이 색) + (대응점 있으면) RMS. 정성·정량 검증.

**Files:**
- Create: `calibration/cam_lidar/overlay_verify.py`

**Interfaces:**
- Consumes: `chain.project`; `calib_io.load_T_front_lidar`; `ds_model.load_rig`; cloud `.npy`; 이미지 폴더(`frame_*/camN.jpg` 또는 지정 4장)
- Produces: (CLI) `<out>/overlay_<cam>.png` 4장 + stdout RMS(대응점 주면)

- [ ] **Step 1: Implement `overlay_verify.py`**

`calibration/cam_lidar/overlay_verify.py`:
```python
#!/usr/bin/env python3
"""라이다 점을 4대 어안 이미지에 투영 오버레이(깊이 색) + 선택적 RMS.

체인: pixel = cam.project(T_cam_front · T_front_lidar · P_lidar).
판정: 기둥·박스 엣지·바닥-벽 경계가 라이다 점과 맞물리면 양호(4대 동시 = 체인+360°).

사용:
  python3 overlay_verify.py --cloud cloud.npy --frame ../../data/calib_260723/extracted/frame_000800 \
    --calib ../../data/calib_260723/calib.yaml --orientation orientation.json --out /tmp/overlay
"""
import argparse
import pathlib
import sys

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "verify"))
from ds_model import load_rig  # noqa: E402

from calib_io import load_T_front_lidar  # noqa: E402
from chain import project  # noqa: E402


def _turbo(t):
    t = np.clip(t, 0, 1)
    return (np.stack([np.clip(1.5 - abs(4 * t - 3), 0, 1),
                      np.clip(1.5 - abs(4 * t - 2), 0, 1),
                      np.clip(1.5 - abs(4 * t - 1), 0, 1)], -1) * 255).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", required=True)
    ap.add_argument("--frame", required=True, help="frame_NNNNNN 폴더(cam0..3.jpg)")
    ap.add_argument("--calib", required=True)
    ap.add_argument("--orientation", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--dmax", type=float, default=10.0, help="깊이 색 상한(m)")
    args = ap.parse_args()

    xyzi = np.load(args.cloud)
    rig = load_rig(args.calib, args.orientation)
    T_fl = load_T_front_lidar(args.calib)
    if T_fl is None:
        print("[에러] calib.yaml 에 T_front_lidar 없음(solve_extrinsic 먼저)", file=sys.stderr)
        sys.exit(1)
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    for idx in rig.indices:
        name = rig.idx_to_name[idx]
        cam = rig.cams_by_name[name]
        img = cv2.imread(str(pathlib.Path(args.frame) / f"cam{idx}.jpg"))
        if img is None:
            continue
        u, v, ok = project(xyzi[:, :3], T_fl, rig.T_cam_front[name], cam)
        depth = np.linalg.norm(xyzi[:, :3], axis=1)
        m = ok & (u >= 0) & (u < cam.width) & (v >= 0) & (v < cam.height)
        cols = _turbo(depth[m] / args.dmax)
        ui, vi = u[m].astype(int), v[m].astype(int)
        for x, y, c in zip(ui, vi, cols):
            cv2.circle(img, (x, y), 1, (int(c[2]), int(c[1]), int(c[0])), -1)
        p = out / f"overlay_{name}.png"
        cv2.imwrite(str(p), img)
        print(f"{name}: {m.sum()} pts → {p}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 수동 스모크 (검증 = 눈)**

Run:
```bash
cd calibration/cam_lidar
python3 overlay_verify.py --cloud /tmp/cloud.npy --frame ../../data/calib_260723/extracted/frame_000800 \
  --calib ../../data/calib_260723/calib.yaml --orientation ../../data/calib_260723/orientation.json --out /tmp/overlay
```
Expected: `front: N pts → ...` 4줄, `/tmp/overlay/overlay_*.png` 4장. 엣지 정합을 눈으로 판정.

- [ ] **Step 3: Commit**

```bash
git add calibration/cam_lidar/overlay_verify.py
git commit -m "feat(calib): overlay_verify CLI(4대 투영 오버레이 검증)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 10: 운영 문서 + CLAUDE.md 갱신

전체 절차를 `docs/CAM_LIDAR_CALIBRATION.md`로 남기고 프로젝트 인덱스에 연결.

**Files:**
- Create: `docs/CAM_LIDAR_CALIBRATION.md`
- Modify: `CLAUDE.md` (상세 문서 목록 + 현재 상태에 cam-lidar 한 줄)
- Modify: `docs/CALIBRATION.md:9` (cam↔LiDAR 범위 밖 → 문서 링크로 교체)

- [ ] **Step 1: Write `docs/CAM_LIDAR_CALIBRATION.md`**

아래 골격으로 작성(각 절은 실제 명령·판정 기준 포함, 플레이스홀더 금지):
```markdown
# Cam–LiDAR Extrinsic 캘리브레이션 가이드

어안 4대(DS)와 Unitree L2 사이 T_front_lidar 를 수동 2D–3D 대응+DS-PnP 로 구해 calib.yaml 에 기록.
설계: superpowers/specs/2026-07-23-cam-lidar-extrinsic-calibration-design.md

## 0. 사전
- pip install open3d scipy opencv-python  (open3d arm64 결과는 calibration/cam_lidar/README.md)
- calib.yaml + orientation.json (카메라 캘리브 산출물) 준비

## 1. 촬영(1단계, 정지)
리그 완전 고정, 앞에 모서리 뚜렷한 물체(박스·구조물)를 라이다·카메라가 함께 보게.
ros2 launch econ_camera_ros record_all.launch.py  # 정지 10–20s, Ctrl-C

## 2. 누적 + 대표 이미지
python3 accumulate_cloud.py <bag> -o cloud.npy --window 3
ros2 run econ_camera_ros bag_extract <bag> -o extracted   # 대표 stamp 근처 frame 사용

## 3. 대응점(디스플레이 필요, ~8쌍+홀드아웃, 카메라별 --append)
python3 pick_correspondences.py --cloud cloud.npy --image extracted/frame_XXXXXX/cam2.jpg \
  --cam front --out corrs.json --append
# 요령: 2D·3D 같은 순서 클릭. 가까운/먼·좌우/상하 분산(회전 관측).

## 4. 풀기 + 기록
python3 solve_extrinsic.py corrs.json --calib calib.yaml --orientation orientation.json \
  --init-rpy <대략 장착 rpy> --init-xyz <대략 위치> --holdout 3 --stage 1
# 판정: train/holdout RMS ≲ 2px

## 5. 검증(눈)
python3 overlay_verify.py --cloud cloud.npy --frame extracted/frame_XXXXXX \
  --calib calib.yaml --orientation orientation.json --out overlay
# 판정: 4대 모두 엣지 정합. 회전 어긋나면 6절(2단계) 또는 재대응.

## 6. 2단계(현장 bag 복원, 조건부)
5절에서 드리프트가 보이면: 양호 LIO bag(DATASET.md)에서
python3 accumulate_cloud.py <field_bag> -o fcloud.npy --trajectory traj.tum --at <t> --window 2
그 뒤 3–5절 반복(solve 는 --init 없이 calib.yaml 기존 T_front_lidar 를 초기값으로).

## 7. 결과 판정 / 문제해결
- RMS ≲2px & 4대 오버레이 엣지 정합 → 양호. calib.yaml.extrinsics.T_front_lidar 확정.
- RMS 큼 → 대응점 기하 다양성 부족(가까운/먼 섞기) 또는 오클릭 → 재수집.
- 투영이 통째로 어긋남 → --init-rpy 부호/축 재확인(라이다가 보는 방향).
```

- [ ] **Step 2: CLAUDE.md · CALIBRATION.md 링크 갱신**

`CLAUDE.md`의 "상세 문서" 목록에 추가:
```markdown
- **Cam-LiDAR 캘리브 가이드**: `docs/CAM_LIDAR_CALIBRATION.md` (T_front_lidar, 수동 대응+DS-PnP, 2단계)
```
`docs/CALIBRATION.md:9`의 `- cam↔LiDAR extrinsic 은 L2 도착 후(본 문서 범위 밖).` 를:
```markdown
- cam↔LiDAR extrinsic(`T_front_lidar`)은 별도: `docs/CAM_LIDAR_CALIBRATION.md`.
```

- [ ] **Step 3: 전체 테스트 회귀 확인**

Run:
```bash
cd calibration/cam_lidar && python3 -m pytest -q
cd ../verify && python3 -m pytest -q
cd ../../src/econ_camera_ros && python3 -m pytest test/ -q
```
Expected: 세 스위트 모두 PASS(기존 18개 + 신규 회귀 없음)

- [ ] **Step 4: Commit**

```bash
git add docs/CAM_LIDAR_CALIBRATION.md CLAUDE.md docs/CALIBRATION.md
git commit -m "docs(calib): Cam-LiDAR 캘리브 가이드 + 인덱스 링크

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## 완료 기준 (전체)

- [ ] `calibration/cam_lidar/` 순수 로직 테스트 전부 PASS, 기존 verify·econ_camera_ros 테스트 회귀 없음.
- [ ] 합성 대응점으로 `solve` 가 알려진 extrinsic 을 <1e-3 px 로 복원(Task 2).
- [ ] (실기) 1단계 통제 촬영 → 대응점 → solve RMS ≲2px → 4대 오버레이 엣지 정합.
- [ ] `calib.yaml` 에 `T_front_lidar` + `verification.cam_lidar` 기록.
- [ ] 문서·인덱스 갱신.

## 자기 검토 메모(스펙 대비)

- 스펙 §3 도구 4개 → Task 6·7·8·9. 공용 로직(§2 잔차·체인) → Task 1·2. 누적/모션(§3.1) → Task 4·5.
- 스펙 §5 calib.yaml 확장 → Task 3(`write_extrinsic`)로 정확히 구현(키·verification 병합).
- 스펙 §6 리스크(open3d arm64) → Task 0 선차단 + 대체 경로 명시.
- 스펙 §7 검증(정량 RMS·홀드아웃·정성 오버레이·교차확인) → Task 2(합성)·7(holdout)·9(오버레이).
- 자동 엣지정렬(스펙 §8)은 범위 밖 — 계획에 미포함(의도적).
```
