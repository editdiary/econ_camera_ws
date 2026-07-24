"""순수 로직 테스트(ROS·이미지·bag 불필요).

실행: cd calibration/cam_lidar && python3 -m pytest -q
"""
import numpy as np

from chain import se3, se3_inv, se3_to_rvec_tvec, se3_from_rpy_xyz, transform
from chain import project, residuals, solve

# ds_model 재사용(형제 디렉터리 verify/)
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "verify"))
from ds_model import DoubleSphereCamera, CameraRig, load_rig


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


# ---- T_front_lidar 로더/쓰기 테스트 ----
import yaml
from calib_io import load_T_front_lidar, write_extrinsic

# load_rig 이미 가져왔으므로 여기서 재사용. 위 sys.path 설정으로 ds_model 접근 가능.

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
