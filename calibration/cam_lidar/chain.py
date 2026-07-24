"""라이다→카메라 투영 체인과 DS-PnP 최적화(순수 로직).

extrinsic 규약: T_<target>_<source> = source 점 → target 점.
투영: pixel = cam.project(T_cam_front · T_front_lidar · P_lidar).
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares


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
