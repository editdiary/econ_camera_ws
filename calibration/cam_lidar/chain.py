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
