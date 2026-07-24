"""Double Sphere (DS) 카메라 모델: 투영/역투영과 calib.yaml 로더.

OpenCV에는 DS 모델이 없어서 여기서 직접 구현한다(Usenko et al. 2018 폐형식).
모든 시각 검증 도구의 토대다.

DS intrinsics 순서: [xi, alpha, fx, fy, cx, cy]  (calib.yaml / Kalibr camchain과 동일)
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

_EPS = 1e-9


@dataclass
class DoubleSphereCamera:
    xi: float
    alpha: float
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    name: str = ""

    @classmethod
    def from_intrinsics(cls, intr, resolution, name=""):
        xi, alpha, fx, fy, cx, cy = intr
        w, h = resolution
        return cls(xi, alpha, fx, fy, cx, cy, int(w), int(h), name)

    # ---- 투영: 3D 점 -> 픽셀 --------------------------------------------
    def project(self, P):
        """P: (..., 3) 카메라 좌표계 3D 점 -> (u, v, valid).

        valid=False 인 점은 DS 모델 정의역 밖(카메라 뒤 등)이라 (u,v)를 신뢰하면 안 됨.
        """
        P = np.asarray(P, dtype=np.float64)
        x, y, z = P[..., 0], P[..., 1], P[..., 2]
        xi, al = self.xi, self.alpha

        d1 = np.sqrt(x * x + y * y + z * z)
        k = xi * d1 + z
        d2 = np.sqrt(x * x + y * y + k * k)
        denom = al * d2 + (1.0 - al) * k

        w1 = al / (1.0 - al) if al <= 0.5 else (1.0 - al) / al
        w2 = (w1 + xi) / np.sqrt(2.0 * w1 * xi + xi * xi + 1.0)
        valid = (z > -w2 * d1) & (np.abs(denom) > _EPS)

        safe = np.where(np.abs(denom) > _EPS, denom, 1.0)
        u = self.fx * x / safe + self.cx
        v = self.fy * y / safe + self.cy
        return u, v, valid

    # ---- 역투영: 픽셀 -> 단위 방향벡터 ----------------------------------
    def unproject(self, u, v):
        """u, v: 픽셀 좌표(배열 가능) -> (dirs(...,3) 단위벡터, valid).

        valid=False 픽셀은 모델 정의역 밖(어안 유효원 바깥 등)이라 방향을 신뢰 불가.
        """
        u = np.asarray(u, dtype=np.float64)
        v = np.asarray(v, dtype=np.float64)
        xi, al = self.xi, self.alpha

        mx = (u - self.cx) / self.fx
        my = (v - self.cy) / self.fy
        r2 = mx * mx + my * my

        if al > 0.5:
            valid = r2 <= 1.0 / (2.0 * al - 1.0)
        else:
            valid = np.ones_like(r2, dtype=bool)
        r2c = np.where(valid, r2, 0.0)

        mz = (1.0 - al * al * r2c) / (al * np.sqrt(np.maximum(1.0 - (2.0 * al - 1.0) * r2c, 0.0)) + (1.0 - al))
        num = mz * xi + np.sqrt(np.maximum(mz * mz + (1.0 - xi * xi) * r2c, 0.0))
        coef = num / (mz * mz + r2c + _EPS)

        x = coef * mx
        y = coef * my
        zz = coef * mz - xi
        dirs = np.stack([x, y, zz], axis=-1)
        n = np.linalg.norm(dirs, axis=-1, keepdims=True)
        dirs = dirs / np.maximum(n, _EPS)
        return dirs, valid


@dataclass
class CameraRig:
    """calib.yaml + orientation.json 를 묶은 4-카메라 리그."""

    cams_by_name: dict          # name -> DoubleSphereCamera
    T_cam_front: dict           # name -> 4x4 (front 프레임 점 -> cam 프레임)
    idx_to_name: dict           # 0..3 -> name  (cam{idx}.jpg 매핑)

    def cam_for_index(self, idx: int) -> DoubleSphereCamera:
        return self.cams_by_name[self.idx_to_name[idx]]

    @property
    def indices(self):
        return sorted(self.idx_to_name)


def load_rig(calib_path, orientation_path) -> CameraRig:
    calib = yaml.safe_load(Path(calib_path).read_text())
    orient = json.loads(Path(orientation_path).read_text())

    cams = {}
    for name, c in calib["cameras"].items():
        cams[name] = DoubleSphereCamera.from_intrinsics(
            c["intrinsics"], c["resolution"], name=name)

    extr = {}
    for key, mat in calib["extrinsics"].items():
        # 카메라 extrinsic 키만(T_<cam>_front). T_front_lidar 등 비-카메라 키는 무시.
        if not (key.startswith("T_") and key.endswith("_front")):
            continue
        name = key[len("T_"):-len("_front")]
        if name not in cams:
            continue
        extr[name] = np.array(mat, dtype=np.float64)

    idx_to_name = {int(k.replace("cam", "")): v for k, v in orient.items()}
    return CameraRig(cams, extr, idx_to_name)
