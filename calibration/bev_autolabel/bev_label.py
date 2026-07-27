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
