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
