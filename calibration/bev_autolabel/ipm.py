"""이미지 전체 → 지면 평면 IPM 투영(BEV RGB 캔버스). 순수 로직, LiDAR·pose·map 불필요.

마스트 LiDAR 는 바닥을 못 봐(관측점이 설치 높이 위쪽에만 존재) map 기반 바닥 컬러화가
불가능하다. 그래서 바닥 표현은 카메라 이미지를 flat-ground(지면 z=z0=C_z-cam_height)로
역투영해 얻는다. 평평한 통로는 정확히 펴지고, 높이가 있는 물체(작물 등)는 방사상으로
번진다(IPM 본질). cam_height 는 카메라 렌즈의 바닥 위 높이[m] 실측값(외부 입력).

좌표계는 bev_label 과 동일: ego=body=LiDAR, x=전방·y=좌·z=상. BEV row=(XF-x)/RES, col=(YH-y)/RES.
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
from chain import se3_inv  # noqa: E402

USE = ("front", "left", "right")


def ipm_project_rgb(img, cam, T_cam_lidar, cam_height, spec):
    """이미지(H,W,3 BGR) → (sumBGR(NX,NY,3) float64, cnt(NX,NY) int64).

    각 픽셀을 광선으로 역투영해 지면 평면(z0=C_z-cam_height)과의 교점을 구하고, 그 교점이
    속한 BEV 셀에 픽셀 색을 누적한다. 아래로 향하는(바닥과 교차하는) 광선만 사용.
    ipm_review 시절의 ipm_project_mask 와 동일한 기하이며, 마스크 대신 RGB 를 나른다.
    """
    T_lidar_cam = se3_inv(T_cam_lidar)
    C = T_lidar_cam[:3, 3]
    R = T_lidar_cam[:3, :3]
    z0 = C[2] - cam_height
    H, W = img.shape[:2]
    vs, us = np.mgrid[0:H, 0:W]
    us = us.ravel().astype(np.float64)
    vs = vs.ravel().astype(np.float64)
    dirs, valid = cam.unproject(us, vs)
    d = dirs @ R.T                                     # 광선을 ego 프레임으로 회전
    dz = d[:, 2]
    ok = valid & (dz < -1e-6)                          # 아래로 향하는 광선만 바닥과 교차
    t = np.where(ok, (z0 - C[2]) / np.where(dz == 0, 1.0, dz), -1.0)
    ok &= t > 0
    X = C[0] + t * d[:, 0]
    Y = C[1] + t * d[:, 1]
    ok &= (X <= spec.XF) & (X >= -spec.XR) & (np.abs(Y) <= spec.YH)
    row = np.round((spec.XF - X) / spec.RES).astype(int)
    col = np.round((spec.YH - Y) / spec.RES).astype(int)
    ok &= (row >= 0) & (row < spec.NX) & (col >= 0) & (col < spec.NY)

    sumbgr = np.zeros((spec.NX, spec.NY, 3), np.float64)
    cnt = np.zeros((spec.NX, spec.NY), np.int64)
    color = img.reshape(-1, 3).astype(np.float64)
    r, c = row[ok], col[ok]
    for ch in range(3):
        np.add.at(sumbgr[:, :, ch], (r, c), color[ok, ch])
    np.add.at(cnt, (r, c), 1)
    return sumbgr, cnt


def ipm_canvas(imgs, cams_by_name, T_cam_front, T_front_lidar, cam_height, spec,
               use_names=USE):
    """여러 카메라 IPM 투영을 누적·평균 → BEV RGB 캔버스(NX,NY,3 BGR uint8). 빈 셀=0."""
    sumbgr = np.zeros((spec.NX, spec.NY, 3), np.float64)
    cnt = np.zeros((spec.NX, spec.NY), np.int64)
    for name in use_names:
        im = imgs.get(name)
        if im is None:
            continue
        T_cam_lidar = T_cam_front[name] @ T_front_lidar
        s, c = ipm_project_rgb(im, cams_by_name[name], T_cam_lidar, cam_height, spec)
        sumbgr += s
        cnt += c
    out = np.zeros((spec.NX, spec.NY, 3), np.uint8)
    m = cnt > 0
    out[m] = (sumbgr[m] / cnt[m, None]).astype(np.uint8)
    return out
