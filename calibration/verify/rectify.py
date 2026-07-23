"""역투영 기반 리매핑: 어안(DS) -> 핀홀/원통 파노라마.

원리: 출력 이미지의 각 픽셀이 바라보는 3D ray를 만들고, 그 ray를 DS.project 로
소스 어안 픽셀에 투영해 (map_x, map_y)를 구성 -> cv2.remap.
straight-line 검사(핀홀), 전체 수평화각 확인(원통)에 쓴다.
"""
from __future__ import annotations

import cv2
import numpy as np


def pinhole_rays(w_out, h_out, hfov_deg):
    """정면을 바라보는 가상 핀홀의 ray 맵 (h_out, w_out, 3), z=+1 정규화 전."""
    f = (w_out / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
    cx, cy = w_out / 2.0, h_out / 2.0
    uu, vv = np.meshgrid(np.arange(w_out), np.arange(h_out))
    x = (uu - cx) / f
    y = (vv - cy) / f
    z = np.ones_like(x)
    return np.stack([x, y, z], axis=-1)


def cylindrical_rays(w_out, h_out, hfov_deg, vfov_deg=None, az_center_deg=0.0, az_span_deg=None):
    """세로축(카메라 +y) 기준 원통 파노라마 ray 맵.

    가로 = 방위각 theta, 세로 = 높이(선형). 세상 수직선이 수직으로 유지된다.
    az_span_deg 를 주면 [center-span/2, center+span/2] 범위(360 파노라마용).
    """
    span = az_span_deg if az_span_deg is not None else hfov_deg
    ppr = w_out / np.radians(span)  # 픽셀 per 라디안
    theta = np.radians(az_center_deg) + (np.arange(w_out) - w_out / 2.0) / ppr
    h = (np.arange(h_out) - h_out / 2.0) / ppr
    th, hh = np.meshgrid(theta, h)
    x = np.sin(th)
    y = hh
    z = np.cos(th)
    dirs = np.stack([x, y, z], axis=-1)
    dirs /= np.linalg.norm(dirs, axis=-1, keepdims=True)
    return dirs


def rays_to_maps(cam, rays):
    """ray 맵(...,3) -> (map_x, map_y, valid). 소스 이미지 밖/모델 밖은 valid=False."""
    u, v, ok = cam.project(rays)
    inb = (u >= 0) & (u <= cam.width - 1) & (v >= 0) & (v <= cam.height - 1)
    valid = ok & inb
    map_x = np.where(valid, u, -1).astype(np.float32)
    map_y = np.where(valid, v, -1).astype(np.float32)
    return map_x, map_y, valid


def render(cam, img, rays):
    """어안 img 를 ray 맵 시점으로 리매핑한 결과와 valid 마스크."""
    map_x, map_y, valid = rays_to_maps(cam, rays)
    out = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    out[~valid] = 0
    return out, valid
