#!/usr/bin/env python3
"""360° 원통 파노라마 스티칭: extrinsic 으로 4대를 하나의 파노라마에 합성.

intrinsic(각 카메라가 잘 펴지는가)과 extrinsic(카메라 간 상대자세가 맞는가)을 동시에
검증한다. 이음새에서 장면이 매끄럽게 이어지면 양쪽 다 양호. 회전-전용 근사라 가까운
물체의 이음새 어긋남(시차)은 정상 -> 멀리 있는 구조물의 정렬을 본다.

두 산출물: 블렌딩(feather) 파노라마 + 카메라 경계 표시(hard-seam)로 경계 정렬 확인.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from common import add_common_args, label, list_frames, load_frame, parse_frames, render_into
from ds_model import load_rig
from rectify import cylindrical_rays


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p, default_out="data/calib_260723/verify/panorama")
    p.add_argument("--width", type=int, default=2400, help="파노라마 가로(360도)")
    p.add_argument("--height", type=int, default=700)
    p.add_argument("--vfov", type=float, default=120.0, help="세로 화각(도) — 파노라마 높이 스케일")
    a = p.parse_args()

    rig = load_rig(a.calib, a.orientation)
    frames = parse_frames(a.frames, list_frames(a.images))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    if not frames:
        print("선택된 프레임 없음"); return

    # front 프레임 기준, 방위각 [-180,180] 전방위 원통 ray
    hh = int(a.width * (a.vfov / 360.0))
    rays = cylindrical_rays(a.width, min(hh, a.height), hfov_deg=360.0, az_span_deg=360.0)
    front_name = rig.idx_to_name[0]

    # 카메라별 색상(hard-seam 시각화용)
    tints = {0: (60, 60, 255), 1: (60, 255, 60), 2: (255, 160, 60), 3: (255, 60, 200)}

    for fn in frames:
        imgs = load_frame(a.images, fn, rig.indices)
        if len(imgs) < 2:
            continue
        acc = np.zeros((rays.shape[0], rays.shape[1], 3), np.float64)
        wsum = np.zeros(rays.shape[:2], np.float64)
        wmax = np.zeros(rays.shape[:2], np.float64)
        owner = -np.ones(rays.shape[:2], np.int32)
        for idx, img in imgs.items():
            cam = rig.cam_for_index(idx)
            R = rig.T_cam_front[cam.name][:3, :3]  # front -> cam
            rend, w = render_into(cam, img, rays, R)
            acc += rend.astype(np.float64) * w[..., None]
            wsum += w
            newown = w > wmax
            owner[newown] = idx
            wmax[newown] = w[newown]

        blend = np.zeros_like(acc, np.uint8)
        m = wsum > 1e-6
        blend[m] = (acc[m] / wsum[m, None]).clip(0, 255).astype(np.uint8)

        # hard-seam: 각 픽셀을 최대가중 카메라에 배정, 경계선 그리기
        seam = blend.copy()
        edge = np.zeros(owner.shape, np.uint8)
        edge[:, 1:][owner[:, 1:] != owner[:, :-1]] = 255
        edge[1:, :][owner[1:, :] != owner[:-1, :]] = 255
        edge = cv2.dilate(edge, np.ones((3, 3), np.uint8))
        seam[edge > 0] = (0, 255, 255)

        cv2.imwrite(str(out / f"{fn}_pano_blend.png"),
                    label(blend, f"{fn}  360 PANO (ref={front_name})"))
        cv2.imwrite(str(out / f"{fn}_pano_seam.png"),
                    label(seam, f"{fn}  360 PANO seams(yellow)=cam boundary"))
        print("wrote", out / f"{fn}_pano_blend.png")


if __name__ == "__main__":
    main()
