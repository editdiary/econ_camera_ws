#!/usr/bin/env python3
"""카메라 간 extrinsic 검증: 인접 쌍의 겹침 영역을 겹쳐 정렬 확인(검출 불필요).

기준 카메라 A 시야(원통)에 이웃 B 를 extrinsic 회전으로 워핑해 겹친다.
- checkerboard: A/B 를 바둑판으로 교차 -> 경계에서 선/모서리가 어긋나면 extrinsic 오차.
- blend: 겹침 영역 50/50 -> 유령현상(ghosting) 이 적을수록 양호.
회전-전용 근사라 가까운 물체의 시차는 정상. 멀리 있는 구조물의 정렬을 본다.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from common import add_common_args, label, list_frames, load_frame, parse_frames, render_into
from ds_model import load_rig
from rectify import cylindrical_rays


def checkerboard(a_img, b_img, mask, tile=48):
    h, w = mask.shape
    yy, xx = np.mgrid[0:h, 0:w]
    board = ((yy // tile + xx // tile) % 2).astype(bool)
    out = a_img.copy()
    take_b = mask & board
    out[take_b] = b_img[take_b]
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p, default_out="data/calib_260723/verify/extrinsics")
    p.add_argument("--fov", type=float, default=200.0, help="기준 카메라 원통 수평화각(도)")
    p.add_argument("--width", type=int, default=1500)
    p.add_argument("--height", type=int, default=650)
    a = p.parse_args()

    rig = load_rig(a.calib, a.orientation)
    frames = parse_frames(a.frames, list_frames(a.images))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    if not frames:
        print("선택된 프레임 없음"); return

    idxs = rig.indices
    pairs = [(idxs[i], idxs[(i + 1) % len(idxs)]) for i in range(len(idxs))]  # 링 인접쌍
    rays = cylindrical_rays(a.width, a.height, hfov_deg=a.fov)

    for fn in frames:
        imgs = load_frame(a.images, fn, idxs)
        for A, B in pairs:
            if A not in imgs or B not in imgs:
                continue
            camA, camB = rig.cam_for_index(A), rig.cam_for_index(B)
            RA = rig.T_cam_front[camA.name][:3, :3]
            RB = rig.T_cam_front[camB.name][:3, :3]
            R_A_ref = np.eye(3)               # 기준 = A
            R_B_A = RB @ RA.T                  # A 프레임 -> B 프레임
            imA, wA = render_into(camA, imgs[A], rays, R_A_ref)
            imB, wB = render_into(camB, imgs[B], rays, R_B_A)
            overlap = (wA > 1e-6) & (wB > 1e-6)
            if overlap.mean() < 0.002:
                continue

            chk = checkerboard(imA, imB, overlap)
            chk = label(chk, f"{fn}  {camA.name}(base) x {camB.name}  overlap={overlap.mean()*100:.1f}%")
            blend = imA.copy()
            blend[overlap] = (0.5 * imA[overlap] + 0.5 * imB[overlap]).astype(np.uint8)
            blend = label(blend, f"{fn}  {camA.name}+{camB.name} 50/50 blend")

            stack = np.vstack([chk, np.full((6, chk.shape[1], 3), 40, np.uint8), blend])
            dst = out / f"{fn}_{camA.name}_{camB.name}.png"
            cv2.imwrite(str(dst), stack)
            print("wrote", dst)


if __name__ == "__main__":
    main()
