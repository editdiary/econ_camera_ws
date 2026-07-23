#!/usr/bin/env python3
"""언디스토션 검증: 어안 원본 | 핀홀(직선성) | 원통 파노라마(전체 수평화각).

세상의 직선이 곧게 펴지는지 눈으로 확인한다. calib 보드 이미지든 직접 수집한
일반 이미지든 동일하게 동작(검출 불필요, DS 투영/역투영만 사용).

예:
  python3 verify_undistort.py --images data/calib_260723/extracted
  python3 verify_undistort.py --images data/cam_out/raws1_out-images --frames 0,100,250
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from common import add_common_args, hstack_pad, label, list_frames, load_frame, parse_frames
from ds_model import load_rig
from rectify import cylindrical_rays, pinhole_rays, render


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p, default_out="data/calib_260723/verify/undistort")
    p.add_argument("--pin-fov", type=float, default=90.0, help="핀홀 수평화각(도)")
    p.add_argument("--cyl-fov", type=float, default=160.0, help="원통 수평화각(도)")
    a = p.parse_args()

    rig = load_rig(a.calib, a.orientation)
    frames = parse_frames(a.frames, list_frames(a.images))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    if not frames:
        print("선택된 프레임 없음"); return

    pin_rays = pinhole_rays(900, 900, a.pin_fov)
    cyl_rays = cylindrical_rays(1400, 500, a.cyl_fov)

    for fn in frames:
        imgs = load_frame(a.images, fn, rig.indices)
        for idx, img in imgs.items():
            cam = rig.cam_for_index(idx)
            ph, _ = render(cam, img, pin_rays)
            cyl, _ = render(cam, img, cyl_rays)
            row = hstack_pad([
                label(img, f"{fn} cam{idx}({cam.name}) RAW"),
                label(ph, f"PINHOLE {a.pin_fov:.0f}deg", color=(0, 220, 255)),
                label(cyl, f"CYL {a.cyl_fov:.0f}deg", color=(255, 200, 0)),
            ])
            dst = out / f"{fn}_cam{idx}_{cam.name}.png"
            cv2.imwrite(str(dst), row)
            print("wrote", dst)


if __name__ == "__main__":
    main()
