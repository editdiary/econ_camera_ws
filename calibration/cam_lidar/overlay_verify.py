#!/usr/bin/env python3
"""라이다 점을 4대 어안 이미지에 투영 오버레이(깊이 색) + 선택적 RMS.

체인: pixel = cam.project(T_cam_front · T_front_lidar · P_lidar).
판정: 기둥·박스 엣지·바닥-벽 경계가 라이다 점과 맞물리면 양호(4대 동시 = 체인+360°).

사용:
  python3 overlay_verify.py --cloud cloud.npy --frame ../../data/calib_260723/extracted/frame_000800 \
    --calib ../../data/calib_260723/calib.yaml --orientation orientation.json --out /tmp/overlay
"""
import argparse
import pathlib
import sys

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "verify"))
from ds_model import load_rig  # noqa: E402

from calib_io import load_T_front_lidar  # noqa: E402
from chain import project  # noqa: E402


def _turbo(t):
    t = np.clip(t, 0, 1)
    return (np.stack([np.clip(1.5 - abs(4 * t - 3), 0, 1),
                      np.clip(1.5 - abs(4 * t - 2), 0, 1),
                      np.clip(1.5 - abs(4 * t - 1), 0, 1)], -1) * 255).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", required=True)
    ap.add_argument("--frame", required=True, help="frame_NNNNNN 폴더(cam0..3.jpg)")
    ap.add_argument("--calib", required=True)
    ap.add_argument("--orientation", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--dmax", type=float, default=10.0, help="깊이 색 상한(m)")
    args = ap.parse_args()

    xyzi = np.load(args.cloud)
    rig = load_rig(args.calib, args.orientation)
    T_fl = load_T_front_lidar(args.calib)
    if T_fl is None:
        print("[에러] calib.yaml 에 T_front_lidar 없음(solve_extrinsic 먼저)", file=sys.stderr)
        sys.exit(1)
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    for idx in rig.indices:
        name = rig.idx_to_name[idx]
        cam = rig.cams_by_name[name]
        img = cv2.imread(str(pathlib.Path(args.frame) / f"cam{idx}.jpg"))
        if img is None:
            continue
        u, v, ok = project(xyzi[:, :3], T_fl, rig.T_cam_front[name], cam)
        depth = np.linalg.norm(xyzi[:, :3], axis=1)
        m = ok & (u >= 0) & (u < cam.width) & (v >= 0) & (v < cam.height)
        cols = _turbo(depth[m] / args.dmax)
        ui, vi = u[m].astype(int), v[m].astype(int)
        for x, y, c in zip(ui, vi, cols):
            cv2.circle(img, (x, y), 1, (int(c[2]), int(c[1]), int(c[0])), -1)
        p = out / f"overlay_{name}.png"
        cv2.imwrite(str(p), img)
        print(f"{name}: {m.sum()} pts → {p}")


if __name__ == "__main__":
    main()
