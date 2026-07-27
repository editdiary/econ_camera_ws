#!/usr/bin/env python3
"""단계1 품질 검증: 지정 프레임들의 build_label 결과를 review PNG로 저장.

사용:
  cd calibration/bev_autolabel
  python3 verify_labels.py \
    --map-dir  ../../data/sj_bags/260722/raws3_mapping \
    --extract-dir ../../data/cam_out/extracted \
    --calib ../../data/calib_260723/calib.yaml \
    --orient ../../data/calib_260723/orientation.json \
    --frames 900 2000 2500 4850 \
    --out ../../data/sj_bags/260722/raws3_mapping
"""
import argparse
import pathlib
import sys

import cv2

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
from calib_io import load_T_front_lidar  # noqa: E402
from ds_model import load_rig            # noqa: E402
import bev_io                            # noqa: E402
import render                            # noqa: E402
from bev_label import BevSpec, build_label, compute_self_voxels  # noqa: E402

USE = ("front", "left", "right")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-dir", required=True)
    ap.add_argument("--extract-dir", required=True)
    ap.add_argument("--calib", required=True)
    ap.add_argument("--orient", required=True)
    ap.add_argument("--frames", type=int, nargs="+", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    spec = BevSpec()
    rig = load_rig(a.calib, a.orient)
    T_front_lidar = load_T_front_lidar(a.calib)
    p, times, poses, tpos = bev_io.load_map(a.map_dir)
    stamps = bev_io.load_stamps(a.extract_dir)
    print(f"map points={len(p)} poses={len(poses)}")
    self_vox = compute_self_voxels(p, poses)
    print(f"self voxels={len(self_vox)}")

    out = pathlib.Path(a.out)
    for idx in a.frames:
        t_ns = stamps[idx]
        lab = build_label(p, times, poses, tpos, self_vox,
                          rig.cams_by_name, rig.T_cam_front, T_front_lidar, t_ns, spec)
        imgs = bev_io.load_cam_images(a.extract_dir, idx, rig.idx_to_name, USE)
        rev = render.review_image(lab, spec, imgs)
        dst = out / f"bev_review_{idx:06d}.png"
        cv2.imwrite(str(dst), rev)
        print(f"saved {dst}")


if __name__ == "__main__":
    main()
