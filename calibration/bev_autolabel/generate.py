#!/usr/bin/env python3
"""단계2: bag의 LIO 맵 + 추출 이미지 → BEV auto-label 데이터셋 일괄 생성.

각 키프레임마다 (1) LiDAR+맵 기하로 auto-label(0/1/2) 을 만들고, (2) 3어안을 지면 평면에
IPM 투영해 BEV RGB 캔버스를 만든 뒤, (3) 캔버스 위에 라벨을 얹은 검수뷰를 낸다. 사람은
카메라 이미지에 마스크를 그리는 대신, 이 BEV 뷰에서 미리 채워진 라벨을 보정만 하면 된다.

샘플: sample_NNNNNN/{label.png(인덱스 팔레트 0/1/2), ipm_rgb.png(BEV RGB 캔버스),
      overlay.png(80×80 ipm_rgb+라벨 오버레이, 장식 없음=CVAT annotation base),
      review.png(확대 검수뷰+원본 3어안), cam_{front,left,right}.jpg, meta.json} + dataset.csv

사용:
  cd calibration/bev_autolabel
  python3 generate.py \
    --map-dir ../../data/sj_bags/260722/raws3_mapping \
    --extract-dir ../../data/cam_out/extracted \
    --calib ../../data/calib_260723/calib.yaml \
    --orient ../../data/calib_260723/orientation.json \
    --out ../../data/bev/dataset/raws3 --kf-step 0.4 --cam-height 0.87
"""
import argparse
import csv
import json
import pathlib
import sys

import numpy as np
import cv2
from PIL import Image

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
from calib_io import load_T_front_lidar  # noqa: E402
from cloud_io import pose_at             # noqa: E402
from ds_model import load_rig            # noqa: E402
import bev_io                            # noqa: E402
import render                            # noqa: E402
import ipm                               # noqa: E402
from bev_label import BevSpec, build_label, compute_self_voxels, select_keyframes  # noqa: E402

USE = ("front", "left", "right")
_PALETTE = [255, 0, 0, 0, 200, 0, 128, 128, 128] + [0] * (256 * 3 - 9)  # RGB: 0빨강 1초록 2회색


def save_label_png(path, label):
    im = Image.fromarray(label, mode="P")
    im.putpalette(_PALETTE)
    im.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-dir", required=True)
    ap.add_argument("--extract-dir", required=True)
    ap.add_argument("--calib", required=True)
    ap.add_argument("--orient", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--kf-step", type=float, default=0.4)
    ap.add_argument("--limit", type=int, default=0, help="테스트용 최대 샘플 수(0=전체)")
    ap.add_argument("--z-gate", type=float, default=0.3)
    ap.add_argument("--cam-height", type=float, default=0.87,
                    help="IPM 지면 평면용 카메라 렌즈의 바닥 위 높이[m] 실측값")
    ap.add_argument("--alpha", type=float, default=0.45, help="review 라벨 오버레이 불투명도")
    ap.add_argument("--blend", choices=("nearest", "average"), default="nearest",
                    help="IPM 다중카메라 합성: nearest(셀별 최근접 1대, 기본)|average(평균)")
    a = ap.parse_args()

    spec = BevSpec()
    rig = load_rig(a.calib, a.orient)
    T_front_lidar = load_T_front_lidar(a.calib)
    if T_front_lidar is None:
        sys.exit("calib.yaml에 extrinsics.T_front_lidar가 없습니다 (Cam-LiDAR 캘리브 필요)")
    p, times, poses, tpos = bev_io.load_map(a.map_dir)
    stamps = bev_io.load_stamps(a.extract_dir)
    self_vox = compute_self_voxels(p, poses)
    kf = select_keyframes(stamps, times, poses, kf_step=a.kf_step)
    if a.limit:
        kf = kf[:a.limit]
    print(f"map={len(p)} poses={len(poses)} self_vox={len(self_vox)} keyframes={len(kf)}")

    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    n = 0
    for idx in kf:
        t_ns = stamps[idx]
        lab = build_label(p, times, poses, tpos, self_vox,
                          rig.cams_by_name, rig.T_cam_front, T_front_lidar, t_ns, spec,
                          z_gate=a.z_gate)
        imgs = bev_io.load_cam_images(a.extract_dir, idx, rig.idx_to_name, USE)
        if any(imgs[name] is None for name in USE):
            print(f"skip (missing image) frame_idx={idx}")
            continue
        canvas = ipm.ipm_canvas(imgs, rig.cams_by_name, rig.T_cam_front, T_front_lidar,
                                a.cam_height, spec, use_names=USE, blend=a.blend)
        sd = out / f"sample_{n:06d}"
        sd.mkdir(exist_ok=True)
        save_label_png(sd / "label.png", lab)
        cv2.imwrite(str(sd / "ipm_rgb.png"), canvas)
        cv2.imwrite(str(sd / "overlay.png"),
                    render.blend_label(canvas, lab, alpha=a.alpha))
        cv2.imwrite(str(sd / "review.png"),
                    render.review_overlay(canvas, lab, imgs, spec, alpha=a.alpha))
        for name in USE:
            cv2.imwrite(str(sd / f"cam_{name}.jpg"), imgs[name])
        T_wb = pose_at(times, poses, t_ns)
        meta = {
            "frame_idx": idx, "stamp_ns": t_ns,
            "world_T_body": T_wb.tolist(),
            "bev": {"XF": spec.XF, "XR": spec.XR, "YH": spec.YH, "RES": spec.RES,
                    "NX": spec.NX, "NY": spec.NY, "R_EGO": spec.R_EGO, "C_EGO": spec.C_EGO},
            "classes": {"0": "obstacle", "1": "drivable", "2": "ignore"},
            "cameras": list(USE),
            "calib": str(pathlib.Path(a.calib).resolve()),
            "orient": str(pathlib.Path(a.orient).resolve()),
            "params": {"z_gate": a.z_gate, "kf_step": a.kf_step, "cam_height": a.cam_height,
                       "blend": a.blend},
        }
        (sd / "meta.json").write_text(json.dumps(meta, indent=2))
        rows.append({"sample": f"sample_{n:06d}", "frame_idx": idx, "stamp_ns": t_ns})
        if n % 20 == 0:
            print(f"{n}/{len(kf)} {sd.name}")
        n += 1

    with open(out / "dataset.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["sample", "frame_idx", "stamp_ns"])
        w.writeheader()
        w.writerows(rows)
    print(f"done: {len(rows)} samples → {out}")


if __name__ == "__main__":
    main()
