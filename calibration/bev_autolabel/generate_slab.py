#!/usr/bin/env python3
"""map_clean.pcd → 키프레임별 BEV occupancy + visibility 라벨.

파이프라인: pose 로 body 프레임 3D crop → 하위 pct z 부터 thick 만큼의 슬래브 →
2D 기둥 누적 count>=min_pts 로 occupancy → ego 셀 2D 360° raycast ∧ 카메라
관측가능성으로 visibility. 설계 근거는
docs/superpowers/specs/2026-08-10-bev-slab-label-design.md

사용:
  cd calibration/bev_autolabel
  python3 generate_slab.py \
    --map-dir ../../data/sj_bags/260722/maps_selfmask/raws3_mapping \
    --extract-dir ../../data/extracted/raws3 \
    --calib ../../data/calib_260723/calib.yaml \
    --orient ../../data/calib_260723/orientation.json \
    --self-mask-dir ../../data/calib_260723/self_mask \
    --out ../../data/bev/slab/raws3
"""
import argparse
import csv
import json
import pathlib
import sys

import numpy as np
import cv2

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
from calib_io import load_T_front_lidar                    # noqa: E402
from chain import se3_inv, transform                       # noqa: E402
from cloud_io import pose_at                                # noqa: E402
from ds_model import load_rig                               # noqa: E402
import bev_io                                              # noqa: E402
from bev_label import BevSpec, raycast_visible, select_keyframes  # noqa: E402
import slab_label as sl                                    # noqa: E402
import slab_io                                             # noqa: E402
import slab_render as sr                                   # noqa: E402
import ipm as ipm_mod                                      # noqa: E402

USE = ("front", "left", "right")


def build_invalid_masks(extract_dir, rig, self_mask_dir, classes=("table",),
                        use_names=USE):
    """{name: (H,W) bool True=무효} = 어안 원 바깥(자동) ∪ self 마스크(수작업).

    self 마스크가 없으면 어안 원만 쓰고 경고한다 — 그 경우 카트 상판이 가린 근거리가
    '보인다'고 나와 visibility 가 낙관적이다. 손잡이·수집자는 여기서 다루지 않는다
    (slab_label.self_box_mask 가 body 프레임에서 덮는다).
    """
    name2idx = {v: k for k, v in rig.idx_to_name.items()}
    self_masks = (slab_io.load_self_masks(self_mask_dir, use_names, classes)
                  if self_mask_dir else {})
    if not self_masks:
        print("[경고] self 마스크가 없습니다 — 카트 상판이 가린 근거리 visibility 가 "
              "낙관적으로 나옵니다.")
    out = {}
    for name in use_names:
        frames = slab_io.sample_frames_gray(extract_dir, name2idx[name])
        m = sl.vignette_mask(frames)
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN,
                             np.ones((5, 5), np.uint8)).astype(bool)
        sm = self_masks.get(name)
        if sm is not None:
            if sm.shape != m.shape:
                sys.exit(f"self 마스크 크기 불일치 {name}: {sm.shape} != {m.shape}")
            m |= sm
        out[name] = m
        print(f"  invalid[{name}]: 무효 화소 {m.mean()*100:.1f}% "
              f"(self 마스크 {'있음' if sm is not None else '없음'})")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-dir", required=True)
    ap.add_argument("--extract-dir", required=True)
    ap.add_argument("--calib", required=True)
    ap.add_argument("--orient", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--self-mask-dir", default=None)
    ap.add_argument("--xf", type=float, default=4.0)
    ap.add_argument("--xr", type=float, default=2.0)
    ap.add_argument("--yh", type=float, default=3.0)
    ap.add_argument("--thick", type=float, default=0.8,
                    help="슬래브 두께[m]. 로봇이 통과해야 하는 높이 구간")
    ap.add_argument("--pct", type=float, default=1.0,
                    help="z_ref 퍼센타일. 5 는 슬래브 바닥을 0.25m 들어올린다")
    ap.add_argument("--min-pts", type=int, default=3)
    ap.add_argument("--ray-step", type=float, default=0.25,
                    help="raycast_visible 광선 간격[도], 기본 0.25(bev_label 기본 0.5의 절반이라 "
                         "레이캐스트 비용 2배). 작을수록 느려진다")
    ap.add_argument("--ground-offset", type=float, default=0.87,
                    help="z_ref 아래 실제 지면까지의 거리[m]. IPM cam_height 와 같은 평면")
    ap.add_argument("--self-mask-classes", default="table",
                    help="이미지 마스크에서 무효로 읽을 클래스(쉼표 구분). handle·human 은 "
                         "이미지 위치가 프레임마다 달라 self 박스로 덮는다(설계 §2.8.1)")
    ap.add_argument("--self-box-near", type=float, default=0.4,
                    help="후방 self 박스 근단[m]. 박스 = -far <= x <= -near, |y| <= yh")
    ap.add_argument("--self-box-far", type=float, default=2.1)
    ap.add_argument("--self-box-yh", type=float, default=0.7)
    ap.add_argument("--kf-step", type=float, default=0.4)
    ap.add_argument("--save-crop", action="store_true")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--review-scale", type=int, default=6)
    ap.add_argument("--cam-height", type=float, default=0.87,
                    help="IPM 지면 평면용 카메라 렌즈 높이[m] 실측값. IPM 정확도의 핵심")
    ap.add_argument("--blend", default="nearest", choices=("nearest", "average"),
                    help="IPM 다중카메라 합성 방식")
    ap.add_argument("--alpha", type=float, default=0.30,
                    help="overlay.png/review.png 의 obstacle 오버레이 불투명도(IPM 우선)")
    ap.add_argument("--no-ipm", action="store_true",
                    help="IPM 생성을 끈다(LiDAR 라벨만 빠르게 뽑을 때)")
    a = ap.parse_args()

    spec = BevSpec(XF=a.xf, XR=a.xr, YH=a.yh)
    print(f"BEV {spec.NX}x{spec.NY} (XF={spec.XF} XR={spec.XR} YH={spec.YH} "
          f"RES={spec.RES}) 슬래브 {a.thick}m @ p{a.pct}")

    rig = load_rig(a.calib, a.orient)
    T_front_lidar = load_T_front_lidar(a.calib)
    if T_front_lidar is None:
        sys.exit("calib.yaml 에 extrinsics.T_front_lidar 가 없습니다")
    classes = tuple(c.strip() for c in a.self_mask_classes.split(",") if c.strip())
    invalid = build_invalid_masks(a.extract_dir, rig, a.self_mask_dir, classes)

    head, arr, xyz, times_ns, poses = slab_io.load_map(a.map_dir)
    stamps = bev_io.load_stamps(a.extract_dir)
    kf = select_keyframes(stamps, times_ns, poses, kf_step=a.kf_step)
    if a.limit:
        kf = kf[:a.limit]
    print(f"map={len(xyz)} poses={len(poses)} keyframes={len(kf)}")

    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    # world bbox 사전 필터 반경: body 프레임 crop 의 가장 먼 모서리까지의 거리.
    # max(XF,XR,YH) 로는 부족하다 — 회전에 따라 (XF,YH) 모서리가 world 축에 정렬되면
    # hypot(4,3)=5.0m 가 필요한데 max()*1.5=4.5m 는 모서리를 잘라낸다.
    reach = float(np.hypot(max(spec.XF, spec.XR), spec.YH)) + spec.RES
    # 박스는 샘플과 무관하므로 루프 밖에서 한 번만 만든다.
    self_box = sl.self_box_mask(spec, a.self_box_near, a.self_box_far, a.self_box_yh)
    print(f"self 박스: x -{a.self_box_far}~-{a.self_box_near} |y|<={a.self_box_yh} "
          f"→ {int(self_box.sum())}셀 ({self_box.mean()*100:.1f}%)")
    rows = []
    for n, idx in enumerate(kf):
        t_ns = stamps[idx]
        imgs = bev_io.load_cam_images(a.extract_dir, idx, rig.idx_to_name, USE)
        if any(imgs[name] is None for name in USE):
            print(f"skip (missing image) frame_idx={idx}")
            continue
        T_wb = pose_at(times_ns, poses, t_ns)
        ctr = T_wb[:3, 3]
        near = np.flatnonzero((np.abs(xyz[:, 0] - ctr[0]) < reach)
                              & (np.abs(xyz[:, 1] - ctr[1]) < reach))
        P = transform(se3_inv(T_wb), xyz[near])
        cm = sl.crop_mask(P, spec)
        crop_idx, crop_P = near[cm], P[cm]
        if len(crop_P) == 0:
            print(f"skip (crop 비어 있음) frame_idx={idx}")
            continue
        z_ref = sl.ref_z(crop_P[:, 2], pct=a.pct)
        sm = sl.slab_mask(crop_P, z_ref, thick=a.thick)
        slab_idx, slab_P = crop_idx[sm], crop_P[sm]

        counts = sl.occupancy_counts(slab_P, spec)
        obstacle = sl.obstacle_from_counts(counts, min_pts=a.min_pts)
        visible = raycast_visible(obstacle, spec, step_deg=a.ray_step)
        cam_ok = sl.camera_observable(spec, z_ref - a.ground_offset,
                                      rig.cams_by_name, rig.T_cam_front,
                                      T_front_lidar, invalid, use_names=USE)
        # self 박스는 cam_ok 와 따로 둔다 — stats 의 camera_ok_pct 가 '기하+이미지 마스크
        # 커버리지' 진단값으로 남아야 박스가 그 수치를 가리지 않는다.
        occupancy, visibility = sl.assemble(obstacle, visible, cam_ok & ~self_box)

        ipm_rgb = None if a.no_ipm else ipm_mod.ipm_canvas(
            imgs, rig.cams_by_name, rig.T_cam_front, T_front_lidar, a.cam_height,
            spec, use_names=USE, blend=a.blend)

        sd = out / f"sample_{n:06d}"
        sd.mkdir(exist_ok=True)
        for name in USE:
            cv2.imwrite(str(sd / f"cam_{name}.jpg"), imgs[name])
        slab_io.write_points(sd / "slab.pcd", head, arr, slab_idx, slab_P)
        if a.save_crop:
            slab_io.write_points(sd / "crop.pcd", head, arr, crop_idx, crop_P)
        sr.save_indexed(sd / "occupancy.png", occupancy, sr.PALETTE_OCC)
        sr.save_indexed(sd / "visibility.png", visibility, sr.PALETTE_VIS)
        if ipm_rgb is not None:
            cv2.imwrite(str(sd / "ipm_rgb.png"), ipm_rgb)
            cv2.imwrite(str(sd / "overlay.png"),
                        sr.blend_slab(ipm_rgb, occupancy, alpha=a.alpha))
        cv2.imwrite(str(sd / "review.png"),
                    sr.review_png(occupancy, visibility, spec, scale=a.review_scale,
                                  cam_imgs=imgs, ipm=ipm_rgb, alpha=a.alpha))
        stats = {"crop_pts": int(len(crop_P)), "slab_pts": int(len(slab_P)),
                 "obstacle_pct": float(obstacle.mean() * 100),
                 "visible_pct": float((visibility == 1).mean() * 100),
                 "camera_ok_pct": float(cam_ok.mean() * 100)}
        meta = {
            "frame_idx": idx, "stamp_ns": t_ns,
            "world_T_body": T_wb.tolist(),
            "bev": {"XF": spec.XF, "XR": spec.XR, "YH": spec.YH, "RES": spec.RES,
                    "NX": spec.NX, "NY": spec.NY,
                    "R_EGO": spec.R_EGO, "C_EGO": spec.C_EGO},
            "z_ref": z_ref,
            "classes": {"occupancy": {"0": "obstacle", "1": "drivable"},
                        "visibility": {"0": "unseen", "1": "visible"}},
            "cameras": list(USE),
            "calib": str(pathlib.Path(a.calib).resolve()),
            "orient": str(pathlib.Path(a.orient).resolve()),
            "self_mask": (str(pathlib.Path(a.self_mask_dir).resolve())
                          if a.self_mask_dir else None),
            "self_mask_classes": list(classes),
            "self_box": {"near": a.self_box_near, "far": a.self_box_far,
                         "yh": a.self_box_yh, "cells": int(self_box.sum())},
            "params": {"thick": a.thick, "pct": a.pct, "min_pts": a.min_pts,
                       "ray_step": a.ray_step, "ground_offset": a.ground_offset,
                       "kf_step": a.kf_step, "cam_height": a.cam_height,
                       "blend": a.blend, "alpha": a.alpha},
            "stats": stats,
        }
        (sd / "meta.json").write_text(json.dumps(meta, indent=2))
        rows.append({"sample": sd.name, "frame_idx": idx, "stamp_ns": t_ns,
                     "z_ref": f"{z_ref:.4f}", **{k: f"{v:.2f}" if isinstance(v, float)
                                                 else v for k, v in stats.items()}})
        if n % 20 == 0:
            print(f"{n}/{len(kf)} {sd.name} z_ref={z_ref:+.3f} "
                  f"slab={len(slab_P)} obs={stats['obstacle_pct']:.1f}% "
                  f"vis={stats['visible_pct']:.1f}%")

    if not rows:
        sys.exit("샘플이 하나도 만들어지지 않았습니다")
    with open(out / "dataset.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    zr = np.array([float(r["z_ref"]) for r in rows])
    ob = np.array([float(r["obstacle_pct"]) for r in rows])
    vi = np.array([float(r["visible_pct"]) for r in rows])
    print(f"done: {len(rows)} samples → {out}")
    print(f"  z_ref {zr.min():+.3f}~{zr.max():+.3f} (중앙 {np.median(zr):+.3f})")
    print(f"  obstacle {ob.min():.1f}~{ob.max():.1f}% (중앙 {np.median(ob):.1f}%)")
    print(f"  visible  {vi.min():.1f}~{vi.max():.1f}% (중앙 {np.median(vi):.1f}%)")


if __name__ == "__main__":
    main()
