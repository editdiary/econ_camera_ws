"""단계3 — 이미지 마스크 IPM 투영 + LiDAR auto-label 융합 검수 뷰.

워크플로우: (1) generate.py 로 데이터셋(sample_NNNNNN) 생성 → (2) 사람이 각 sample 의
cam_{front,left,right}.jpg 에 drivable 마스크를 그림 → (3) 이 도구가 각 sample 의 meta.json
(stamp·z_gate)으로 LiDAR 라벨을 재구성하고, 마스크를 바닥 평면(H)에 IPM 투영해 겹친
검수 3-패널 + 80x80 카테고리 맵을 만든다.

마스크 입력 레이아웃(--mask-dir 아래, 데이터셋 sample 구조를 미러링):
    <mask-dir>/sample_NNNNNN/cam_{front,left,right}.png   (흰색=drivable)

IPM 은 "카메라 렌즈의 바닥 위 높이 H(--cam-height, 기본 0.87m)" 를 외부 입력으로 받는다
(마스트 LiDAR 는 바닥을 못 봐 H 를 못 주기 때문 — 실측값 필요).
"""
import argparse
import glob
import json
import os
import sys

BA = os.path.dirname(os.path.abspath(__file__))
for _p in (BA, os.path.join(BA, "..", "cam_lidar"), os.path.join(BA, "..", "verify")):
    sys.path.insert(0, os.path.abspath(_p))

import numpy as np                          # noqa: E402
import cv2                                  # noqa: E402

import bev_io                               # noqa: E402
from bev_label import (BevSpec, build_label, compute_self_voxels,  # noqa: E402
                       keep_ego_connected)
from ds_model import load_rig               # noqa: E402
from calib_io import load_T_front_lidar     # noqa: E402
from chain import se3_inv                   # noqa: E402

USE = ("front", "left", "right")


# ---------------------------------------------------------------- 순수 로직
def ipm_project_mask(mask, cam, T_cam_lidar, cam_height, spec):
    """이미지 drivable 마스크(H,W>0) → 바닥 평면 z=-H 투영 → BEV bool 격자(NX,NY).

    T_cam_lidar: ego(LiDAR)→cam (4x4). 바닥은 카메라 아래 cam_height 만큼(z=위 가정).
    """
    T_lidar_cam = se3_inv(T_cam_lidar)
    C = T_lidar_cam[:3, 3]
    R = T_lidar_cam[:3, :3]
    z0 = C[2] - cam_height
    vs, us = np.where(mask > 0)
    grid = np.zeros((spec.NX, spec.NY), bool)
    if len(us) == 0:
        return grid
    dirs, valid = cam.unproject(us.astype(np.float64), vs.astype(np.float64))
    d = dirs @ R.T                                   # 광선을 ego 프레임으로 회전
    dz = d[:, 2]
    ok = valid & (dz < -1e-6)                         # 아래로 향하는 광선만 바닥과 교차
    t = np.where(ok, (z0 - C[2]) / np.where(dz == 0, 1.0, dz), -1.0)
    ok &= t > 0
    X = C[0] + t * d[:, 0]
    Y = C[1] + t * d[:, 1]
    ok &= (X <= spec.XF) & (X >= -spec.XR) & (np.abs(Y) <= spec.YH)
    row = np.round((spec.XF - X[ok]) / spec.RES).astype(int)
    col = np.round((spec.YH - Y[ok]) / spec.RES).astype(int)
    ib = (row >= 0) & (row < spec.NX) & (col >= 0) & (col < spec.NY)
    grid[row[ib], col[ib]] = True
    return grid


def _cell_dist(spec):
    r = np.arange(spec.NX)[:, None]
    c = np.arange(spec.NY)[None, :]
    return np.hypot(spec.XF - r * spec.RES, spec.YH - c * spec.RES)


def fuse_labels(lidar_label, obs_rc, ipm_floor, spec, near_m=2.0):
    """LiDAR 라벨 + 물리 장애물(obs_rc) + 이미지 IPM 바닥 → (fused 0/1/2, agree BGR).

    규칙: 합의는 자동 확정, 주황(이미지바닥∩장애물)=obstacle(LiDAR 채택),
    근거리 하늘(이미지바닥∩LiDAR미확정)=drivable, 원거리 하늘=ignore, ego 연결성 정리.
    """
    Ld = lidar_label == 1                              # LiDAR 확정 drivable
    fused = np.full((spec.NX, spec.NY), 2, np.uint8)
    fused[Ld] = 1
    fused[obs_rc] = 0                                   # 물리 장애물 우선(주황 포함)
    cand = ipm_floor & ~Ld & ~obs_rc                    # 이미지가 채운 미확정 바닥
    near = _cell_dist(spec) <= near_m
    fused[cand & near] = 1
    driv = keep_ego_connected(fused == 1, spec)
    fused[(fused == 1) & ~driv] = 2

    agree = np.full((spec.NX, spec.NY, 3), 60, np.uint8)
    agree[obs_rc & ~ipm_floor] = (0, 0, 200)            # 확정 장애물(빨강)
    agree[Ld & ~ipm_floor] = (0, 120, 0)                # LiDAR만 drivable(어두운 초록)
    agree[Ld & ipm_floor] = (0, 255, 0)                 # 둘 다 drivable(밝은 초록)
    agree[cand] = (255, 255, 0)                         # 후보 바닥(하늘)
    agree[obs_rc & ipm_floor] = (0, 140, 255)           # 검토: 이미지바닥∩장애물(주황)
    return fused, agree


# ---------------------------------------------------------------- 렌더
def _draw_bev(rgb, title, spec, U=6):
    img = cv2.resize(rgb, (spec.NY * U, spec.NX * U), interpolation=cv2.INTER_NEAREST)
    for mx in range(0, spec.NY + 1, 10):
        cv2.line(img, (mx * U, 0), (mx * U, spec.NX * U), (80, 80, 80), 1)
    for my in range(0, spec.NX + 1, 10):
        cv2.line(img, (0, my * U), (spec.NY * U, my * U), (80, 80, 80), 1)
        cv2.putText(img, f"{spec.XF - my * spec.RES:+.0f}m", (2, my * U + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 0), 1)
    r, c = spec.R_EGO, spec.C_EGO
    cv2.rectangle(img, ((c - 4) * U, (r - 4) * U), ((c + 4) * U, (r + 4) * U), (255, 255, 255), 1)
    cv2.arrowedLine(img, (c * U, r * U), (c * U, (r - 8) * U), (255, 255, 255), 2, tipLength=0.3)
    bar = np.zeros((24, img.shape[1], 3), np.uint8)
    cv2.putText(bar, title, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    return np.vstack([bar, img])


def _colorize_label(label, spec):
    rgb = np.full((spec.NX, spec.NY, 3), 128, np.uint8)
    rgb[label == 0] = (0, 0, 255)
    rgb[label == 1] = (0, 200, 0)
    return rgb


def render_review(cam_imgs, masks, lidar_label, obs_rc, ipm_floor, agree, spec, cam_height):
    W = spec.NY * 6
    # 상단: 카메라 이미지 + 그린 마스크 오버레이
    tops = []
    for name in USE:
        im = cam_imgs.get(name)
        if im is None:
            im = np.zeros((720, 1280, 3), np.uint8)
        else:
            im = im.copy()
            m = masks.get(name)
            if m is not None:
                ov = im.copy(); ov[m > 0] = (0, 255, 0)
                im = cv2.addWeighted(ov, 0.35, im, 0.65, 0)
        im = cv2.resize(im, (W, int(W * im.shape[0] / im.shape[1])))
        cv2.putText(im, f"{name} (painted drivable)", (6, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        tops.append(im)
    top = np.hstack(tops)
    # 하단: LiDAR / IPM / COMBINED
    lidar_rgb = _colorize_label(lidar_label, spec); lidar_rgb[obs_rc] = (0, 0, 255)
    ipm_rgb = np.full((spec.NX, spec.NY, 3), 60, np.uint8); ipm_rgb[ipm_floor] = (255, 255, 0)
    bottom = np.hstack([
        _draw_bev(lidar_rgb, "1) LiDAR auto-label", spec),
        _draw_bev(ipm_rgb, f"2) image IPM floor @H={cam_height:.2f}m", spec),
        _draw_bev(agree, "3) COMBINED (grn=agree red=obs org=review cyan=cand)", spec),
    ])
    if top.shape[1] != bottom.shape[1]:
        top = cv2.resize(top, (bottom.shape[1], top.shape[0]))
    return np.vstack([top, bottom])


# ---------------------------------------------------------------- CLI
def _samples(dataset_dir, mask_dir):
    """마스크 폴더가 존재하는 sample 목록 [(name, sample_dir, mask_dir)]."""
    out = []
    for d in sorted(glob.glob(os.path.join(dataset_dir, "sample_*"))):
        name = os.path.basename(d)
        md = os.path.join(mask_dir, name)
        if os.path.isdir(d) and os.path.isdir(md):
            out.append((name, d, md))
    return out


def main():
    ap = argparse.ArgumentParser(
        description="생성된 BEV 데이터셋 sample 에 이미지 마스크를 IPM 투영 + LiDAR 융합 검수")
    ap.add_argument("--dataset-dir", required=True, help="generate.py 산출(sample_NNNNNN + meta.json)")
    ap.add_argument("--map-dir", required=True, help="LiDAR 라벨 재구성용 LIO 맵")
    ap.add_argument("--mask-dir", required=True,
                    help="sample_NNNNNN/cam_{front,left,right}.png 루트 (흰=drivable)")
    ap.add_argument("--out", default=None, help="미지정 시 각 sample 폴더 안에 기록")
    ap.add_argument("--calib", default=None, help="미지정 시 sample meta.json 의 calib 경로 사용")
    ap.add_argument("--orient", default=None, help="미지정 시 sample meta.json 의 orient 경로 사용")
    ap.add_argument("--cam-height", type=float, default=0.87, help="카메라 렌즈의 바닥 위 높이[m]")
    ap.add_argument("--near", type=float, default=2.0, help="하늘(후보) 바닥을 drivable로 승격하는 반경[m]")
    ap.add_argument("--z-gate", type=float, default=None, help="미지정 시 sample meta 의 params.z_gate 사용")
    args = ap.parse_args()

    samples = _samples(args.dataset_dir, args.mask_dir)
    if not samples:
        sys.exit(f"[ERR] 마스크 폴더가 있는 sample 없음 "
                 f"(dataset={args.dataset_dir}, mask={args.mask_dir})")

    first_meta = json.load(open(os.path.join(samples[0][1], "meta.json")))
    calib = args.calib or first_meta["calib"]
    orient = args.orient or first_meta["orient"]
    rig = load_rig(calib, orient)
    Tfl = load_T_front_lidar(calib)
    if Tfl is None:
        sys.exit(f"[ERR] {calib} 에 T_front_lidar 없음 — Cam-LiDAR 캘리브 선행 필요.")

    p, times, poses, tpos = bev_io.load_map(args.map_dir)
    self_vox = compute_self_voxels(p, poses)
    spec = BevSpec()

    n_ok = 0
    for name, sdir, mdir in samples:
        meta = json.load(open(os.path.join(sdir, "meta.json")))
        t_ns = int(meta["stamp_ns"])
        z_gate = args.z_gate if args.z_gate is not None else meta.get("params", {}).get("z_gate", 0.3)
        masks = {}
        for cam in USE:
            mp = os.path.join(mdir, f"cam_{cam}.png")
            if os.path.exists(mp):
                masks[cam] = cv2.imread(mp, cv2.IMREAD_GRAYSCALE)
        if not masks:
            print(f"[skip] {name}: cam_*.png 마스크 없음 ({mdir})"); continue

        label, parts = build_label(p, times, poses, tpos, self_vox, rig.cams_by_name,
                                   rig.T_cam_front, Tfl, t_ns, spec,
                                   use_names=USE, z_gate=z_gate, return_parts=True)
        obs_rc = parts["obs_rc"]

        ipm = np.zeros((spec.NX, spec.NY), bool)
        for cam, m in masks.items():
            T_cam_lidar = rig.T_cam_front[cam] @ Tfl
            ipm |= ipm_project_mask(m, rig.cams_by_name[cam], T_cam_lidar, args.cam_height, spec)

        fused, agree = fuse_labels(label, obs_rc, ipm, spec, near_m=args.near)

        cam_imgs = {}
        for cam in USE:
            ip = os.path.join(sdir, f"cam_{cam}.jpg")
            if os.path.exists(ip):
                cam_imgs[cam] = cv2.imread(ip)
        review = render_review(cam_imgs, masks, label, obs_rc, ipm, agree, spec, args.cam_height)

        odir = sdir if args.out is None else os.path.join(args.out, name)
        os.makedirs(odir, exist_ok=True)
        cv2.imwrite(os.path.join(odir, "review_combined.png"), review)
        # label_fused.png = 장식 없는 80x80 다색 카테고리 맵(label tool에 바로 로드). 색 의미는 review_combined 범례.
        cv2.imwrite(os.path.join(odir, "label_fused.png"), agree)
        Ld = label == 1
        rmeta = {"sample": name, "frame_idx": meta.get("frame_idx"), "stamp_ns": t_ns,
                 "cam_height_m": args.cam_height, "z_gate": z_gate, "near_m": args.near,
                 "cams_masked": list(masks.keys()),
                 "category_cells": {
                     "both_drivable": int((Ld & ipm).sum()),
                     "lidar_only_drivable": int((Ld & ~ipm).sum()),
                     "lidar_obstacle": int((obs_rc & ~ipm).sum()),
                     "image_floor_on_obstacle_review": int((obs_rc & ipm).sum()),
                     "image_candidate_floor": int((ipm & ~Ld & ~obs_rc).sum())},
                 "fused_counts": {"obstacle": int((fused == 0).sum()),
                                  "drivable": int((fused == 1).sum()), "ignore": int((fused == 2).sum())}}
        json.dump(rmeta, open(os.path.join(odir, "meta_review.json"), "w"), ensure_ascii=False, indent=2)
        print(f"[ok] {name} (frame {meta.get('frame_idx')}): cams={list(masks.keys())} "
              f"ipm={int(ipm.sum())} -> {odir}")
        n_ok += 1
    print(f"done: {n_ok}/{len(samples)} samples")


if __name__ == "__main__":
    main()
