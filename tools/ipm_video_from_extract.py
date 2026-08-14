#!/usr/bin/env python3
"""Extracted synchronized camera frames -> IPM BEV MP4.

This is intentionally bag-adjacent rather than a general dataset generator:
it consumes the existing ``bag_extract.py`` output layout and writes one BEV
review video from front/left/right camera frames.
"""
from __future__ import annotations

import argparse
import csv
import pathlib
import sys
from dataclasses import dataclass

import cv2
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
CAL = REPO / "calibration"
for p in (CAL / "cam_lidar", CAL / "verify", CAL / "bev_autolabel"):
    sys.path.insert(0, str(p))

from calib_io import load_T_front_lidar  # noqa: E402
from chain import se3_inv  # noqa: E402
from ds_model import load_rig  # noqa: E402
from bev_label import BevSpec  # noqa: E402

USE = ("front", "left", "right")


@dataclass(frozen=True)
class ExtractedFrame:
    idx: int
    stamp: float
    imgs: dict[str, np.ndarray]


@dataclass(frozen=True)
class ProjectionMap:
    src_flat: np.ndarray
    cell_flat: np.ndarray
    cnt: np.ndarray
    dist: np.ndarray


def iter_extracted_frames(
    extract_dir: str | pathlib.Path,
    idx_to_name: dict[int, str],
    use_names: tuple[str, ...] = USE,
):
    """Yield synchronized frames from ``bag_extract.py`` output.

    Missing required camera JPEGs cause that frame to be skipped. The mapping
    from ``camN.jpg`` to semantic names comes from orientation.json.
    """
    extract_dir = pathlib.Path(extract_dir)
    with (extract_dir / "sets.csv").open(newline="") as f:
        for row in csv.DictReader(f):
            idx = int(row["idx"])
            frame_dir = extract_dir / f"frame_{idx:06d}"
            imgs: dict[str, np.ndarray] = {}
            ok = True
            for cam_idx, name in idx_to_name.items():
                if name not in use_names:
                    continue
                img = cv2.imread(str(frame_dir / f"cam{cam_idx}.jpg"), cv2.IMREAD_COLOR)
                if img is None:
                    ok = False
                    break
                imgs[name] = img
            if ok and all(name in imgs for name in use_names):
                yield ExtractedFrame(idx=idx, stamp=float(row["stamp0"]), imgs=imgs)


def build_projection_map(img_shape, cam, T_cam_lidar, cam_height, spec, pixel_step=1):
    """Precompute pixel -> BEV-cell mapping for one camera."""
    T_lidar_cam = se3_inv(T_cam_lidar)
    C = T_lidar_cam[:3, 3]
    R = T_lidar_cam[:3, :3]
    z0 = C[2] - cam_height
    height, width = img_shape[:2]
    vs, us = np.mgrid[0:height:pixel_step, 0:width:pixel_step]
    us = us.ravel().astype(np.float64)
    vs = vs.ravel().astype(np.float64)
    src_flat = (vs.astype(np.int64) * width + us.astype(np.int64))

    dirs, valid = cam.unproject(us, vs)
    d = dirs @ R.T
    dz = d[:, 2]
    ok = valid & (dz < -1e-6)
    t = np.where(ok, (z0 - C[2]) / np.where(dz == 0, 1.0, dz), -1.0)
    ok &= t > 0
    X = C[0] + t * d[:, 0]
    Y = C[1] + t * d[:, 1]
    ok &= (X <= spec.XF) & (X >= -spec.XR) & (np.abs(Y) <= spec.YH)
    row = np.round((spec.XF - X) / spec.RES).astype(np.int64)
    col = np.round((spec.YH - Y) / spec.RES).astype(np.int64)
    ok &= (row >= 0) & (row < spec.NX) & (col >= 0) & (col < spec.NY)

    cell_flat = row[ok] * spec.NY + col[ok]
    ncell = spec.NX * spec.NY
    cnt = np.bincount(cell_flat, minlength=ncell).astype(np.int32)
    dist = np.full(ncell, np.inf, np.float64)
    cell_x = spec.XF - (np.arange(ncell) // spec.NY) * spec.RES
    cell_y = spec.YH - (np.arange(ncell) % spec.NY) * spec.RES
    covered = cnt > 0
    dist[covered] = np.sqrt(
        (cell_x[covered] - C[0]) ** 2
        + (cell_y[covered] - C[1]) ** 2
        + cam_height ** 2
    )
    return ProjectionMap(src_flat=src_flat[ok], cell_flat=cell_flat, cnt=cnt, dist=dist)


def render_ipm(imgs, projection_maps, spec, blend="nearest"):
    ncell = spec.NX * spec.NY
    colors = []
    dists = []
    for name, pmap in projection_maps.items():
        pix = imgs[name].reshape(-1, 3)[pmap.src_flat].astype(np.float64)
        out = np.zeros((ncell, 3), np.uint8)
        covered = pmap.cnt > 0
        for ch in range(3):
            sums = np.bincount(pmap.cell_flat, weights=pix[:, ch], minlength=ncell)
            out[covered, ch] = np.clip(sums[covered] / pmap.cnt[covered], 0, 255).astype(np.uint8)
        colors.append(out.reshape(spec.NX, spec.NY, 3))
        dists.append(pmap.dist.reshape(spec.NX, spec.NY))

    if blend == "average":
        acc = np.zeros((spec.NX, spec.NY, 3), np.float64)
        cnt = np.zeros((spec.NX, spec.NY), np.int32)
        for color, dist in zip(colors, dists):
            m = np.isfinite(dist)
            acc[m] += color[m]
            cnt[m] += 1
        bev = np.zeros((spec.NX, spec.NY, 3), np.uint8)
        m = cnt > 0
        bev[m] = np.clip(acc[m] / cnt[m, None], 0, 255).astype(np.uint8)
        return bev

    if blend != "nearest":
        raise ValueError(f"unknown blend: {blend!r}")
    stack = np.stack(dists, axis=0)
    best = np.argmin(stack, axis=0)
    has = np.isfinite(stack).any(axis=0)
    bev = np.zeros((spec.NX, spec.NY, 3), np.uint8)
    for i, color in enumerate(colors):
        m = has & (best == i)
        bev[m] = color[m]
    return bev


def draw_bev_review(bev, spec, scale=6, label=""):
    """Scale BEV and add meter grid, ego marker, and optional frame label."""
    out = cv2.resize(bev, (spec.NY * scale, spec.NX * scale), interpolation=cv2.INTER_NEAREST)
    height, width = out.shape[:2]
    for x_m in np.arange(np.floor(spec.XF / 0.5) * 0.5, -spec.XR - 1e-9, -0.5):
        y_px = int(round((spec.XF - x_m) / spec.RES)) * scale
        cv2.line(out, (0, y_px), (width, y_px), (60, 60, 60), 1)
    for y_m in np.arange(np.floor(spec.YH / 0.5) * 0.5, -spec.YH - 1e-9, -0.5):
        x_px = int(round((spec.YH - y_m) / spec.RES)) * scale
        cv2.line(out, (x_px, 0), (x_px, height), (60, 60, 60), 1)
    ego_x, ego_y = spec.C_EGO * scale, spec.R_EGO * scale
    cv2.circle(out, (ego_x, ego_y), max(2, scale // 2), (255, 255, 255), -1)
    cv2.arrowedLine(out, (ego_x, ego_y), (ego_x, max(0, ego_y - 5 * scale)),
                    (255, 255, 255), 2, tipLength=0.3)
    if label:
        cv2.putText(out, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2, cv2.LINE_AA)
    return out


def _camera_strip(cam_imgs, width, order=("left", "front", "right")):
    names = [name for name in order if cam_imgs.get(name) is not None]
    if not names:
        return None
    cell_w = width // len(names)
    row = []
    for name in names:
        img = cam_imgs[name]
        resized = cv2.resize(img, (cell_w, int(round(cell_w * img.shape[0] / img.shape[1]))))
        cv2.putText(resized, name, (6, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2, cv2.LINE_AA)
        row.append(resized)
    out = np.hstack(row)
    if out.shape[1] != width:
        out = cv2.resize(out, (width, out.shape[0]))
    return out


def compose_video_frame(bev_review, cam_imgs, width, camera_strip="top"):
    """Compose final video frame from BEV review and optional raw camera strip."""
    bev = cv2.resize(bev_review, (width, int(round(width * bev_review.shape[0] / bev_review.shape[1]))),
                     interpolation=cv2.INTER_NEAREST)
    if camera_strip == "none":
        out = bev
        pad_h = out.shape[0] % 2
        pad_w = out.shape[1] % 2
        return cv2.copyMakeBorder(out, 0, pad_h, 0, pad_w, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    if camera_strip != "top":
        raise ValueError(f"unknown camera_strip: {camera_strip!r}")
    strip = _camera_strip(cam_imgs, width)
    out = bev if strip is None else np.vstack([strip, bev])
    pad_h = out.shape[0] % 2
    pad_w = out.shape[1] % 2
    return cv2.copyMakeBorder(out, 0, pad_h, 0, pad_w, cv2.BORDER_CONSTANT, value=(0, 0, 0))


def parse_args(argv=None):
    ap = argparse.ArgumentParser()
    d = REPO / "data"
    ap.add_argument("--extract-dir", default=str(d / "extracted/raws1"))
    ap.add_argument("--calib", default=str(d / "calib_260723/calib.yaml"))
    ap.add_argument("--orient", default=str(d / "calib_260723/orientation.json"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--scale", type=int, default=6)
    ap.add_argument("--xf", type=float, default=4.0)
    ap.add_argument("--xr", type=float, default=2.0)
    ap.add_argument("--yh", type=float, default=3.0)
    ap.add_argument("--cam-height", type=float, default=0.87)
    ap.add_argument("--blend", choices=("nearest", "average"), default="nearest")
    ap.add_argument("--pixel-step", type=int, default=1,
                    help="Use every Nth source pixel for speed; 1 is full resolution.")
    ap.add_argument("--camera-strip", choices=("top", "none"), default="top",
                    help="Add raw left/front/right images above the BEV, or use 'none'.")
    ap.add_argument("--limit", type=int, default=0)
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    spec = BevSpec(XF=args.xf, XR=args.xr, YH=args.yh)
    rig = load_rig(args.calib, args.orient)
    T_front_lidar = load_T_front_lidar(args.calib)
    if T_front_lidar is None:
        raise SystemExit(f"{args.calib} has no extrinsics.T_front_lidar")

    frame_iter = iter_extracted_frames(args.extract_dir, rig.idx_to_name, USE)
    try:
        first = next(frame_iter)
    except StopIteration as exc:
        raise SystemExit(f"no complete front/left/right frames in {args.extract_dir}") from exc

    projection_maps = {
        name: build_projection_map(
            first.imgs[name].shape,
            rig.cams_by_name[name],
            rig.T_cam_front[name] @ T_front_lidar,
            args.cam_height,
            spec,
            pixel_step=args.pixel_step,
        )
        for name in USE
    }
    print(f"BEV {spec.NX}x{spec.NY}, scale={args.scale}, blend={args.blend}, "
          f"pixel_step={args.pixel_step}")
    for name, pmap in projection_maps.items():
        print(f"  {name}: covered_cells={int((pmap.cnt > 0).sum())}")

    out_path = pathlib.Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    first_bev = render_ipm(first.imgs, projection_maps, spec, blend=args.blend)
    first_review = draw_bev_review(first_bev, spec, scale=args.scale,
                                   label=f"frame {first.idx}  t={first.stamp:.3f}")
    first_out = compose_video_frame(first_review, first.imgs, spec.NY * args.scale,
                                    camera_strip=args.camera_strip)
    size = (first_out.shape[1], first_out.shape[0])
    writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, size)
    if not writer.isOpened():
        raise SystemExit(f"failed to open video writer: {out_path}")

    n = 0
    pending = first
    pending_out = first_out
    while pending is not None:
        frame = pending
        pending = None
        if args.limit and n >= args.limit:
            break
        if pending_out is None:
            bev = render_ipm(frame.imgs, projection_maps, spec, blend=args.blend)
            review = draw_bev_review(bev, spec, scale=args.scale,
                                     label=f"frame {frame.idx}  t={frame.stamp:.3f}")
            pending_out = compose_video_frame(review, frame.imgs, spec.NY * args.scale,
                                              camera_strip=args.camera_strip)
        writer.write(pending_out)
        pending_out = None
        n += 1
        if n % 100 == 0:
            print(f"  wrote {n} frames")
        try:
            pending = next(frame_iter)
        except StopIteration:
            pending = None
    writer.release()
    print(f"done: {n} frames -> {out_path}")


if __name__ == "__main__":
    main()
