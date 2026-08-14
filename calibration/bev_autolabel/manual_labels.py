#!/usr/bin/env python3
"""CVAT-style manual BEV annotation export -> training occupancy/visibility labels.

Expected manual folder:
  data/bev/manual_annotated/<dataset>_120x120_annotation/
    labelmap.txt
    SegmentationClass/sample_000000.png

Output:
  data/bev/manual_labels/<dataset>/
    rgb_images/sample_000000/cam_{front,left,right}.jpg
    occupancy_npy/*.npy      uint8, 0=obstacle, 1=drivable
    visibility_npy/*.npy     uint8, 0=unseen, 1=visible
    occupancy_png/*.png      indexed PNG preserving occupancy ids
    visibility_png/*.png     indexed PNG preserving visibility ids
    review_png/*.png         visual inspection images with fixed self/rear mask overlays
    labels.csv
"""
from __future__ import annotations

import argparse
import csv
import json
import pathlib
import re
import shutil
import sys

import cv2
import numpy as np
from PIL import Image

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))

from bev_label import BevSpec, raycast_visible  # noqa: E402
from gather_annotations import spec_from_meta  # noqa: E402
import slab_render as sr  # noqa: E402

USE = ("front", "left", "right")
SELF_MASK_OVERLAY_BGR = np.array([0, 255, 255], np.float64)
REAR_MASK_OVERLAY_BGR = np.array([255, 0, 255], np.float64)


def infer_dataset_name(manual_dir):
    """Infer `raws1` from `raws1_120x120_annotation`."""
    name = pathlib.Path(manual_dir).name
    m = re.match(r"^(.+)_\d+x\d+_annotation$", name)
    if not m:
        raise ValueError(f"manual annotation folder name must be <dataset>_<H>x<W>_annotation: {name}")
    return m.group(1)


def load_labelmap_color(manual_dir, label="occupancy"):
    """Read CVAT labelmap.txt and return the RGB color tuple for `label`."""
    path = pathlib.Path(manual_dir) / "labelmap.txt"
    if not path.exists():
        raise FileNotFoundError(path)
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(":")
        if len(parts) >= 2 and parts[0] == label:
            return tuple(int(v) for v in parts[1].split(","))
    raise ValueError(f"{path} has no {label!r} entry")


def mask_rgb_to_occupancy(rgb, occupancy_color, color_tol=8):
    """RGB mask -> uint8 occupancy using repo convention: 0=obstacle, 1=drivable."""
    img = np.asarray(rgb)
    if img.ndim != 3 or img.shape[2] != 3:
        raise ValueError(f"manual mask must be RGB, got shape {img.shape}")
    color = np.asarray(occupancy_color, dtype=np.int16)
    dist = np.max(np.abs(img.astype(np.int16) - color), axis=-1)
    obstacle = dist <= int(color_tol)
    return np.where(obstacle, 0, 1).astype(np.uint8)


def load_fixed_mask(sample_dir, spec, filename):
    """Load a fixed BEV mask from the sample metadata's self_mask folder."""
    sample = pathlib.Path(sample_dir)
    mp = sample / "meta.json"
    if not mp.exists():
        return None
    meta = json.loads(mp.read_text())
    mask_dir = meta.get("self_mask")
    if not mask_dir:
        return None
    path = pathlib.Path(mask_dir) / filename
    if not path.exists():
        return None
    mask = np.array(Image.open(path))
    if mask.shape != (spec.NX, spec.NY):
        raise ValueError(f"{path} shape {mask.shape} != {(spec.NX, spec.NY)}")
    return mask > 0


def overlay_mask_on_review(review, mask, spec, scale, color_bgr, alpha=0.65):
    if mask is None or scale <= 0:
        return review
    out = np.asarray(review).copy()
    h = spec.NX * scale
    w = spec.NY * scale
    y0 = out.shape[0] - h
    if y0 < 0:
        return out
    m = cv2.resize(mask.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST).astype(bool)
    for x0 in range(0, out.shape[1], w):
        x1 = min(x0 + w, out.shape[1])
        if x1 - x0 != w:
            continue
        roi = out[y0:y0 + h, x0:x1]
        roi[m] = (alpha * color_bgr + (1.0 - alpha) * roi[m]).astype(np.uint8)
    return out


def overlay_fixed_masks_on_review(review, sample_dir, spec, scale):
    out = overlay_mask_on_review(
        review, load_fixed_mask(sample_dir, spec, "bev_self_mask.png"), spec, scale,
        SELF_MASK_OVERLAY_BGR)
    return overlay_mask_on_review(
        out, load_fixed_mask(sample_dir, spec, "bev_rear_self_box_03.png"), spec,
        scale, REAR_MASK_OVERLAY_BGR)


def copy_rgb_images(sample_dir, dst_dir):
    dst_dir.mkdir(parents=True, exist_ok=True)
    copied = 0
    for name in USE:
        src = pathlib.Path(sample_dir) / f"cam_{name}.jpg"
        if not src.exists():
            print(f"[경고] RGB 이미지 없음: {src}")
            continue
        shutil.copyfile(str(src), str(dst_dir / src.name))
        copied += 1
    return copied


def _write_readme(path):
    path.write_text(
        "# Manual BEV labels\n\n"
        "Generated from `data/bev/manual_annotated/<dataset>_120x120_annotation/SegmentationClass`.\n\n"
        "- `occupancy_npy/*.npy`: uint8 `(NX, NY)`, `0=obstacle`, `1=drivable`.\n"
        "- `visibility_npy/*.npy`: uint8 `(NX, NY)`, `0=unseen`, `1=visible`.\n"
        "- `visibility` is regenerated by ray-casting from the manual occupancy mask.\n"
        "- `rgb_images/<sample>/cam_{front,left,right}.jpg` contains the original RGB inputs.\n"
        "- `occupancy_png/*.png` and `visibility_png/*.png` preserve the same class ids as indexed PNGs.\n"
        "- `review_png/*.png` is for visual inspection only.\n",
        encoding="utf-8",
    )


def generate_manual_labels(manual_dir, dataset_dir, out_root, name="", color_tol=8,
                           ray_step=0.25, review_scale=6):
    """Generate labels for one manual annotation folder.

    Returns a small stats dict for tests and CLI reporting.
    """
    manual = pathlib.Path(manual_dir)
    dataset_name = name or infer_dataset_name(manual)
    dataset = pathlib.Path(dataset_dir)
    samples = sorted((manual / "SegmentationClass").glob("sample_*.png"))
    if not samples:
        raise FileNotFoundError(f"no sample_*.png under {manual / 'SegmentationClass'}")

    ds_samples = sorted(d for d in dataset.glob("sample_*") if d.is_dir())
    if not ds_samples:
        raise FileNotFoundError(f"no sample_* directories under {dataset}")
    spec = spec_from_meta(ds_samples[0])
    expected_shape = (spec.NX, spec.NY)
    occ_color = load_labelmap_color(manual, "occupancy")

    root = pathlib.Path(out_root) / dataset_name
    stale_debug = root / "mask_debug_png"
    if stale_debug.exists():
        shutil.rmtree(stale_debug)
    for sub in ("rgb_images", "occupancy_npy", "visibility_npy", "occupancy_png",
                "visibility_png", "review_png"):
        (root / sub).mkdir(parents=True, exist_ok=True)

    rows = []
    for mask_path in samples:
        sample = mask_path.stem
        sample_dir = dataset / sample
        rgb = np.array(Image.open(mask_path).convert("RGB"))
        if rgb.shape[:2] != expected_shape:
            raise ValueError(f"{mask_path} shape {rgb.shape[:2]} != {expected_shape}")
        occupancy = mask_rgb_to_occupancy(rgb, occ_color, color_tol=color_tol)
        obstacle = occupancy == 0
        visibility = raycast_visible(obstacle, spec, step_deg=ray_step).astype(np.uint8)

        np.save(root / "occupancy_npy" / f"{sample}.npy", occupancy)
        np.save(root / "visibility_npy" / f"{sample}.npy", visibility)
        sr.save_indexed(root / "occupancy_png" / f"{sample}.png", occupancy, sr.PALETTE_OCC)
        sr.save_indexed(root / "visibility_png" / f"{sample}.png", visibility, sr.PALETTE_VIS)
        copy_rgb_images(sample_dir, root / "rgb_images" / sample)

        ipm_path = sample_dir / "ipm_rgb.png"
        ipm = cv2.imread(str(ipm_path)) if ipm_path.exists() else None
        if review_scale:
            cams = {name: cv2.imread(str(sample_dir / f"cam_{name}.jpg"))
                    for name in USE}
            review = sr.review_png(occupancy, visibility, spec, scale=review_scale,
                                   cam_imgs=cams, ipm=ipm)
            review = overlay_fixed_masks_on_review(review, sample_dir, spec, review_scale)
            cv2.imwrite(str(root / "review_png" / f"{sample}.png"), review)

        rows.append({
            "sample": sample,
            "rgb_dir": f"rgb_images/{sample}",
            "occupancy_npy": f"occupancy_npy/{sample}.npy",
            "visibility_npy": f"visibility_npy/{sample}.npy",
            "occupancy_png": f"occupancy_png/{sample}.png",
            "visibility_png": f"visibility_png/{sample}.png",
            "review_png": f"review_png/{sample}.png",
            "obstacle_cells": int(obstacle.sum()),
            "obstacle_pct": float(obstacle.mean() * 100.0),
            "visible_cells": int(visibility.sum()),
            "visible_pct": float(visibility.mean() * 100.0),
        })

    with (root / "labels.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    _write_readme(root / "README.md")

    return {
        "dataset": dataset_name,
        "samples": len(rows),
        "out": str(root),
        "spec": {
            "XF": spec.XF,
            "XR": spec.XR,
            "YH": spec.YH,
            "RES": spec.RES,
            "NX": spec.NX,
            "NY": spec.NY,
        },
        "obstacle_pct_min": min(r["obstacle_pct"] for r in rows),
        "obstacle_pct_max": max(r["obstacle_pct"] for r in rows),
        "visible_pct_min": min(r["visible_pct"] for r in rows),
        "visible_pct_max": max(r["visible_pct"] for r in rows),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manual-dir", required=True,
                    help="manual_annotated/<dataset>_120x120_annotation folder")
    ap.add_argument("--dataset-root", default=str(_HERE.parent.parent / "data" / "bev" / "dataset"),
                    help="root containing original dataset folders")
    ap.add_argument("--out", default=str(_HERE.parent.parent / "data" / "bev" / "manual_labels"),
                    help="output root; writes <out>/<dataset>")
    ap.add_argument("--name", default="",
                    help="dataset/output folder name; default inferred from manual-dir")
    ap.add_argument("--color-tol", type=int, default=8,
                    help="RGB tolerance for occupancy labelmap color")
    ap.add_argument("--ray-step", type=float, default=0.25,
                    help="raycast visibility angular step in degrees")
    ap.add_argument("--review-scale", type=int, default=6,
                    help="review_png BEV scale; 0 skips review images")
    a = ap.parse_args()

    dataset_name = a.name or infer_dataset_name(a.manual_dir)
    dataset_dir = pathlib.Path(a.dataset_root) / dataset_name
    stats = generate_manual_labels(a.manual_dir, dataset_dir, a.out, name=dataset_name,
                                   color_tol=a.color_tol, ray_step=a.ray_step,
                                   review_scale=a.review_scale)
    b = stats["spec"]
    print(f"BEV {b['NX']}x{b['NY']} (XF={b['XF']} XR={b['XR']} YH={b['YH']} RES={b['RES']})")
    print(f"generated {stats['samples']} samples -> {stats['out']}")
    print("obstacle_pct range "
          f"{stats['obstacle_pct_min']:.2f}..{stats['obstacle_pct_max']:.2f}")
    print("visible_pct range "
          f"{stats['visible_pct_min']:.2f}..{stats['visible_pct_max']:.2f}")


if __name__ == "__main__":
    main()
