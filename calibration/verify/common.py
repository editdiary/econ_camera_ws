"""공용 유틸: 프레임 로딩, 프레임 선택 파싱, 라벨/저장, 회전 렌더, 공통 인자."""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from rectify import rays_to_maps


def list_frames(images_root):
    root = Path(images_root)
    return sorted(p.name for p in root.iterdir() if p.is_dir() and p.name.startswith("frame_"))


def parse_frames(spec, available):
    """--frames 파싱: "0,10,20" | "0-499:50"(start-end:step) | "" -> 앞 6장 등간격."""
    if not spec:
        n = len(available)
        step = max(1, n // 6)
        return available[::step][:6]
    names = []
    for tok in spec.split(","):
        tok = tok.strip()
        if "-" in tok:
            rng, _, st = tok.partition(":")
            a, b = rng.split("-")
            step = int(st) if st else 1
            for i in range(int(a), int(b) + 1, step):
                names.append(f"frame_{i:06d}")
        else:
            names.append(f"frame_{int(tok):06d}")
    return [n for n in names if n in set(available)]


def load_frame(images_root, frame_name, indices):
    """{idx: BGR img}. 없는 파일은 건너뜀."""
    d = Path(images_root) / frame_name
    out = {}
    for idx in indices:
        p = d / f"cam{idx}.jpg"
        if not p.exists():
            p = d / f"cam{idx}.png"
        if p.exists():
            img = cv2.imread(str(p))
            if img is not None:
                out[idx] = img
    return out


def label(img, text, org=(12, 34), scale=0.9, color=(0, 255, 0)):
    out = img.copy()
    cv2.putText(out, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(out, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)
    return out


def hstack_pad(imgs, pad=6, bg=40):
    """높이를 맞춰 가로로 이어 붙임(구분 여백 포함)."""
    h = max(i.shape[0] for i in imgs)
    parts = []
    for k, im in enumerate(imgs):
        if im.shape[0] != h:
            s = h / im.shape[0]
            im = cv2.resize(im, (int(im.shape[1] * s), h))
        parts.append(im)
        if k != len(imgs) - 1:
            parts.append(np.full((h, pad, 3), bg, np.uint8))
    return np.hstack(parts)


def render_into(cam, img, rays_ref, R_cam_ref):
    """기준 프레임 ray 를 R_cam_ref 로 카메라 프레임에 회전시켜 렌더.

    ray_cam = R_cam_ref @ ray_ref. 반환 (렌더 이미지, valid 마스크).
    """
    rays_cam = rays_ref @ R_cam_ref.T
    map_x, map_y, valid = rays_to_maps(cam, rays_cam)
    out = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    out[~valid] = 0
    # 광축(+z)에서 멀수록 낮은 가중치(이음새 페더링용)
    w = np.clip(rays_cam[..., 2], 0, 1) ** 2
    w[~valid] = 0
    return out, w


def add_common_args(p: argparse.ArgumentParser, default_out):
    p.add_argument("--calib", default="data/calib_260723/calib.yaml")
    p.add_argument("--orientation", default="data/calib_260723/orientation.json")
    p.add_argument("--images", required=True,
                   help="frame_XXXXXX/cam{0..3}.jpg 를 담은 루트 디렉터리")
    p.add_argument("--frames", default="", help='예: "0,10,20" 또는 "0-499:50" (기본: 등간격 6장)')
    p.add_argument("--out", default=default_out, help="PNG 출력 디렉터리")
    return p
