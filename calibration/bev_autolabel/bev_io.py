"""BEV auto-label IO: 맵/궤적/stamp/이미지 로드. open3d·cv2 지연 import."""
from __future__ import annotations

import csv
import pathlib

import numpy as np

import sys
_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
from cloud_io import load_tum  # noqa: E402


def load_map(map_dir):
    """map.pcd + trajectory.tum → (p(N,3), times_ns, poses, tpos(T,3))."""
    import open3d as o3d
    md = pathlib.Path(map_dir)
    p = np.asarray(o3d.io.read_point_cloud(str(md / "map.pcd")).points, float)
    times_ns, poses = load_tum(str(md / "trajectory.tum"))
    tpos = np.array([P[:3, 3] for P in poses], float)
    return p, times_ns, poses, tpos


def load_stamps(extract_dir):
    """sets.csv → {idx: stamp_ns}. stamp0(=front 기준) 을 ns 로."""
    out = {}
    with open(pathlib.Path(extract_dir) / "sets.csv") as f:
        for r in csv.DictReader(f):
            out[int(r["idx"])] = int(round(float(r["stamp0"]) * 1e9))
    return out


def load_cam_images(extract_dir, idx, idx_to_name, use_names):
    """frame_NNNNNN/cam{k}.jpg → {name: BGR ndarray}. rig.idx_to_name 로 name 매핑."""
    import cv2
    ed = pathlib.Path(extract_dir)
    name2idx = {v: k for k, v in idx_to_name.items()}
    out = {}
    for name in use_names:
        k = name2idx[name]
        img = cv2.imread(str(ed / f"frame_{idx:06d}" / f"cam{k}.jpg"))
        out[name] = img
    return out
