#!/usr/bin/env python3
"""scans.pcd -> top-down(BEV) + side preview PNG (numpy only, headless).
사용법: pcd_preview.py <pcd> [out_dir]"""
import os
import struct
import sys
import zlib

import numpy as np

PCD = sys.argv[1]
OUT_DIR = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(PCD))
os.makedirs(OUT_DIR, exist_ok=True)

with open(PCD, "rb") as f:
    raw = f.read()
hdr_end = raw.find(b"DATA binary\n") + len(b"DATA binary\n")
header = raw[:hdr_end].decode("ascii", "replace")
sizes, counts, npts = [], [], 0
for line in header.splitlines():
    p = line.split()
    if not p:
        continue
    if p[0] == "SIZE":
        sizes = list(map(int, p[1:]))
    elif p[0] == "COUNT":
        counts = list(map(int, p[1:]))
    elif p[0] == "POINTS":
        npts = int(p[1])
stride = sum(s * c for s, c in zip(sizes, counts))
data = np.frombuffer(raw[hdr_end:hdr_end + npts * stride], dtype=np.uint8).reshape(npts, stride)
xyz = data[:, :12].copy().view(np.float32).reshape(npts, 3).astype(np.float64)
x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
print(f"points={npts}  x[{x.min():.1f},{x.max():.1f}] y[{y.min():.1f},{y.max():.1f}] z[{z.min():.1f},{z.max():.1f}]")


def turbo(t):
    t = np.clip(t, 0, 1)
    r = np.clip(1.5 - abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - abs(4 * t - 1), 0, 1)
    return (np.stack([r, g, b], -1) * 255).astype(np.uint8)


def write_png(path, img):
    h, w, _ = img.shape

    def chunk(typ, d):
        c = typ + d
        return struct.pack(">I", len(d)) + c + struct.pack(">I", zlib.crc32(c) & 0xffffffff)

    body = b"".join(b"\x00" + img[i].tobytes() for i in range(h))
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(body, 9))
           + chunk(b"IEND", b""))
    open(path, "wb").write(png)


def rasterize(a, b, cval, name, res_px=900, pad=0.02):
    alo, ahi = np.percentile(a, [0.5, 99.5])
    blo, bhi = np.percentile(b, [0.5, 99.5])
    da = (ahi - alo) or 1
    db = (bhi - blo) or 1
    alo -= da * pad; ahi += da * pad; blo -= db * pad; bhi += db * pad
    W = res_px
    H = max(60, int(res_px * (bhi - blo) / (ahi - alo)))
    ia = ((a - alo) / (ahi - alo) * (W - 1)).astype(int)
    ib = ((bhi - b) / (bhi - blo) * (H - 1)).astype(int)
    m = (ia >= 0) & (ia < W) & (ib >= 0) & (ib < H)
    ia, ib, cv = ia[m], ib[m], cval[m]
    flat = ib * W + ia
    lo, hi = np.percentile(cv, [2, 98])
    t = (cv - lo) / ((hi - lo) or 1)
    order = np.argsort(cv)
    img = np.zeros((H * W, 3), np.uint8)
    img[flat[order]] = turbo(t[order])
    write_png(f"{OUT_DIR}/{name}.png", img.reshape(H, W, 3))
    print(f"{name}.png  {W}x{H}px  span {ahi-alo:.1f}x{bhi-blo:.1f} m")


rasterize(x, y, z, "bev_topdown")
rasterize(x, z, z, "side_xz")
