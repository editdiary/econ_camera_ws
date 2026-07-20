#!/usr/bin/env python3
"""scans.pcd -> per-cell BEV height map + density map (numpy only, headless).
셀마다 값 하나로 집계 = 실제 BEV 라벨 형식. 사용법: bev_grid.py <pcd> [out_dir]"""
import os
import struct
import sys
import zlib

import numpy as np

PCD = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(PCD))
os.makedirs(OUT, exist_ok=True)
RES = 0.05  # meters per cell

with open(PCD, "rb") as f:
    raw = f.read()
he = raw.find(b"DATA binary\n") + len(b"DATA binary\n")
hdr = raw[:he].decode("ascii", "replace")
sizes, counts, npts = [], [], 0
for line in hdr.splitlines():
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
d = np.frombuffer(raw[he:he + npts * stride], np.uint8).reshape(npts, stride)
xyz = d[:, :12].copy().view(np.float32).reshape(npts, 3).astype(np.float64)
x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]


def turbo(t):
    t = np.clip(t, 0, 1)
    r = np.clip(1.5 - abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - abs(4 * t - 1), 0, 1)
    return (np.stack([r, g, b], -1) * 255).astype(np.uint8)


def write_png(path, img):
    h, w, _ = img.shape

    def ch(t, dd):
        c = t + dd
        return struct.pack(">I", len(dd)) + c + struct.pack(">I", zlib.crc32(c) & 0xffffffff)

    body = b"".join(b"\x00" + img[i].tobytes() for i in range(h))
    open(path, "wb").write(b"\x89PNG\r\n\x1a\n"
        + ch(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + ch(b"IDAT", zlib.compress(body, 9)) + ch(b"IEND", b""))


xlo, xhi = np.percentile(x, [0.3, 99.7])
ylo, yhi = np.percentile(y, [0.3, 99.7])
W = int((xhi - xlo) / RES) + 1
H = int((yhi - ylo) / RES) + 1
ix = ((x - xlo) / RES).astype(int)
iy = ((yhi - y) / RES).astype(int)
m = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
ix, iy, zz = ix[m], iy[m], z[m]
cell = iy * W + ix

hmax = np.full(W * H, -np.inf)
np.maximum.at(hmax, cell, zz)
occ = np.isfinite(hmax)
img = np.zeros((W * H, 3), np.uint8)
lo, hi = np.percentile(hmax[occ], [2, 98])
img[occ] = turbo((hmax[occ] - lo) / ((hi - lo) or 1))
write_png(f"{OUT}/bev_heightmap.png", img.reshape(H, W, 3))

cnt = np.bincount(cell, minlength=W * H).astype(np.float64)
occ2 = cnt > 0
img2 = np.zeros((W * H, 3), np.uint8)
lc = np.log1p(cnt[occ2])
img2[occ2] = turbo(lc / (lc.max() or 1))
write_png(f"{OUT}/bev_density.png", img2.reshape(H, W, 3))

filled = int(occ.sum())
print(f"grid {W}x{H} @ {RES}m  occupied {filled}/{W*H} = {100*filled/(W*H):.0f}%  max_pts_in_cell={int(cnt.max())}")
