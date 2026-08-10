#!/usr/bin/env python3
"""map.pcd 에 '자기 자신'(카트를 끄는 수집자) 이 얼마나 남아 있는지 판정.

Point-LIO 의 `pcd_save.self_mask_*` 를 켜기 전/후를 같은 잣대로 비교하는 도구다.
절차·근거는 `docs/MAPPING.md §6.5`.

## 판정 원리

수집자는 카트 **뒤 0.8~0.9 m, |y|<=0.3, z<=0.85** 를 따라온다(260722 bag 7종 실측).
그래서 각 pose 의 **바디프레임**으로 맵을 되돌려서 self 박스 안의 점을 직접 센다.
궤적까지의 거리만으로 재면 좁은 통로의 좌우 벽(0.33~0.58 m)이 섞여 들어와 못 쓴다.

두 수치를 낸다:

  코어 (|y| < y_abs/2)  ← **이게 판정 대상.** 통로 중앙, 사람 말고는 있을 게 없는 자리
  박스 전체             ← 참고. 가장자리는 벽이 걸리므로 원래 0 이 되지 않는다

## 읽는 법

self_mask 전/후를 같은 bag 으로 비교한다. 260722 raws1 앞 25초 실측:

    mask off : 코어 1355 pts/pose
    mask on  : 코어  133 pts/pose   (-90%)

박스 전체가 안 줄어드는 건 정상이다. 맵은 전 시간의 누적이라, 앞쪽에 있을 때 찍힌 벽이
나중 pose 기준으로는 박스 뒤편에 들어앉는다. 마스크는 '찍히는 순간' 박스 안이던 점만 막는다.

코어가 안 떨어지면 사람이 마스크 밖으로 걸은 것이다. `--y-abs` 를 넓히거나
`--x-max` 를 0 쪽으로 옮겨 보고, 맞는 값을 `config/unilidar_l2.yaml` 에 반영한다.

## 눈으로 보기 (--png)

`<map_dir>/preview/self_xsec.png` 를 만든다. 후방 창(x_min~x_max)의 점을 pose 바디프레임
y-z 단면으로 전부 겹친 그림이다. 흰 세로선 = `|y|=y_abs`, 흰 가로선 = `z=z_max`(마스크 박스
경계), 회색 가로선 = 센서 높이. **박스 안만 비고 좌우 덩어리(통로 벽)는 그대로**여야 성공이다.

사용법:
  python3 mapping/check_self_points.py data/sj_bags/260722/maps_selfmask/raws1_mapping
  python3 mapping/check_self_points.py <mask전_out> <mask후_out> --png
"""
import argparse
import pathlib

import numpy as np
from scipy.spatial import cKDTree

from check_lidar_bag import turbo, write_png
from pcd_denoise import read_pcd_raw


def quat_to_R(q):
    """(qx,qy,qz,qw) -> 3x3 회전행렬."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


XSEC_RES, XSEC_YH, XSEC_ZL, XSEC_ZH = 0.02, 1.2, -0.3, 1.8   # 단면 그림 범위[m]


def write_xsec(path, hist, a):
    """후방 창의 y-z 단면 누적 → PNG. 마스크 박스 경계를 흰 선으로 그린다."""
    H, W = hist.shape
    img = np.full((H * W, 3), 12, np.uint8)
    occ = hist.reshape(-1) > 0
    if occ.any():
        v = np.log1p(hist.reshape(-1)[occ])
        img[occ] = turbo((v - v.min()) / max(v.ptp(), 1e-9))
    img = np.flipud(img.reshape(H, W, 3))
    for yy in (-a.y_abs, a.y_abs):                       # 박스 좌우 경계
        img[:, int((yy + XSEC_YH) / XSEC_RES)] = [255, 255, 255]
    img[H - 1 - int((a.z_max - XSEC_ZL) / XSEC_RES), :] = [255, 255, 255]   # 박스 상단
    img[H - 1 - int((0.0 - XSEC_ZL) / XSEC_RES), :] = [120, 120, 120]       # 센서 높이
    path.parent.mkdir(parents=True, exist_ok=True)
    write_png(path, np.repeat(np.repeat(img, 3, 0), 3, 1))


def analyze(map_dir, a):
    map_dir = pathlib.Path(map_dir)
    _, arr = read_pcd_raw(map_dir / "map.pcd")
    P = np.stack([arr["x"], arr["y"], arr["z"]], -1).astype(np.float64)
    tum = np.loadtxt(map_dir / "trajectory.tum")

    tree = cKDTree(P)
    reach = max(abs(a.x_min), a.y_abs, abs(a.z_max)) + 0.5
    W = int(2 * XSEC_YH / XSEC_RES)
    H = int((XSEC_ZH - XSEC_ZL) / XSEC_RES)
    hist = np.zeros(H * W, np.int64)
    core, box = [], []
    for i in np.linspace(0, len(tum) - 1, a.samples).astype(int):
        t, R = tum[i, 1:4], quat_to_R(tum[i, 4:8])
        idx = tree.query_ball_point(t, r=reach)
        if not idx:
            core.append(0), box.append(0)
            continue
        B = (P[idx] - t) @ R                                  # world -> body
        m = ((B[:, 0] > a.x_min) & (B[:, 0] < a.x_max)
             & (np.abs(B[:, 1]) < a.y_abs) & (B[:, 2] < a.z_max) & (B[:, 2] > a.z_min))
        box.append(int(m.sum()))
        core.append(int((m & (np.abs(B[:, 1]) < a.y_abs / 2)).sum()))
        if a.png:
            # 단면은 박스가 아니라 후방 창 전체를 본다(벽이 남았는지 같이 보려고)
            w = (B[:, 0] > a.x_min) & (B[:, 0] < a.x_max)
            y, z = B[w, 1], B[w, 2]
            k = (np.abs(y) < XSEC_YH) & (z > XSEC_ZL) & (z < XSEC_ZH)
            np.add.at(hist, (((z[k] - XSEC_ZL) / XSEC_RES).astype(int) * W
                             + ((y[k] + XSEC_YH) / XSEC_RES).astype(int)), 1)
    if a.png:
        write_xsec(map_dir / "preview" / "self_xsec.png", hist.reshape(H, W), a)
    return len(P), float(np.mean(core)), float(np.mean(box))


def main():
    ap = argparse.ArgumentParser(description="map.pcd 의 자기 자신(수집자) 잔재 판정")
    ap.add_argument("map_dir", nargs="+", help="map.pcd + trajectory.tum 이 있는 폴더")
    ap.add_argument("--x-min", type=float, default=-1.5, help="self 박스 x 하한[m]")
    ap.add_argument("--x-max", type=float, default=-0.45, help="self 박스 x 상한[m]")
    ap.add_argument("--y-abs", type=float, default=0.35, help="self 박스 |y| 한계[m]")
    ap.add_argument("--z-min", type=float, default=-0.2, help="self 박스 z 하한[m]")
    ap.add_argument("--z-max", type=float, default=1.0, help="self 박스 z 상한[m]")
    ap.add_argument("--samples", type=int, default=300, help="샘플링할 pose 수(기본 300)")
    ap.add_argument("--png", action="store_true",
                    help="<map_dir>/preview/self_xsec.png (후방 창 y-z 단면) 생성")
    args = ap.parse_args()

    print(f"self 박스: x {args.x_min}~{args.x_max}, |y|<{args.y_abs}, "
          f"z {args.z_min}~{args.z_max}  (코어 = |y|<{args.y_abs/2:.3f})")
    for d in args.map_dir:
        total, core, box = analyze(d, args)
        png = f"   -> {pathlib.Path(d)/'preview'/'self_xsec.png'}" if args.png else ""
        print(f"{pathlib.Path(d).name:26s} map {total:9,d} pts   "
              f"코어 {core:7.1f} pts/pose   박스전체 {box:7.1f} pts/pose{png}")


if __name__ == "__main__":
    main()
