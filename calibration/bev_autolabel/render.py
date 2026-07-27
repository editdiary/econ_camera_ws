"""BEV 라벨 렌더: 색칠 + 미터축·격자 검수뷰(3카메라 합성)."""
from __future__ import annotations

import numpy as np
import cv2

_COLOR = {0: (0, 0, 255), 1: (0, 200, 0), 2: (128, 128, 128)}  # BGR


def colorize(label):
    out = np.zeros((label.shape[0], label.shape[1], 3), np.uint8)
    for v, col in _COLOR.items():
        out[label == v] = col
    return out


def review_image(label, spec, cam_imgs, scale=8):
    """BEV 확대 + 미터축·0.5m 격자·ego·전방화살표, 상단 3카메라 가로 배치."""
    bev = cv2.resize(colorize(label), (spec.NY * scale, spec.NX * scale),
                     interpolation=cv2.INTER_NEAREST)
    H, W = bev.shape[:2]
    # 0.5m 격자 + 미터 라벨
    step = int(round(0.5 / spec.RES)) * scale
    for gx in range(0, W, step):
        cv2.line(bev, (gx, 0), (gx, H), (60, 60, 60), 1)
    for gy in range(0, H, step):
        cv2.line(bev, (0, gy), (W, gy), (60, 60, 60), 1)
    # 축 라벨(전방 x: 위로 +, 좌우 y)
    for r_m in range(-int(spec.XR), int(spec.XF) + 1):
        py = int((spec.XF - r_m) / spec.RES) * scale
        cv2.putText(bev, f"{r_m}m", (2, max(12, py)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (255, 255, 0), 1, cv2.LINE_AA)
    # ego 위치 + 전방 화살표
    ex, ey = spec.C_EGO * scale, spec.R_EGO * scale
    cv2.circle(bev, (ex, ey), 4, (255, 255, 255), -1)
    cv2.arrowedLine(bev, (ex, ey), (ex, ey - 4 * scale), (255, 255, 255), 2, tipLength=0.3)
    half = int(round(0.2 / spec.RES)) * scale            # 40x40cm footprint, half = 0.2m
    cv2.rectangle(bev, (ex - half, ey - half), (ex + half, ey + half), (255, 255, 255), 2)
    # 상단 3카메라(front/left/right) 가로 배치, BEV 폭에 맞춤
    order = [n for n in ("front", "left", "right") if cam_imgs.get(n) is not None]
    if order:
        cw = W // len(order)
        row = []
        for n in order:
            im = cv2.resize(cam_imgs[n], (cw, int(cw * 0.5625)))  # 16:9
            cv2.putText(im, n, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            row.append(im)
        top = np.hstack(row)
        if top.shape[1] != W:
            top = cv2.resize(top, (W, top.shape[0]))
        return np.vstack([top, bev])
    return bev
