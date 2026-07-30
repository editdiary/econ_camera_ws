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


def _grid_axes(bev, spec, scale, step_m=0.5):
    """0.5m 격자선 + 정수 미터 축 라벨. 선 위치는 **미터좌표 기준**(격자 좌상단 기준이면
    XF/YH 가 0.5 배수가 아닐 때 선과 미터 라벨이 어긋나 거리를 잘못 읽는다)."""
    H, W = bev.shape[:2]
    for x_m in np.arange(np.floor(spec.XF / step_m) * step_m, -spec.XR - 1e-9, -step_m):
        gy = int(round((spec.XF - x_m) / spec.RES)) * scale
        cv2.line(bev, (0, gy), (W, gy), (60, 60, 60), 1)
    for y_m in np.arange(np.floor(spec.YH / step_m) * step_m, -spec.YH - 1e-9, -step_m):
        gx = int(round((spec.YH - y_m) / spec.RES)) * scale
        cv2.line(bev, (gx, 0), (gx, H), (60, 60, 60), 1)
    for r_m in range(-int(spec.XR), int(spec.XF) + 1):     # 축 라벨(전방 x: 위로 +)
        py = int(round((spec.XF - r_m) / spec.RES)) * scale
        cv2.putText(bev, f"{r_m}m", (2, max(12, py)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (255, 255, 0), 1, cv2.LINE_AA)


def review_image(label, spec, cam_imgs, scale=8):
    """BEV 확대 + 미터축·0.5m 격자·ego·전방화살표, 상단 3카메라 가로 배치."""
    bev = cv2.resize(colorize(label), (spec.NY * scale, spec.NX * scale),
                     interpolation=cv2.INTER_NEAREST)
    W = bev.shape[1]
    _grid_axes(bev, spec, scale)
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


def _camera_row(cam_imgs, W, order=("left", "front", "right")):
    """상단 카메라 가로 배치(폭 W에 맞춤). 파노라마 순서 좌·전·우. 없으면 None."""
    order = [n for n in order if cam_imgs.get(n) is not None]
    if not order:
        return None
    cw = W // len(order)
    row = []
    for n in order:
        im = cam_imgs[n]
        r = cv2.resize(im, (cw, int(cw * im.shape[0] / im.shape[1])))
        cv2.putText(r, n, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        row.append(r)
    top = np.hstack(row)
    if top.shape[1] != W:
        top = cv2.resize(top, (W, top.shape[0]))
    return top


def blend_label(ipm, label, alpha=0.45):
    """IPM RGB 캔버스에 라벨(obstacle/drivable) 반투명 오버레이. **네이티브 해상도·장식 없음.**

    CVAT 등에 올려 그 위에서 라벨을 보정하는 annotation base(예: 80×80). 격자·미터축·ego
    박스는 셀 크기와 맞먹어 네이티브에선 실제 셀을 덮으므로 넣지 않는다(확대 검수는 label_overlay).
    ignore(2) 셀은 배경(IPM 이미지)을 그대로 보여줘 어떤 장면인지 판단할 수 있게 한다.
    """
    over = ipm.copy()
    lab = colorize(label)
    mm = (label == 0) | (label == 1)
    over[mm] = (alpha * lab[mm] + (1 - alpha) * ipm[mm]).astype(np.uint8)
    return over


def label_overlay(ipm, label, spec, alpha=0.45, scale=9):
    """blend_label 를 scale 배 확대 + 미터축·격자·ego 박스(= review.png 의 아래 BEV, 사람 검수용).
    카메라 행은 붙이지 않는다. 장식이 있어 CVAT annotation base 로는 부적합(그건 blend_label)."""
    bev = cv2.resize(blend_label(ipm, label, alpha), (spec.NY * scale, spec.NX * scale),
                     interpolation=cv2.INTER_NEAREST)
    _grid_axes(bev, spec, scale)
    ex, ey = spec.C_EGO * scale, spec.R_EGO * scale
    cv2.arrowedLine(bev, (ex, ey), (ex, ey - 4 * scale), (255, 128, 0), 2, tipLength=0.3)
    half = int(round(0.2 / spec.RES)) * scale            # 40x40cm footprint
    cv2.rectangle(bev, (ex - half, ey - half), (ex + half, ey + half), (255, 128, 0), 2)
    return bev


def review_overlay(ipm, label, cam_imgs, spec, alpha=0.45, scale=9):
    """label_overlay(아래 BEV) + 상단 원본 3어안(좌·전·우). 사람이 이 뷰에서 라벨을 보정한다."""
    bev = label_overlay(ipm, label, spec, alpha=alpha, scale=scale)
    top = _camera_row(cam_imgs, bev.shape[1])
    return bev if top is None else np.vstack([top, bev])
