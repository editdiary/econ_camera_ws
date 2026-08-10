"""슬래브 라벨 렌더: 인덱스 팔레트 PNG + 4색 검수뷰."""
from __future__ import annotations

import numpy as np
import cv2
from PIL import Image

# occupancy: 0=obstacle 빨강, 1=drivable 초록 (기존 label.png 와 같은 약속)
PALETTE_OCC = [255, 0, 0, 0, 200, 0] + [0] * (256 * 3 - 6)
# visibility: 0=unseen 검정, 1=visible 흰색
PALETTE_VIS = [0, 0, 0, 255, 255, 255] + [0] * (256 * 3 - 6)

_C_VIS_DRIV = (0, 190, 0)      # BGR 초록: 보이는 drivable = 학습에 쓸 신뢰 영역
_C_VIS_OBS = (40, 40, 220)     # 빨강: 보이는 장애물 표면
_C_OCC_OBS = (60, 60, 110)     # 갈색: 가려진 장애물
_C_UNSEEN = (30, 30, 30)       # 검정: 미관측
_C_EGO = (0, 255, 255)


def save_indexed(path, arr, palette):
    """uint8 인덱스 배열을 팔레트 PNG 로. 화소값은 인덱스 그대로 보존된다."""
    im = Image.fromarray(np.asarray(arr, np.uint8), mode="P")
    im.putpalette(palette)
    im.save(str(path))


def four_color(occupancy, visibility):
    """(NX,NY,3) BGR. occ/vis 네 조합을 서로 다른 색으로."""
    occ = np.asarray(occupancy)
    vis = np.asarray(visibility).astype(bool)
    obstacle = occ == 0
    out = np.zeros(occ.shape + (3,), np.uint8)
    out[...] = _C_UNSEEN
    out[vis & ~obstacle] = _C_VIS_DRIV
    out[vis & obstacle] = _C_VIS_OBS
    out[~vis & obstacle] = _C_OCC_OBS
    return out


def review_png(occupancy, visibility, spec, scale=6):
    """4색 확대 + 0.5m 격자 + ego 마커 + 전방 화살표.

    격자선 위치는 미터좌표 기준으로 잡는다 — 격자 좌상단 기준이면 XF/YH 가 0.5 배수가
    아닐 때 선과 미터 라벨이 어긋나 거리를 잘못 읽는다.
    """
    img = cv2.resize(four_color(occupancy, visibility),
                     (spec.NY * scale, spec.NX * scale),
                     interpolation=cv2.INTER_NEAREST)
    H, W = img.shape[:2]
    step = 0.5
    for x_m in np.arange(np.floor(spec.XF / step) * step, -spec.XR - 1e-9, -step):
        gy = int(round((spec.XF - x_m) / spec.RES)) * scale
        if 0 <= gy < H:
            cv2.line(img, (0, gy), (W, gy), (70, 70, 70), 1)
    for y_m in np.arange(np.floor(spec.YH / step) * step, -spec.YH - 1e-9, -step):
        gx = int(round((spec.YH - y_m) / spec.RES)) * scale
        if 0 <= gx < W:
            cv2.line(img, (gx, 0), (gx, H), (70, 70, 70), 1)
    for r_m in range(-int(spec.XR), int(spec.XF) + 1):
        py = int(round((spec.XF - r_m) / spec.RES)) * scale
        cv2.putText(img, f"{r_m}m", (2, max(12, min(py, H - 2))),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1, cv2.LINE_AA)
    ex, ey = spec.C_EGO * scale, spec.R_EGO * scale
    cv2.circle(img, (ex, ey), max(2, scale // 2), _C_EGO, -1)
    cv2.arrowedLine(img, (ex, ey), (ex, max(0, ey - 6 * scale)), _C_EGO, 1,
                    tipLength=0.3)
    return img
