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


def blend_slab(ipm, occupancy, alpha=0.30):
    """IPM RGB 캔버스에 obstacle 만 반투명 오버레이. 네이티브 해상도·장식 없음.

    CVAT 등에 올려 그 위에서 라벨을 보정하는 annotation base 다. 격자·미터축은 셀 크기와
    맞먹어 네이티브에선 실제 셀을 덮으므로 넣지 않는다(확대 검수는 review_png).

    보정 대상은 occupancy 뿐이다 — visibility 는 보정된 occupancy 로 raycast_visible 을
    다시 돌리면 재생성되므로 이 base 에 실을 이유가 없다. 예전엔 vis=1 셀만 칠했는데,
    raycast 가 장애물 앞면에서 멈춰 obstacle 의 96%(raws3 실측)가 아무 표시도 못 받았다.
    drivable 은 칠하지 않는다 — 잉크를 보정이 필요한 곳에만 두고 IPM 을 최대한 살린다.
    """
    over = np.asarray(ipm).copy()
    m = np.asarray(occupancy) == 0
    over[m] = (alpha * np.float64(_C_VIS_OBS) + (1 - alpha) * over[m]).astype(np.uint8)
    return over


def _camera_strip(cam_imgs, width, order=("left", "front", "right")):
    """review_png 상단에 붙일 카메라 가로 배치. 폭 width 에 맞춰 등분·이름표. 없으면 None.

    render.py 의 review_overlay/_camera_row 와 같은 방식(3등분 리사이즈 + 라벨)이지만
    slab_render 는 render.py 를 참조하지 않고 독립적으로 유지한다.
    """
    names = [n for n in order if cam_imgs.get(n) is not None]
    if not names:
        return None
    cw = width // len(names)
    row = []
    for n in names:
        im = cam_imgs[n]
        r = cv2.resize(im, (cw, int(cw * im.shape[0] / im.shape[1])))
        cv2.putText(r, n, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        row.append(r)
    top = np.hstack(row)
    if top.shape[1] != width:
        top = cv2.resize(top, (width, top.shape[0]))
    return top


def _grid_axes_ego(img, spec, scale):
    """0.5m 격자선 + 정수 미터 축 라벨 + ego 마커 + 전방 화살표(제자리 수정).

    격자선 위치는 미터좌표 기준으로 잡는다 — 격자 좌상단 기준이면 XF/YH 가 0.5 배수가
    아닐 때 선과 미터 라벨이 어긋나 거리를 잘못 읽는다.
    """
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


def review_png(occupancy, visibility, spec, scale=6, cam_imgs=None, ipm=None, alpha=0.30):
    """4색 확대 + 0.5m 격자 + ego 마커 + 전방 화살표. cam_imgs 주면 상단에 원본 3어안을 붙인다.

    cam_imgs={name: BGR ndarray} 는 선택 인자다 — 안 주면(기본값 None) 기존과 똑같이
    BEV 만 반환한다. 라벨만 봐서는 실제로 맞는 라벨인지 판단하기 어렵다는 사용자 피드백에
    따라 원본 이미지를 나란히 붙여 사람이 한 장으로 대조할 수 있게 한다.

    ipm=(NX,NY,3) BGR 도 선택 인자다 — 주면 4색 BEV 패널 옆에 IPM+occupancy(blend_slab)
    패널을 나란히 붙이고 각 패널에 제목을 얹는다. 안 주면(기본값 None) 4색 패널 단독으로,
    기존 동작을 그대로 유지한다. visibility 는 4색 패널이 계속 보여준다.
    """
    bev = cv2.resize(four_color(occupancy, visibility),
                     (spec.NY * scale, spec.NX * scale),
                     interpolation=cv2.INTER_NEAREST)
    _grid_axes_ego(bev, spec, scale)
    img = bev
    if ipm is not None:
        ov = cv2.resize(blend_slab(ipm, occupancy, alpha),
                        (spec.NY * scale, spec.NX * scale),
                        interpolation=cv2.INTER_NEAREST)
        _grid_axes_ego(ov, spec, scale)
        cv2.putText(bev, "4color(occ+vis)", (4, bev.shape[0] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(ov, "ipm+occupancy", (4, ov.shape[0] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        img = np.hstack([bev, ov])
    if cam_imgs:
        top = _camera_strip(cam_imgs, img.shape[1])
        if top is not None:
            img = np.vstack([top, img])
    return img
