"""BEV 슬래브 라벨 순수 로직. 하드웨어·파일 불필요.

좌표계: ego=body=LiDAR, x=전방·y=좌·z=상. BEV row=(XF-x)/RES, col=(YH-y)/RES.
라벨: occupancy 0=obstacle 1=drivable, visibility 0=unseen 1=visible.

LiDAR 가 바닥을 못 보고 자기 수평면 근처부터 돔으로 수집하므로, crop 안 최저 z 가
LiDAR 수평면(실제 지상 약 0.87m)이다. 슬래브 [z_ref, z_ref+thick] 은 로봇이 통과해야
하는 높이 구간을 뜻한다. 근거·실측: docs/superpowers/specs/2026-08-10-bev-slab-label-design.md
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))

from bev_label import rc_of, cell_centers  # noqa: E402
from chain import project                  # noqa: E402


def crop_mask(P_body, spec):
    """body 프레임 점 (N,3) → BEV 박스 안인지 (N,) bool. z 는 제한하지 않는다.

    z 를 여기서 자르지 않는 이유: z_ref 를 crop 안에서 구해야 전역 드리프트에 면역이 된다.
    """
    P = np.asarray(P_body, float)
    if len(P) == 0:
        return np.zeros(0, bool)
    return ((P[:, 0] <= spec.XF) & (P[:, 0] >= -spec.XR)
            & (np.abs(P[:, 1]) <= spec.YH))


def ref_z(z, pct=1.0):
    """슬래브 하단 = z 의 하위 pct 퍼센타일. min 이 아닌 이유는 이상치 flyer 때문."""
    return float(np.percentile(np.asarray(z, float), pct))


def slab_mask(P_crop, z_ref, thick=0.8):
    """crop 점 (M,3) → [z_ref, z_ref+thick] 밴드 안인지 (M,) bool. 양쪽 경계 포함."""
    P = np.asarray(P_crop, float)
    if len(P) == 0:
        return np.zeros(0, bool)
    return (P[:, 2] >= z_ref) & (P[:, 2] <= z_ref + thick)


def occupancy_counts(P_slab, spec):
    """슬래브 점을 (row,col) 로 눌러 셀당 점 개수 (NX,NY) int.

    2D 기둥 누적이면 충분하다 — 슬래브 자르기가 flatten 앞에 오므로 로봇보다 높은
    장애물(열린 문·상인방·천장)은 이미 배제돼 있다.
    """
    cnt = np.zeros((spec.NX, spec.NY), int)
    P = np.asarray(P_slab, float)
    if len(P) == 0:
        return cnt
    r, c = rc_of(P[:, 0], P[:, 1], spec)
    ok = (r >= 0) & (r < spec.NX) & (c >= 0) & (c < spec.NY)
    np.add.at(cnt, (r[ok], c[ok]), 1)
    return cnt


def obstacle_from_counts(counts, min_pts=3):
    """count >= min_pts 인 셀만 obstacle. min_pts=3 은 실측으로 정한 값 —
    카트가 실제 지나간 궤적 셀이 obstacle 로 찍히는 비율이 N=1 에서 2.8~4.8%,
    N=3 에서 0.2~1.2% 다. N>=10 은 실구조물까지 지운다."""
    return np.asarray(counts) >= min_pts


def vignette_mask(frames_gray, dark=25, quantile=0.90):
    """프레임 스택 (F,H,W) → '거의 항상 어두운' 화소 = 어안 유효원 바깥. True=무효.

    지면점이 1280x720 사각형 안에만 들어오면 보인다고 세면 틀린다 — 각 이미지의
    12~18%(좌우단 열에서는 60~66%)가 어안 원 바깥 검은 영역이다.

    max 가 아니라 퍼센타일을 쓰는 이유는 JPEG 노이즈로 한두 프레임만 튄 화소에 속지
    않기 위함이다. 0.98 은 너무 높다 — 프레임 40장이면 상위 1장이 그대로 결과를
    뒤집는다(20장·이상치 1개면 quantile(0.98)=158 로 유효 판정된다). 0.90 은 상위
    10% 를 버려 강건하다. 어안 원 바깥이 프레임의 10% 이상 밝아지는 일은 없다.
    """
    A = np.asarray(frames_gray)
    return np.quantile(A, quantile, axis=0) < dark


def camera_observable(spec, z_body, cams, T_cam_front, T_front_lidar, invalid,
                      use_names=("front", "left", "right")):
    """각 BEV 셀을 z_body 평면에 놓고 카메라별로 판정한 뒤 OR. (NX,NY) bool.

    판정 평면이 결정적이다. body z=0 은 카메라 높이(=수평선) 평면이라 그 위의 점은
    이미지 세로 중앙 근처에 찍혀 항상 유효해 보인다(측정 100%). 실제 지면
    z=-0.87 에서 재야 사각지대가 드러난다(96%, 반경 0.5m 완전 사각).

    invalid={name: (H,W) bool}. True 인 화소는 카트 자기 몸(상판·프레임·팔)이나
    어안 원 바깥이라 그 방향은 못 본 것으로 센다. 없는 이름은 마스크 없이 판정한다.
    """
    X, Y = cell_centers(spec)
    P = np.stack([X, Y, np.full_like(X, float(z_body))], axis=-1).reshape(-1, 3)
    ok_any = np.zeros(len(P), bool)
    for name in use_names:
        cam = cams[name]
        u, v, valid = project(P, T_front_lidar, T_cam_front[name], cam)
        ui = np.floor(np.nan_to_num(u, nan=-1.0)).astype(int)
        vi = np.floor(np.nan_to_num(v, nan=-1.0)).astype(int)
        good = (valid & (ui >= 0) & (ui < cam.width)
                & (vi >= 0) & (vi < cam.height))
        m = invalid.get(name)
        if m is not None:
            idx = np.flatnonzero(good)
            good[idx] = ~m[vi[idx], ui[idx]]
        ok_any |= good
    return ok_any.reshape(spec.NX, spec.NY)


def self_box_mask(spec, near=0.4, far=2.1, yh=0.7):
    """후방 self 박스: -far <= x <= -near 이고 |y| <= yh 인 셀 (NX,NY) bool. True=무효.

    카트 손잡이(폭 실측 46cm)와 이를 밀며 따라오는 수집자(LiDAR 기준 약 60cm 후방)를 덮어
    visibility 를 0 으로 만든다.

    이미지 마스크가 아니라 body 프레임 박스로 처리하는 이유: 수집자는 화면을 확인하려 몸을
    기울이고, 회전 구간에서 위치가 바뀌고, 턱에 걸려 흔들린다. 정적 이미지 마스크는 그 변동
    앞에서 없는 자리를 가리고(데이터 손실) 있는 자리를 놓친다(틀린 라벨). 물리적 위치는
    body 프레임에서 늘 같은 영역이다.

    기본값은 실측으로 정했다. 이 박스(896셀, 6.2%)는 (1) handle·human 이미지 마스크가
    죽이던 셀 143개(x −1.98~−0.98m, |y| 최대 0.68m)를 100% 포함하고, (2) Point-LIO
    self mask 박스(x −1.5~−0.45, |y|<0.35)도 완전히 담는다 — 후자는 map_clean.pcd 에 남은
    수집자 잔재(허위 obstacle)의 위치이며 이미지 마스크로는 고칠 수 없는 부분이다.
    """
    X, Y = cell_centers(spec)
    return (X >= -far) & (X <= -near) & (np.abs(Y) <= yh)


def assemble(obstacle, visible, camera_ok):
    """(occupancy, visibility) uint8. occupancy 0=obstacle 1=drivable,
    visibility 0=unseen 1=visible.

    occupancy 에 unknown 클래스를 두지 않는다 — '한 번도 관측되지 않은 영역'은
    visibility=0 이 정확히 그 뜻이고, 두 채널의 네 조합이 각각 의미를 갖는다.
    """
    occupancy = np.where(np.asarray(obstacle), 0, 1).astype(np.uint8)
    visibility = (np.asarray(visible) & np.asarray(camera_ok)).astype(np.uint8)
    return occupancy, visibility
