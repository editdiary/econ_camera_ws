"""render 순수 로직 테스트(라벨 오버레이/검수뷰). 하드웨어 불필요."""
import os
import sys

import numpy as np

BA = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BA)

from bev_label import BevSpec                         # noqa: E402
import render                                          # noqa: E402


def _ipm(spec, val=100):
    return np.full((spec.NX, spec.NY, 3), val, np.uint8)


def test_blend_label_native_res_tint_only():
    spec = BevSpec()
    ipm = _ipm(spec, 100)
    label = np.full((spec.NX, spec.NY), 2, np.uint8)   # 전부 ignore
    label[13, 55] = 0                                  # obstacle → 빨강(BGR[2])
    label[23, 53] = 1                                  # drivable → 초록(BGR[1])
    over = render.blend_label(ipm, label)
    assert over.shape == ipm.shape                     # 네이티브 해상도 유지(리사이즈 없음)
    assert tuple(over[0, 0]) == (100, 100, 100)        # ignore 셀 = 배경 그대로(장식 없음)
    assert over[13, 55, 2] > 100 and over[23, 53, 1] > 100


def test_label_overlay_shape_and_no_camera_row():
    spec = BevSpec(); scale = 9
    label = np.full((spec.NX, spec.NY), 2, np.uint8)   # 전부 ignore
    over = render.label_overlay(_ipm(spec), label, spec, scale=scale)
    assert over.shape == (spec.NX * scale, spec.NY * scale, 3)
    assert over.dtype == np.uint8


def test_label_overlay_tints_obstacle_and_drivable_only():
    spec = BevSpec()
    ipm = _ipm(spec, 100)
    label = np.full((spec.NX, spec.NY), 2, np.uint8)
    # 0.5m 격자선(10칸 배수)·좌측 축 텍스트를 피한 셀 선택
    label[13, 55] = 0                                  # obstacle → 빨강 쪽으로
    label[23, 53] = 1                                  # drivable → 초록 쪽으로
    over = render.label_overlay(ipm, label, spec, alpha=0.45, scale=1)
    # ignore 셀은 배경 그대로(100,100,100)
    assert tuple(over[13, 53]) == (100, 100, 100)
    # obstacle 셀은 R(BGR[2])이 배경보다 커짐, drivable 셀은 G(BGR[1])가 커짐
    assert over[13, 55, 2] > 100
    assert over[23, 53, 1] > 100


def test_grid_lines_anchored_to_metric_grid():
    """0.5m 격자선은 미터좌표에 고정 — XF/YH 가 0.5 배수가 아니어도 x=0·y=0 에 선이 온다.

    격자선이 격자 좌상단 기준이면 미터축 텍스트(미터 계산)와 선이 어긋나 검수자가
    거리를 잘못 읽는다.
    """
    spec = BevSpec(XF=3.2, XR=1.0, YH=2.2)             # 0.5 배수가 아닌 범위
    label = np.full((spec.NX, spec.NY), 2, np.uint8)   # 전부 ignore → 배경 그대로
    over = render.label_overlay(_ipm(spec, 100), label, spec, scale=1)
    r0 = int(round(spec.XF / spec.RES))                # x=0 인 행 = 64
    c0 = int(round(spec.YH / spec.RES))                # y=0 인 열 = 44
    assert tuple(over[r0, 25]) == (60, 60, 60)         # x=0 에 가로 격자선
    assert tuple(over[25, c0]) == (60, 60, 60)         # y=0 에 세로 격자선
    assert tuple(over[60, 25]) == (100, 100, 100)      # 격자 좌상단 기준 선(row 60)은 없음


def test_review_overlay_stacks_camera_row_on_top():
    spec = BevSpec(); scale = 9
    label = np.full((spec.NX, spec.NY), 2, np.uint8)
    ipm = _ipm(spec)
    bev = render.label_overlay(ipm, label, spec, scale=scale)
    cams = {"front": np.zeros((720, 1280, 3), np.uint8)}
    full = render.review_overlay(ipm, label, cams, spec, scale=scale)
    assert full.shape[1] == bev.shape[1]               # 폭 동일
    assert full.shape[0] > bev.shape[0]                # 카메라 행만큼 더 높음
    # 카메라 없으면 label_overlay 와 동일
    same = render.review_overlay(ipm, label, {}, spec, scale=scale)
    assert np.array_equal(same, bev)
