"""슬래브 라벨 순수 로직 테스트. 하드웨어·파일 불필요."""
import pathlib
import sys

import numpy as np
import pytest

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
sys.path.insert(0, str(_HERE.parent.parent / "mapping"))
from bev_label import BevSpec                                   # noqa: E402
import slab_label as sl                                         # noqa: E402
from pcd_denoise import read_pcd_raw  # noqa: E402


def test_crop_mask_keeps_inside_box():
    spec = BevSpec(XF=4.0, XR=2.0, YH=3.0)
    P = np.array([
        [0.0, 0.0, 0.0],       # 중앙 → 포함
        [4.0, 0.0, 99.0],      # x 상한 경계 → 포함 (z 는 무제한)
        [-2.0, 0.0, -99.0],    # x 하한 경계 → 포함
        [0.0, 3.0, 0.0],       # y 상한 경계 → 포함
        [0.0, -3.0, 0.0],      # y 하한 경계 → 포함
        [4.01, 0.0, 0.0],      # x 초과 → 제외
        [-2.01, 0.0, 0.0],     # x 미달 → 제외
        [0.0, 3.01, 0.0],      # y 초과 → 제외
    ])
    m = sl.crop_mask(P, spec)
    assert m.tolist() == [True, True, True, True, True, False, False, False]


def test_crop_mask_ignores_z_entirely():
    spec = BevSpec(XF=4.0, XR=2.0, YH=3.0)
    P = np.array([[1.0, 1.0, -1000.0], [1.0, 1.0, 1000.0]])
    assert sl.crop_mask(P, spec).all()


def test_crop_mask_empty_input():
    spec = BevSpec(XF=4.0, XR=2.0, YH=3.0)
    m = sl.crop_mask(np.empty((0, 3)), spec)
    assert m.shape == (0,) and m.dtype == bool


def test_ref_z_is_low_percentile_not_min():
    z = np.concatenate([[-5.0], np.linspace(0.0, 1.0, 100)])   # -5 는 이상치 1개
    r = sl.ref_z(z, pct=1.0)
    assert -5.0 < r < 0.05, r          # 이상치에 끌려가지 않는다


def test_ref_z_pct_zero_is_min():
    z = np.array([3.0, 1.0, 2.0])
    assert sl.ref_z(z, pct=0.0) == pytest.approx(1.0)


def test_slab_mask_band_boundaries_inclusive():
    P = np.array([
        [0.0, 0.0, 0.999],   # z_ref 바로 아래 → 제외
        [0.0, 0.0, 1.0],     # z_ref → 포함
        [0.0, 0.0, 1.8],     # z_ref+thick → 포함
        [0.0, 0.0, 1.801],   # 초과 → 제외
    ])
    m = sl.slab_mask(P, z_ref=1.0, thick=0.8)
    assert m.tolist() == [False, True, True, False]


def test_slab_mask_empty_input():
    m = sl.slab_mask(np.empty((0, 3)), z_ref=0.0, thick=0.8)
    assert m.shape == (0,) and m.dtype == bool


def test_occupancy_counts_bins_by_row_col():
    spec = BevSpec(XF=1.0, XR=0.0, YH=0.5)      # NX=20, NY=20
    # x=0.975,y=0.475 → row=floor((1.0-0.975)/0.05)=0, col=floor((0.5-0.475)/0.05)=0
    P = np.array([[0.975, 0.475, 0.0]] * 3 + [[0.025, -0.475, 0.0]])
    cnt = sl.occupancy_counts(P, spec)
    assert cnt.shape == (spec.NX, spec.NY)
    assert cnt[0, 0] == 3
    assert cnt[19, 19] == 1
    assert cnt.sum() == 4


def test_occupancy_counts_drops_out_of_range():
    spec = BevSpec(XF=1.0, XR=0.0, YH=0.5)
    P = np.array([[5.0, 0.0, 0.0], [0.5, 5.0, 0.0]])
    assert sl.occupancy_counts(P, spec).sum() == 0


def test_occupancy_counts_agrees_with_rc_of():
    from bev_label import rc_of
    spec = BevSpec(XF=4.0, XR=2.0, YH=3.0)
    rng = np.random.default_rng(0)
    P = np.column_stack([rng.uniform(-2, 4, 500), rng.uniform(-3, 3, 500),
                         np.zeros(500)])
    cnt = sl.occupancy_counts(P, spec)
    r, c = rc_of(P[:, 0], P[:, 1], spec)
    ok = (r >= 0) & (r < spec.NX) & (c >= 0) & (c < spec.NY)
    assert cnt.sum() == int(ok.sum())
    assert cnt[r[ok][0], c[ok][0]] >= 1


def test_obstacle_threshold_is_min_pts():
    counts = np.array([[0, 1, 2], [3, 4, 10]])
    obs = sl.obstacle_from_counts(counts, min_pts=3)
    assert obs.tolist() == [[False, False, False], [True, True, True]]


def test_raycast_empty_grid_is_all_visible():
    from bev_label import raycast_visible
    spec = BevSpec(XF=1.0, XR=1.0, YH=1.0)      # 40x40
    vis = raycast_visible(np.zeros((spec.NX, spec.NY), bool), spec, step_deg=0.25)
    assert vis.mean() > 0.95


def test_raycast_wall_blocks_cells_behind_it():
    from bev_label import raycast_visible
    spec = BevSpec(XF=1.0, XR=1.0, YH=1.0)      # NX=NY=40, ego=(20,20)
    obs = np.zeros((spec.NX, spec.NY), bool)
    obs[10, :] = True                            # ego 전방 0.5m 에 가로 벽
    vis = raycast_visible(obs, spec, step_deg=0.25)
    assert vis[10, spec.C_EGO]                   # 벽 표면은 보인다
    assert not vis[:10, :].any()                 # 벽 뒤는 전부 미관측


def _fake_cam(width=100, height=80):
    """z>0 이면 (x,y) 를 그대로 픽셀로 쓰는 최소 카메라 스텁."""
    class Cam:
        def __init__(self):
            self.width, self.height = width, height

        def project(self, P):
            P = np.asarray(P, float)
            u = P[..., 0] + width / 2.0
            v = P[..., 1] + height / 2.0
            return u, v, P[..., 2] > 0
    return Cam()


def test_vignette_mask_finds_always_dark_pixels():
    F, H, W = 20, 40, 60
    frames = np.zeros((F, H, W), np.uint8)
    frames[:, 10:30, 15:45] = 200                 # 항상 밝은 유효원
    frames[3, 0, 0] = 255                         # 한 프레임만 튄 화소
    m = sl.vignette_mask(frames)
    assert m[0, 0]                                # 이상치 1개로는 유효가 되지 않는다
                                                  # (quantile=0.98 이면 여기서 실패한다)
    assert not m[20, 30]                          # 밝은 영역은 유효
    assert m.mean() > 0.5


def test_vignette_mask_all_bright_is_all_valid():
    frames = np.full((5, 20, 20), 180, np.uint8)
    assert not sl.vignette_mask(frames).any()


def test_camera_observable_rejects_masked_pixels():
    spec = BevSpec(XF=0.5, XR=0.5, YH=0.5)        # 20x20
    cam = _fake_cam()
    cams = {"front": cam}
    T = np.eye(4)
    # z_body=1.0 → 스텁의 z>0 조건 만족, u=x+50, v=y+40 → 전부 이미지 안
    free = sl.camera_observable(spec, 1.0, cams, {"front": T}, T, {}, ("front",))
    assert free.all()
    blocked = np.ones((cam.height, cam.width), bool)
    none = sl.camera_observable(spec, 1.0, cams, {"front": T}, T,
                                {"front": blocked}, ("front",))
    assert not none.any()


def test_camera_observable_ors_over_cameras():
    spec = BevSpec(XF=0.5, XR=0.5, YH=0.5)
    cam = _fake_cam()
    T = np.eye(4)
    blocked = np.ones((cam.height, cam.width), bool)
    got = sl.camera_observable(spec, 1.0, {"a": cam, "b": cam},
                               {"a": T, "b": T}, T,
                               {"a": blocked}, ("a", "b"))
    assert got.all()                              # b 가 보므로 OR 결과는 전부 True


def test_camera_observable_rejects_behind_camera():
    spec = BevSpec(XF=0.5, XR=0.5, YH=0.5)
    cam = _fake_cam()
    T = np.eye(4)
    # z_body=-1.0 → 스텁의 z>0 실패 → 전부 무효
    got = sl.camera_observable(spec, -1.0, {"front": cam}, {"front": T}, T,
                               {}, ("front",))
    assert not got.any()


def test_assemble_encodes_project_convention():
    obstacle = np.array([[True, False]])
    visible = np.array([[True, True]])
    camera_ok = np.array([[True, False]])
    occ, vis = sl.assemble(obstacle, visible, camera_ok)
    assert occ.tolist() == [[0, 1]]               # 0=obstacle, 1=drivable
    assert vis.tolist() == [[1, 0]]               # camera_ok=False → unseen
    assert occ.dtype == np.uint8 and vis.dtype == np.uint8


def _write_min_pcd(path, xyz, inten):
    """테스트용 최소 8필드 binary PCD 작성."""
    import struct
    n = len(xyz)
    head = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
        "SIZE 4 4 4 4 4 4 4 4\nTYPE F F F F F F F F\n"
        "COUNT 1 1 1 1 1 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(head.encode("ascii"))
        for i in range(n):
            f.write(struct.pack("<8f", xyz[i, 0], xyz[i, 1], xyz[i, 2],
                                inten[i], 0.0, 0.0, 0.0, 0.0))


def test_write_points_preserves_intensity_and_overwrites_xyz(tmp_path):
    import slab_io
    src = tmp_path / "map_clean.pcd"
    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
    _write_min_pcd(src, xyz, np.array([11.0, 22.0, 33.0]))
    head, arr = read_pcd_raw(str(src))
    out = tmp_path / "slab.pcd"
    idx = np.array([0, 2])
    body = np.array([[-1.0, -2.0, -3.0], [-7.0, -8.0, -9.0]])
    slab_io.write_points(str(out), head, arr, idx, body)
    _, got = read_pcd_raw(str(out))
    assert len(got) == 2
    assert got["intensity"].tolist() == [11.0, 33.0]        # 보존
    assert got["x"].tolist() == [-1.0, -7.0]                # body 좌표로 교체
    assert got["z"].tolist() == [-3.0, -9.0]


def test_load_self_masks_missing_dir_is_empty(tmp_path):
    import slab_io
    assert slab_io.load_self_masks(str(tmp_path / "nope")) == {}


_LABELMAP = ("# label:color_rgb:parts:actions\n"
             "background:0,0,0::\n"
             "handle:61,245,61::\n"
             "human:140,120,240::\n"
             "table:250,50,83::\n")


def _write_class_mask(path):
    """실측 마스크와 같은 클래스 색으로 3줄짜리 마스크를 만든다(BGR 로 씀)."""
    import cv2
    img = np.zeros((10, 20, 3), np.uint8)        # background=(0,0,0)
    img[0:3] = (83, 50, 250)                     # table  (RGB 250,50,83)
    img[3:5] = (240, 120, 140)                   # human  (RGB 140,120,240)
    img[5:6] = (61, 245, 61)                     # handle (RGB 61,245,61)
    cv2.imwrite(str(path), img)


def test_load_labelmap_excludes_background(tmp_path):
    import slab_io
    (tmp_path / "labelmap.txt").write_text(_LABELMAP)
    assert slab_io.load_labelmap(str(tmp_path)) == {
        "handle": (61, 245, 61), "human": (140, 120, 240), "table": (250, 50, 83)}


def test_load_self_masks_default_reads_table_only(tmp_path):
    import slab_io
    (tmp_path / "labelmap.txt").write_text(_LABELMAP)
    _write_class_mask(tmp_path / "mask_front.png")
    m = slab_io.load_self_masks(str(tmp_path), use_names=("front",))
    assert set(m) == {"front"}
    assert m["front"][:3].all()                  # table 만 무효
    assert not m["front"][3:6].any()             # human·handle 은 self 박스가 덮는다
    assert not m["front"][6:].any()              # background 는 유효


def test_load_self_masks_can_select_more_classes(tmp_path):
    import slab_io
    (tmp_path / "labelmap.txt").write_text(_LABELMAP)
    _write_class_mask(tmp_path / "mask_front.png")
    m = slab_io.load_self_masks(str(tmp_path), use_names=("front",),
                                classes=("table", "human", "handle"))
    assert m["front"][:6].all()
    assert not m["front"][6:].any()


def test_load_self_masks_skips_missing_file(tmp_path):
    import slab_io
    (tmp_path / "labelmap.txt").write_text(_LABELMAP)
    assert slab_io.load_self_masks(str(tmp_path), use_names=("front",)) == {}


def test_load_self_masks_rejects_unknown_class(tmp_path):
    import slab_io
    (tmp_path / "labelmap.txt").write_text(_LABELMAP)
    _write_class_mask(tmp_path / "mask_front.png")
    with pytest.raises(ValueError):
        slab_io.load_self_masks(str(tmp_path), use_names=("front",),
                                classes=("nope",))


def test_self_box_mask_bounds():
    from bev_label import cell_centers
    spec = BevSpec(XF=1.0, XR=1.0, YH=1.0)          # NX=NY=40
    m = sl.self_box_mask(spec, near=0.2, far=0.6, yh=0.1)
    X, Y = cell_centers(spec)
    assert not m[X > -0.2].any()                     # near 보다 가까우면 제외
    assert not m[X < -0.6].any()                     # far 보다 멀면 제외
    assert not m[np.abs(Y) > 0.1].any()
    assert m.sum() == 8 * 4                          # x 0.4m→8셀, |y|<=0.1→4셀


def test_self_box_default_contains_pointlio_box():
    from bev_label import cell_centers
    spec = BevSpec(XF=4.0, XR=2.0, YH=3.0)           # 120x120
    m = sl.self_box_mask(spec)
    X, Y = cell_centers(spec)
    pointlio = (X >= -1.5) & (X <= -0.45) & (np.abs(Y) < 0.35)
    assert not (pointlio & ~m).any()                 # Point-LIO 잔재 영역을 전부 담는다
    assert m.sum() == 896                            # 실측으로 정한 기본 박스 크기
