"""순수 로직 테스트(ROS·이미지·bag·calib 파일 불필요). 실행: cd calibration/bev_autolabel && python3 -m pytest -q"""
import numpy as np
from bev_label import BevSpec, rc_of, cell_centers, _key3
from bev_label import floor_grid, obstacle_mask

import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent.parent / "verify"))
from ds_model import DoubleSphereCamera
from bev_label import fov_mask, raycast_visible


def test_bevspec_derived_shapes():
    s = BevSpec()
    assert (s.NX, s.NY) == (80, 80)
    assert (s.R_EGO, s.C_EGO) == (60, 40)


def test_rc_of_ego_origin():
    s = BevSpec()
    r, c = rc_of(0.0, 0.0, s)          # ego 원점(x=0,y=0)
    assert (int(r), int(c)) == (60, 40)


def test_rc_of_forward_is_up():
    s = BevSpec()
    r_fwd, _ = rc_of(2.0, 0.0, s)      # 전방 2m → row 작아짐(위)
    r_ego, _ = rc_of(0.0, 0.0, s)
    assert r_fwd < r_ego


def test_cell_centers_roundtrip():
    s = BevSpec()
    X, Y = cell_centers(s)
    r, c = rc_of(X, Y, s)
    rr, cc = np.meshgrid(np.arange(s.NX), np.arange(s.NY), indexing="ij")
    assert np.array_equal(r, rr) and np.array_equal(c, cc)


def test_key3_unique():
    C = np.array([[1, 2, 3], [1, 2, 3], [1, 2, 4]])
    k = _key3(C)
    assert k[0] == k[1] and k[0] != k[2]


def _column(x, y, z0, z1, n=40):
    z = np.linspace(z0, z1, n)
    return np.stack([np.full(n, x), np.full(n, y), z], -1)


def test_floor_grid_flat_ground():
    s = BevSpec()
    # z=0 평면에 흩뿌린 점
    xy = np.random.RandomState(0).uniform(-1, 2, (500, 2))
    P = np.hstack([xy, np.zeros((500, 1))])
    f = floor_grid(P, s)
    assert f.shape == (s.NX, s.NY)
    assert abs(np.median(f)) < 0.05


def test_obstacle_column_detected():
    s = BevSpec()
    floor = np.zeros((s.NX, s.NY))
    # (1.0, 0.0)이 정확히 셀 경계(부동소수 오차로 인접 셀과 겹침)라 1e-6만큼 살짝 비켜
    # 3x3 블록이 인접 3개 행/열에 고르게 퍼지도록 함(그래야 OPEN 3x3에서 안 지워짐).
    cx, cy = 1.0 - 1e-6, 0.0 - 1e-6
    cols = [_column(cx + dx, cy + dy, 0.05, 1.5)
            for dx in (-0.05, 0, 0.05) for dy in (-0.05, 0, 0.05)]
    P = np.vstack(cols)
    obs = obstacle_mask(P, floor, s)
    r, c = rc_of(1.0, 0.0, s)
    assert obs[int(r), int(c)]


def test_ceiling_only_not_obstacle():
    s = BevSpec()
    floor = np.zeros((s.NX, s.NY))
    # 높이 2.0~2.3m 에만 떠 있는 천장 클러스터(바닥까지 안 이어짐)
    P = np.repeat(_column(0.5, 1.0, 2.0, 2.3, n=10), 1, axis=0)
    obs = obstacle_mask(P, floor, s)
    r, c = rc_of(0.5, 1.0, s)
    assert not obs[int(r), int(c)]


def test_floor_noise_speck_not_obstacle():
    s = BevSpec()
    floor = np.zeros((s.NX, s.NY))
    # 바닥 근처 단발 점 몇 개(수직 extent 없음)
    P = np.array([[0.5, 0.5, 0.02], [0.5, 0.5, 0.05], [0.5, 0.5, 0.08]])
    obs = obstacle_mask(P, floor, s)
    r, c = rc_of(0.5, 0.5, s)
    assert not obs[int(r), int(c)]


def _front_cam():
    # 전방을 +z로 보는 표준 핀홀 유사 DS(alpha=0.5, xi=0): z>0 만 유효
    return DoubleSphereCamera(xi=0.0, alpha=0.5, fx=300, fy=300, cx=640, cy=360,
                              width=1280, height=720, name="front")


def test_fov_forward_visible_backward_not():
    s = BevSpec()
    floor = np.zeros((s.NX, s.NY))
    cams = {"front": _front_cam()}
    # front 카메라: LiDAR +x(전방)를 카메라 +z로 보내는 회전 T_cam_front
    Tcf = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]], float)
    fov = fov_mask(floor, s, cams, {"front": Tcf}, np.eye(4), use_names=("front",))
    r_f, c_f = rc_of(2.0, 0.0, s)     # 전방 2m → 보여야
    r_b, c_b = rc_of(-0.5, 0.0, s)    # 후방 → 안 보여야(카메라 뒤)
    assert fov[int(r_f), int(c_f)]
    assert not fov[int(r_b), int(c_b)]


def test_raycast_blocks_behind_wall():
    s = BevSpec()
    obs = np.zeros((s.NX, s.NY), bool)
    # ego 앞(전방 1m) 가로벽
    rw, _ = rc_of(1.0, 0.0, s)
    obs[int(rw), :] = True
    vis = raycast_visible(obs, s)
    r_near, c_near = rc_of(0.5, 0.0, s)   # 벽 앞: 보임
    r_far, c_far = rc_of(2.0, 0.0, s)     # 벽 뒤: 가려짐
    assert vis[int(r_near), int(c_near)]
    assert not vis[int(r_far), int(c_far)]


from bev_label import compute_self_voxels, corridor_mask, keep_ego_connected, assemble_label


def test_self_voxels_catches_close_not_far():
    # 모든 pose에서 body 원점 근처(0.1m)에 카트 점, 0.7m 옆에 기둥 점.
    # pose는 x축으로 전진(회전 없음).
    poses = []
    for i in range(20):
        T = np.eye(4); T[0, 3] = i * 0.3
        poses.append(T)
    cart = np.array([[i * 0.3 + 0.1, 0.0, 0.0] for i in range(20)])   # body +0.1m 지속
    pillar = np.array([[i * 0.3, 0.7, 0.0] for i in range(20)])       # body +0.7m 지속
    p = np.vstack([cart, pillar])
    SELF = compute_self_voxels(p, poses, radius=0.28)
    # 카트(0.1m)는 SELF, 기둥(0.7m)은 반경 밖이라 아님
    from bev_label import _key3
    cart_key = _key3(np.floor(np.array([[0.1, 0.0, 0.0]]) / 0.15).astype(int))[0]
    pillar_key = _key3(np.floor(np.array([[0.0, 0.7, 0.0]]) / 0.15).astype(int))[0]
    assert cart_key in set(SELF.tolist())
    assert pillar_key not in set(SELF.tolist())


def test_corridor_forward_only():
    s = BevSpec()
    TE = np.array([[1.0, 0.0, 0.0], [-0.5, 0.0, 0.0]])   # 전방 1m, 후방 0.5m
    corr = corridor_mask(TE, s)
    r_f, c_f = rc_of(1.0, 0.0, s)
    r_b, c_b = rc_of(-0.5, 0.0, s)
    assert corr[int(r_f), int(c_f)]        # 전방 마킹
    assert not corr[int(r_b), int(c_b)]    # 후방 미마킹(단, ego 원과 안 겹치는 위치)


def test_keep_ego_connected_drops_island():
    s = BevSpec()
    d = np.zeros((s.NX, s.NY), bool)
    d[s.R_EGO, s.C_EGO] = True
    d[s.R_EGO - 1, s.C_EGO] = True         # ego 연결
    d[0, 0] = True                         # 떨어진 섬
    out = keep_ego_connected(d, s)
    assert out[s.R_EGO, s.C_EGO] and not out[0, 0]


def test_assemble_label_values():
    s = BevSpec()
    observed = np.zeros((s.NX, s.NY), bool)
    observed[s.R_EGO - 5:s.R_EGO + 1, s.C_EGO] = True   # ego~전방 한 줄 관측
    obs_rc = np.zeros((s.NX, s.NY), bool)
    obs_rc[s.R_EGO - 5, s.C_EGO] = True                 # 관측 줄 끝에 장애물
    corridor = np.zeros((s.NX, s.NY), bool)
    lab = assemble_label(observed, obs_rc, corridor, s)
    assert lab[s.R_EGO - 5, s.C_EGO] == 0               # obstacle
    assert lab[s.R_EGO - 1, s.C_EGO] == 1               # drivable
    assert lab[0, 0] == 2                               # 미관측 ignore
