from econ_camera_ros import bag_extract


def test_group_perfect_sync():
    # 4대, 3프레임, stamp 편차 sub-ms(0.1ms) — 3세트 전부 4대 채워짐
    frames = {d: [(i * 0.0333 + d * 0.0001, f"c{d}f{i}") for i in range(3)]
              for d in range(4)}
    sets = bag_extract.group_synchronized(frames, tol=0.010)
    assert len(sets) == 3
    for s in sets:
        assert set(s) == {0, 1, 2, 3}


def test_group_drops_incomplete_sets():
    # cam2 가 중간 프레임을 드롭 → 그 순간 세트는 제외(4대 못 채움)
    frames = {
        0: [(0.0, "a0"), (0.033, "a1"), (0.066, "a2")],
        1: [(0.0, "b0"), (0.033, "b1"), (0.066, "b2")],
        2: [(0.0, "c0"), (0.066, "c2")],
        3: [(0.0, "d0"), (0.033, "d1"), (0.066, "d2")],
    }
    sets = bag_extract.group_synchronized(frames, tol=0.010)
    assert len(sets) == 2
    assert [round(s[0][0], 3) for s in sets] == [0.0, 0.066]


def test_group_no_frame_reuse_when_anchors_close():
    # 워밍업 지터: cam0 anchor 두 개가 7ms 간격(< 2*tol). cam1~3 은 그 사이 프레임이
    # 하나뿐 → 첫 anchor 만 세트 성립, 그 프레임이 재사용되면 안 됨(두 번째는 드롭).
    frames = {
        0: [(0.938, "a0"), (0.945, "a1")],
        1: [(0.941, "b0")],
        2: [(0.941, "c0")],
        3: [(0.940, "d0")],
    }
    sets = bag_extract.group_synchronized(frames, tol=0.010)
    assert len(sets) == 1
    assert sets[0][1] == (0.941, "b0")


def test_group_beyond_tolerance_excluded():
    # cam1 프레임이 20ms 어긋남(> tol 10ms) → 세트 미완성 → 제외
    frames = {
        0: [(0.0, "a0")],
        1: [(0.020, "b0")],
        2: [(0.0, "c0")],
        3: [(0.0, "d0")],
    }
    sets = bag_extract.group_synchronized(frames, tol=0.010)
    assert sets == []
