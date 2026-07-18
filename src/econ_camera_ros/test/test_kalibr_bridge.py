from econ_camera_ros import kalibr_bridge as kb


def test_select_by_rate_downsamples_30hz_to_4hz():
    # 30fps 30프레임(0..0.9667s) → 4Hz(0.25s 간격)면 0,0.25,0.5,0.75초 부근 = 4개
    stamps = [i / 30.0 for i in range(30)]
    kept = kb.select_by_rate(stamps, target_hz=4.0)
    assert kept[0] == 0
    # 인접 유지 세트 간 실제 간격이 0.25s 이상
    for a, b in zip(kept, kept[1:]):
        assert stamps[b] - stamps[a] >= 0.25 - 1e-9
    assert len(kept) == 4


def test_select_by_rate_empty():
    assert kb.select_by_rate([], 4.0) == []


def test_stamp_to_ns_rounds():
    assert kb.stamp_to_ns(1.000000001) == 1000000001
    assert kb.stamp_to_ns(0.0) == 0


def test_parse_set_stamps_reads_first_stamp_column():
    csv_text = (
        "idx,stamp0,stamp1,stamp2,stamp3,spread_ms\n"
        "0,100.000000000,100.000100000,100.000200000,100.000050000,0.200\n"
        "1,100.033000000,100.033100000,100.033200000,100.033050000,0.200\n"
    )
    assert kb.parse_set_stamps(csv_text) == [100.0, 100.033]
