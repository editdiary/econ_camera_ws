import pytest
from econ_camera_ros import cam_layout as cl


def test_order_from_json_ring_order():
    orientation = {"cam0": "left", "cam1": "right", "cam2": "front", "cam3": "rear"}
    assert cl.order_from_json(orientation) == [(2, "front"), (1, "right"), (3, "rear"), (0, "left")]


def test_order_from_json_identity_mapping():
    # cam0=front..cam3=left 이면 링 순서 인덱스가 [0,1,2,3] 그대로.
    orientation = {"cam0": "front", "cam1": "right", "cam2": "rear", "cam3": "left"}
    assert cl.order_from_json(orientation) == [(0, "front"), (1, "right"), (2, "rear"), (3, "left")]


def test_order_from_json_missing_direction():
    # rear 누락 + left 중복 → 무효.
    orientation = {"cam0": "left", "cam1": "right", "cam2": "front", "cam3": "left"}
    with pytest.raises(ValueError):
        cl.order_from_json(orientation)


def test_order_from_json_unknown_direction():
    orientation = {"cam0": "left", "cam1": "right", "cam2": "front", "cam3": "top"}
    with pytest.raises(ValueError):
        cl.order_from_json(orientation)


def test_order_from_json_wrong_keys():
    # cam3 없음(cam4 오타).
    orientation = {"cam0": "left", "cam1": "right", "cam2": "front", "cam4": "rear"}
    with pytest.raises(ValueError):
        cl.order_from_json(orientation)
