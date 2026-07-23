from econ_camera_ros import calib_convert as cvt


def _cam(model, intr, topic, t=None):
    c = {"camera_model": model, "intrinsics": list(intr),
         "distortion_model": "none", "distortion_coeffs": [],
         "resolution": [1280, 720], "rostopic": topic}
    if t is not None:
        c["T_cn_cnm1"] = t
    return c


def test_camchain_to_calib_basic_and_cumulative_extrinsics():
    I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    t01 = [[1, 0, 0, 0.1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    t12 = [[1, 0, 0, 0.2], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    camchain = {
        "cam0": _cam("ds", [0.1, 0.6, 700, 700, 640, 360], "/cam0/image_raw"),
        "cam1": _cam("ds", [0.1, 0.6, 701, 701, 641, 361], "/cam1/image_raw", t01),
        "cam2": _cam("ds", [0.1, 0.6, 702, 702, 642, 362], "/cam2/image_raw", t12),
    }
    out = cvt.camchain_to_calib(camchain, "ds", {"cam0": 0.30, "cam1": 0.35, "cam2": 0.33})

    assert out["model_chosen"] == "ds"
    assert out["cameras"]["cam1"]["intrinsics"] == [0.1, 0.6, 701, 701, 641, 361]
    assert out["cameras"]["cam0"]["camera_model"] == "ds"
    # cam0 = 항등, cam1 = t01, cam2 = t12 @ t01 → x 평행이동 0.3
    assert out["extrinsics"]["T_cam0_cam0"] == I
    assert out["extrinsics"]["T_cam1_cam0"] == t01
    assert out["extrinsics"]["T_cam2_cam0"][0][3] == 0.30000000000000004 or \
           abs(out["extrinsics"]["T_cam2_cam0"][0][3] - 0.3) < 1e-9
    assert out["verification"]["reproj_rms_px"]["cam2"] == 0.33


def test_camchain_to_calib_without_rms():
    camchain = {"cam0": _cam("eucm", [0.6, 1.0, 700, 700, 640, 360], "/cam0/image_raw")}
    out = cvt.camchain_to_calib(camchain, "eucm")
    assert out["verification"]["reproj_rms_px"] == {}
    assert out["extrinsics"]["T_cam0_cam0"][3][3] == 1


def test_camchain_to_calib_extrinsic_order_sensitive():
    # 회전+평행이동 조합이라 곱셈 순서가 뒤집히면 결과가 달라진다(순서 민감 검증).
    rot_z90 = [[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    trans_x = [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    camchain = {
        "cam0": _cam("ds", [0.1, 0.6, 700, 700, 640, 360], "/cam0/image_raw"),
        "cam1": _cam("ds", [0.1, 0.6, 700, 700, 640, 360], "/cam1/image_raw", rot_z90),
        "cam2": _cam("ds", [0.1, 0.6, 700, 700, 640, 360], "/cam2/image_raw", trans_x),
    }
    out = cvt.camchain_to_calib(camchain, "ds")
    # 올바른 순서 T_cam2_cam0 = trans_x @ rot_z90 → [0][3]==1. 뒤집힌 순서면 [0][3]==0.
    assert out["extrinsics"]["T_cam2_cam0"] == [[0, -1, 0, 1], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def test_camchain_to_calib_relabel_by_rostopic():
    I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    t01 = [[1, 0, 0, 0.1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    # 링 순서로 녹화된 camchain: Kalibr cam0=front 토픽(/cam2), cam1=right 토픽(/cam1).
    camchain = {
        "cam0": _cam("ds", [0.1, 0.6, 700, 700, 640, 360], "/cam2/image_raw"),
        "cam1": _cam("ds", [0.1, 0.6, 701, 701, 641, 361], "/cam1/image_raw", t01),
    }
    topic_to_dir = {"/cam2/image_raw": "front", "/cam1/image_raw": "right"}
    out = cvt.camchain_to_calib(camchain, "ds", None, topic_to_dir)
    assert set(out["cameras"]) == {"front", "right"}
    assert out["extrinsics"]["T_front_front"] == I    # 기준 = cams[0]=cam0=front
    assert out["extrinsics"]["T_right_front"] == t01


def test_camchain_to_calib_labels_by_rostopic_not_position():
    # 순서와 방향이 어긋난 camchain: Kalibr cam0 의 rostopic 이 /cam1(right), cam1 이 /cam2(front).
    # 위치 기반이면 cam0→front 로 잘못 라벨하지만, rostopic 기반이면 cam0→right 로 올바르게 라벨.
    t01 = [[1, 0, 0, 0.1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    camchain = {
        "cam0": _cam("ds", [0.1, 0.6, 700, 700, 640, 360], "/cam1/image_raw"),
        "cam1": _cam("ds", [0.1, 0.6, 701, 701, 641, 361], "/cam2/image_raw", t01),
    }
    topic_to_dir = {"/cam1/image_raw": "right", "/cam2/image_raw": "front"}
    out = cvt.camchain_to_calib(camchain, "ds", None, topic_to_dir)
    assert set(out["cameras"]) == {"right", "front"}
    assert "T_right_right" in out["extrinsics"]   # 기준 = cams[0]=cam0=right
    assert "T_front_right" in out["extrinsics"]
