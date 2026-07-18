from econ_camera_ros import gst_builder


def test_one_branch_per_device():
    desc = gst_builder.capture_pipeline([0, 1, 2, 3], 1280, 720)
    assert desc.count("v4l2src") == 4
    for d in (0, 1, 2, 3):
        assert f"device=/dev/video{d}" in desc
        assert f"appsink name=sink{d}" in desc


def test_caps_and_encoder():
    desc = gst_builder.capture_pipeline([0], 1280, 720, jpeg_quality=90)
    assert "format=UYVY,width=1280,height=720" in desc
    assert "nvvidconv" in desc
    assert "nvjpegenc quality=90" in desc
    assert "image/jpeg" in desc


def test_appsink_no_drop_for_continuous_capture():
    desc = gst_builder.capture_pipeline([0], 1280, 720, max_buffers=5)
    assert "max-buffers=5" in desc
    assert "drop=false" in desc
    assert "sync=false" in desc


def test_sink_prefix_override():
    desc = gst_builder.capture_pipeline([2], 640, 480, sink_prefix="cap")
    assert "appsink name=cap2" in desc
