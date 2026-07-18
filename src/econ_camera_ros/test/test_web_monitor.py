from econ_camera_ros import web_monitor_node as wm


def test_index_html_has_all_streams():
    html = wm.index_html([0, 1, 2, 3])
    for d in (0, 1, 2, 3):
        assert f"/stream/{d}" in html
        assert f"cam{d}" in html
    assert "grid" in html


def test_index_html_has_status_bar():
    html = wm.index_html([0, 1, 2, 3])
    assert "id='status'" in html
    assert "/status" in html          # JS 폴링 대상 엔드포인트


def test_mjpeg_chunk_framing():
    jpeg = b"\xff\xd8\xff\xd9"
    chunk = wm.mjpeg_chunk(jpeg)
    assert chunk.startswith(b"--frame\r\n")
    assert b"Content-Type: image/jpeg" in chunk
    assert b"Content-Length: 4" in chunk
    assert chunk.endswith(jpeg + b"\r\n")
