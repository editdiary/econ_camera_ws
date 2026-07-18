"""연속 수집용 GStreamer 파이프라인 문자열 빌더 (순수 함수).

선행 프로젝트(econ_cam.gst_pipeline)의 tee/valve 게이팅 구조와 달리, 연속 수집은
tee·valve 없이 원본을 계속 JPEG로 인코딩해 흘려보낸다. 4개 v4l2src 브랜치를 하나의
문자열로 이어 단일 파이프라인(공유 클럭)으로 실행하면 카메라 간 PTS 비교가 유효하다.

실기 검증된 경로: v4l2src ! video/x-raw,format=UYVY ! nvvidconv ! nvjpegenc ! image/jpeg ! appsink
"""


def _branch(dev, width, height, jpeg_quality, max_buffers, sink_prefix):
    return (
        f"v4l2src device=/dev/video{dev} "
        f"! video/x-raw,format=UYVY,width={width},height={height} "
        f"! nvvidconv ! nvjpegenc quality={jpeg_quality} ! image/jpeg "
        f"! appsink name={sink_prefix}{dev} max-buffers={max_buffers} drop=false sync=false"
    )


def capture_pipeline(devs, width, height, jpeg_quality=90, max_buffers=5,
                     sink_prefix="sink"):
    """N개 v4l2src 연속 캡처 브랜치를 단일 공유클럭 파이프라인 문자열로 반환.

    appsink 이름은 f"{sink_prefix}{dev}". drop=false 로 무단 드롭을 피한다(소비자가
    new-sample 콜백에서 즉시 pull).
    """
    return "   ".join(
        _branch(d, width, height, jpeg_quality, max_buffers, sink_prefix)
        for d in devs
    )
