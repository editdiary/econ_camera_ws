"""웹 기반 온디맨드 모니터 (구독 전용, 녹화와 격리).

4개 CompressedImage(JPEG) 토픽을 구독해 카메라별 '최신 JPEG 바이트'만 보관하고,
표준 라이브러리 http.server 로 2x2 HTML + 카메라별 MJPEG 스트림을 제공한다.
발행 페이로드가 이미 JPEG라 디코드/재인코딩 없이 그대로 흘려보낸다(가볍고 격리적).
cv2/GTK 불필요 → 헤드리스/SSH 에서 브라우저로 확인.

상태 라인: 카메라별 수신 fps(모니터 관측)와 동기 spread/std(각 프레임 header.stamp
기반, capture 로그와 동일한 econ_cam.stats 지표)를 /status JSON 으로 제공하고, 페이지
상단 상태바가 1초마다 폴링해 갱신한다.

실행: ros2 run econ_camera_ros monitor  [--ros-args -p port:=10010]
접속: http://<Orin-IP>:10010
      (SSH만 열려 있으면  ssh -L 10010:localhost:10010 ...  후 브라우저 localhost:10010)
"""

import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from econ_cam import stats  # 동기 지표 재사용(capture_node 와 동일)

_BOUNDARY = "frame"
_FPS_WINDOW = 2.0   # 수신 fps 측정 창(초)
_STAMP_RING = 10    # 동기 계산용 카메라별 최근 stamp 개수


def index_html(devs):
    """2x2 그리드 + 상단 상태바 HTML(순수 함수). 각 셀은 /stream/{dev} MJPEG."""
    cells = "\n".join(
        f'<figure><figcaption>cam{d}</figcaption>'
        f'<img src="/stream/{d}" alt="cam{d}"></figure>'
        for d in devs
    )
    return (
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<title>econ camera monitor</title><style>"
        "body{margin:0;background:#111;font-family:sans-serif}"
        "#status{position:sticky;top:0;background:#000;color:#0f0;"
        "font-family:monospace;font-size:14px;padding:6px 10px;"
        "border-bottom:1px solid #333;white-space:nowrap;overflow-x:auto}"
        ".grid{display:grid;grid-template-columns:1fr 1fr;gap:4px}"
        "figure{margin:0}img{width:100%;display:block;background:#000}"
        "figcaption{color:#0f0;padding:2px 6px;font-size:14px}"
        "</style></head><body>"
        "<div id='status'>connecting…</div>"
        f"<div class='grid'>{cells}</div>"
        "<script>"
        "async function poll(){"
        "try{const r=await fetch('/status');const s=await r.json();"
        "const f=s.devices.map(d=>`cam${d.dev} ${d.fps.toFixed(1)}`).join('  ');"
        "const y=s.sync?`sync spread ${s.sync.spread_ms.toFixed(2)}ms "
        "std ${s.sync.std_ms.toFixed(2)}ms`:'sync n/a';"
        "document.getElementById('status').textContent="
        "`${f} fps   ·   ${y}`;}"
        "catch(e){document.getElementById('status').textContent='status error';}}"
        "setInterval(poll,1000);poll();"
        "</script></body></html>"
    )


def mjpeg_chunk(jpeg):
    """MJPEG multipart/x-mixed-replace 프레임 한 조각(순수 함수)."""
    return (b"--" + _BOUNDARY.encode() + b"\r\n"
            + b"Content-Type: image/jpeg\r\n"
            + b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
            + jpeg + b"\r\n")


class WebMonitorNode(Node):
    def __init__(self):
        super().__init__("econ_camera_web_monitor")
        self.declare_parameter("devices", [0, 1, 2, 3])
        self.declare_parameter("port", 10010)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("fps", 15)
        self.devs = list(self.get_parameter("devices").value)
        port = self.get_parameter("port").value
        host = self.get_parameter("host").value
        self.fps = max(1, self.get_parameter("fps").value)

        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self._latest = {}                                  # dev -> 최신 JPEG bytes
        self._recv = {d: deque() for d in self.devs}       # dev -> 최근 수신 monotonic 시각
        self._stamp_ring = {d: deque(maxlen=_STAMP_RING)   # dev -> 최근 header.stamp(초)
                            for d in self.devs}
        self._lock = threading.Lock()
        for d in self.devs:
            self.create_subscription(
                CompressedImage, f"/camera{d}/image_raw/compressed",
                lambda msg, dev=d: self._on_image(msg, dev), qos)

        self._httpd = ThreadingHTTPServer((host, port), self._make_handler())
        self._srv_thread = threading.Thread(
            target=self._httpd.serve_forever, daemon=True)
        self._srv_thread.start()
        self.get_logger().info(
            f"web monitor: http://{host}:{port}  devices={self.devs} fps={self.fps}")

    def _on_image(self, msg, dev):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._latest[dev] = bytes(msg.data)
            self._stamp_ring[dev].append(stamp)
            q = self._recv[dev]
            q.append(now)
            while q and now - q[0] > _FPS_WINDOW:
                q.popleft()

    def _get(self, dev):
        with self._lock:
            return self._latest.get(dev)

    def _status(self):
        now = time.monotonic()
        with self._lock:
            recv = {d: list(q) for d, q in self._recv.items()}
            rings = {d: list(r) for d, r in self._stamp_ring.items()}
        devices = []
        for d in self.devs:
            times = [t for t in recv[d] if now - t <= _FPS_WINDOW]
            span = now - times[0] if len(times) >= 2 else 0.0
            fps = len(times) / span if span > 0 else 0.0
            devices.append({"dev": d, "fps": fps})
        sync = None
        frames = {d: [(s, None) for s in rings[d]] for d in self.devs if rings[d]}
        if len(frames) == len(self.devs):
            chosen = stats.match_frames(frames)
            if chosen:
                s = stats.timestamp_stats({d: c[0] for d, c in chosen.items()})
                sync = {"spread_ms": s["spread_ms"], "std_ms": s["std_ms"]}
        return {"devices": devices, "sync": sync}

    def _make_handler(self):
        node = self
        devs = self.devs
        period = 1.0 / self.fps

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):  # ROS 로그 오염 방지
                pass

            def _send(self, body, ctype):
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._send(index_html(devs).encode(),
                               "text/html; charset=utf-8")
                    return
                if self.path == "/status":
                    self._send(json.dumps(node._status()).encode(),
                               "application/json")
                    return
                if self.path.startswith("/stream/"):
                    try:
                        dev = int(self.path.rsplit("/", 1)[1])
                    except ValueError:
                        self.send_error(404)
                        return
                    if dev not in devs:
                        self.send_error(404)
                        return
                    self.send_response(200)
                    self.send_header(
                        "Content-Type",
                        f"multipart/x-mixed-replace; boundary={_BOUNDARY}")
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    try:
                        while rclpy.ok():
                            jpeg = node._get(dev)
                            if jpeg:
                                self.wfile.write(mjpeg_chunk(jpeg))
                            time.sleep(period)
                    except (BrokenPipeError, ConnectionResetError):
                        pass  # 브라우저 탭 닫힘 — 정상
                    return
                self.send_error(404)

        return Handler

    def destroy_node(self):
        self._httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WebMonitorNode()
    except OSError as e:
        # 포트 점유 등 바인드 실패 — 트레이스백 대신 안내 후 종료.
        print(f"web monitor: HTTP 서버 시작 실패 ({e}).\n"
              "  포트가 이미 사용 중이면 다른 포트로 실행하세요:\n"
              "    ros2 run econ_camera_ros monitor --ros-args -p port:=10011")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # Ctrl-C 시 rclpy SIGINT 핸들러가 이미 shutdown → 이중 호출 방지
            rclpy.shutdown()


if __name__ == "__main__":
    main()
