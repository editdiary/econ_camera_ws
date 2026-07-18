"""e-con 4-camera 연속 동기 수집 ROS2 노드.

단일 공유클럭 GStreamer 파이프라인을 소유하고, 각 카메라 appsink의 new-sample 콜백에서
JPEG 버퍼를 꺼내 sensor_msgs/CompressedImage 로 즉시 발행한다. 버퍼 PTS(파이프라인 공유
클럭)를 ROS 시각에 앵커링해 header.stamp 로 쓰므로 카메라 간 stamp 가 직접 비교 가능하다.
주기적으로 4대 타임스탬프 편차(spread/std)와 카메라별 수신 프레임 수를 로그한다.
"""

import array
import threading

import gi
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

from econ_cam import controls, stats  # noqa: E402
from econ_camera_ros import gst_builder  # noqa: E402

Gst.init(None)

_RING = 10        # 동기 로그용 카메라별 최근 PTS 개수
_CYCLE_TOL = 0.010  # 같은 frame_sync 주기로 볼 최대 PTS 간격(초). 주기(33ms)보다 작게.


class CaptureNode(Node):
    def __init__(self):
        super().__init__("econ_camera_capture")
        self.declare_parameter("devices", [0, 1, 2, 3])
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("sync_mode", 1)
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("log_period_s", 5.0)
        self.declare_parameter("warmup_s", 4.0)

        self.devs = list(self.get_parameter("devices").value)
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.sync_mode = self.get_parameter("sync_mode").value
        quality = self.get_parameter("jpeg_quality").value
        log_period = self.get_parameter("log_period_s").value
        self._warmup_s = self.get_parameter("warmup_s").value

        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._pubs = {
            d: self.create_publisher(
                CompressedImage, f"/camera{d}/image_raw/compressed", qos)
            for d in self.devs
        }

        self._lock = threading.Lock()
        self._pts_ring = {d: [] for d in self.devs}
        self._counts = {d: 0 for d in self.devs}
        self._t0_ros = None   # 첫 프레임 ROS 시각(ns)
        self._t0_pts = None   # 첫 프레임 PTS(ns)

        # 워밍업: warmup_s 동안 프레임을 폐기(발행/카운트 안 함)하고 4대 정상 동작을
        # 확인한 뒤, 같은 주기의 4대가 모인 첫 '깨끗한 사이클'부터 발행을 시작한다.
        self._publishing = False
        self._warmup_t0 = None
        self._warm_counts = {d: 0 for d in self.devs}  # 워밍업 중 폐기한 프레임 수
        self._pending = {}         # dev -> (stamp_ns, pts_ns, data): 첫 사이클 버퍼
        self._pending_pts0 = None  # 버퍼 기준 PTS(초)

        for d in self.devs:
            if not controls.set_frame_sync(d, self.sync_mode):
                self.get_logger().warn(f"frame_sync 설정 실패: /dev/video{d}")

        desc = gst_builder.capture_pipeline(
            self.devs, self.width, self.height, jpeg_quality=quality)
        self.get_logger().info(f"pipeline: {desc}")
        self._pipeline = Gst.parse_launch(desc)
        for d in self.devs:
            sink = self._pipeline.get_by_name(f"sink{d}")
            sink.set_property("emit-signals", True)
            sink.connect("new-sample", self._on_sample, d)
        self._pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(
            f"파이프라인 시작: devices={self.devs} @ {self.width}x{self.height}, "
            f"sync_mode={self.sync_mode}, jpeg_quality={quality} — "
            f"워밍업 {self._warmup_s:.1f}s 후 발행 시작")

        self._timer = self.create_timer(log_period, self._log_sync)

    def _on_sample(self, sink, dev):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        data = buf.extract_dup(0, buf.get_size())
        pts_ns = buf.pts

        with self._lock:
            stamp_ns = self._stamp_ns(pts_ns)
            if not self._publishing:
                to_publish = self._warmup(dev, pts_ns, stamp_ns, data)
                if not to_publish:
                    return Gst.FlowReturn.OK
            else:
                self._account(dev, pts_ns)
                to_publish = [(dev, stamp_ns, data)]

        for d, s_ns, payload in to_publish:
            self._publish(d, s_ns, payload)
        return Gst.FlowReturn.OK

    def _stamp_ns(self, pts_ns):
        """PTS→ROS 시각(ns). 첫 프레임(워밍업 포함)에서 오프셋 앵커링. 락 안에서 호출."""
        if pts_ns == Gst.CLOCK_TIME_NONE:
            return self.get_clock().now().nanoseconds
        if self._t0_ros is None:
            self._t0_ros = self.get_clock().now().nanoseconds
            self._t0_pts = pts_ns
        return self._t0_ros + (pts_ns - self._t0_pts)

    def _account(self, dev, pts_ns):
        """발행 프레임 집계: 동기 로그용 PTS 링 + 카운트. 락 안에서 호출."""
        if pts_ns != Gst.CLOCK_TIME_NONE:
            ring = self._pts_ring[dev]
            ring.append(pts_ns / 1e9)
            del ring[:-_RING]
        self._counts[dev] += 1

    def _warmup(self, dev, pts_ns, stamp_ns, data):
        """워밍업 처리(락 안에서 호출). 발행 개시 시 첫 사이클 프레임 리스트, 아니면 []."""
        now = self.get_clock().now().nanoseconds
        if self._warmup_t0 is None:
            self._warmup_t0 = now
        self._warm_counts[dev] += 1
        elapsed = (now - self._warmup_t0) / 1e9

        # 1) warmup_s 경과 + 4대 모두 프레임 확인 전엔 폐기(카메라 정상 동작 확인).
        if elapsed < self._warmup_s or not all(c > 0 for c in self._warm_counts.values()):
            return []

        # 2) 같은 주기의 4대가 모인 첫 '깨끗한 사이클'을 버퍼링. 다른 주기 프레임이 섞이면
        #    (dev 중복 또는 PTS 가 _CYCLE_TOL 초과) 리셋해 시작 카운트가 어긋나지 않게 한다.
        pts_s = None if pts_ns == Gst.CLOCK_TIME_NONE else pts_ns / 1e9
        mixed = self._pending and (
            dev in self._pending
            or (pts_s is not None and self._pending_pts0 is not None
                and abs(pts_s - self._pending_pts0) > _CYCLE_TOL))
        if mixed or not self._pending:
            self._pending = {}
            self._pending_pts0 = pts_s
        self._pending[dev] = (stamp_ns, pts_ns, data)
        if len(self._pending) < len(self.devs):
            return []

        # 3) 완전한 첫 사이클 확보 → 발행 개시. 이 4프레임을 첫 발행 세트로 반환.
        self._publishing = True
        cycle, self._pending = self._pending, {}
        self.get_logger().info(
            f"워밍업 {elapsed:.1f}s 완료(4대 정상 촬영 확인) — 발행 시작. "
            f"폐기 프레임={dict(self._warm_counts)}")
        out = []
        for d in self.devs:
            s_ns, pv, dt = cycle[d]
            self._account(d, pv)
            out.append((d, s_ns, dt))
        return out

    def _publish(self, dev, stamp_ns, data):
        msg = CompressedImage()
        msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        msg.header.frame_id = f"camera{dev}"
        msg.format = "jpeg"
        # rclpy는 bytes를 uint8[] 필드에 넣을 때 요소별 검증으로 ~11ms/frame(83KB)를
        # 쓴다(4대 GIL 직렬화 시 총 ~34fps로 병목). array.array('B')는 typecode가 맞아
        # 벌크 복사 빠른 경로를 타 ~0.005ms로 끝난다. (실측: 2200배)
        msg.data = array.array("B", data)
        self._pubs[dev].publish(msg)

    def _log_sync(self):
        with self._lock:
            publishing = self._publishing
            warm = dict(self._warm_counts)
            rings = {d: list(r) for d, r in self._pts_ring.items()}
            counts = dict(self._counts)
        if not publishing:
            self.get_logger().info(f"워밍업 중... 폐기 프레임={warm}")
            return
        frames = {d: [(p, None) for p in r] for d, r in rings.items() if r}
        if len(frames) < len(self.devs):
            self.get_logger().info(f"수신 대기중... frames={counts}")
            return
        chosen = stats.match_frames(frames)
        if not chosen:
            return
        s = stats.timestamp_stats({d: c[0] for d, c in chosen.items()})
        self.get_logger().info(
            f"동기 spread={s['spread_ms']:.2f}ms std={s['std_ms']:.2f}ms | "
            f"frames={counts}")

    def destroy_node(self):
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
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
