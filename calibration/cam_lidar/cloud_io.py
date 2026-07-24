"""라이다 클라우드 IO·누적(순수 로직 + bag 읽기 지연 import).

PointCloud2 파싱은 mapping/check_lidar_bag.py 패턴을 따른다.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from chain import se3, se3_inv, transform

CLOUD_TOPIC = "/unilidar/cloud"


def cloud_to_xyzi(msg):
    """PointCloud2 유사 → (N,4) x,y,z,intensity(float64). 비유한 xyz 제거."""
    off = {f.name: f.offset for f in msg.fields}
    for k in ("x", "y", "z"):
        if k not in off:
            return np.empty((0, 4))
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 4))
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)

    def col(name):
        o = off[name]
        return buf[:, o:o + 4].copy().view(np.float32).reshape(n).astype(np.float64)

    inten = col("intensity") if "intensity" in off else np.zeros(n)
    xyzi = np.stack([col("x"), col("y"), col("z"), inten], -1)
    return xyzi[np.isfinite(xyzi[:, :3]).all(1)]


def voxel_downsample(xyzi, voxel):
    """voxel(m) 격자 양자화 후 셀당 첫 점만."""
    if len(xyzi) == 0 or voxel <= 0:
        return xyzi
    keys = np.floor(xyzi[:, :3] / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyzi[np.sort(idx)]


def accumulate_static(clouds, ref_ns=None, window_s=2.0):
    """정지 가정: 기준 시각 ±window 안의 프레임 단순 병합. clouds=[(stamp_ns, xyzi)]."""
    if not clouds:
        return np.empty((0, 4))
    if ref_ns is None:
        ref_ns = clouds[0][0]
    w = int(window_s * 1e9)
    sel = [xyzi for (s, xyzi) in clouds if abs(s - ref_ns) <= w]
    return np.vstack(sel) if sel else np.empty((0, 4))


def read_clouds(bag_path, topic=CLOUD_TOPIC):
    """(stamp_ns, xyzi) 리스트를 시간순 반환(bag 읽기). 지연 import."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from sensor_msgs.msg import PointCloud2

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id="mcap"),
                ConverterOptions("", ""))
    out = []
    while reader.has_next():
        t, data, stamp = reader.read_next()
        if t != topic:
            continue
        out.append((stamp, cloud_to_xyzi(deserialize_message(data, PointCloud2))))
    return out


def load_tum(path):
    """TUM(t tx ty tz qx qy qz qw, t=초) → (times_ns int64(T,), poses list[4x4])."""
    times, poses = [], []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        t, x, y, z, qx, qy, qz, qw = (float(v) for v in line.split()[:8])
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        T[:3, 3] = [x, y, z]
        times.append(int(round(t * 1e9)))
        poses.append(T)
    return np.array(times, np.int64), poses


def pose_at(times_ns, poses, t_ns):
    """브래킷 선형보간(위치)+slerp(회전). 범위 밖은 끝값 클램프."""
    if t_ns <= times_ns[0]:
        return poses[0].copy()
    if t_ns >= times_ns[-1]:
        return poses[-1].copy()
    j = int(np.searchsorted(times_ns, t_ns))
    t0, t1 = times_ns[j - 1], times_ns[j]
    a = (t_ns - t0) / (t1 - t0)
    rots = Rotation.from_matrix([poses[j - 1][:3, :3], poses[j][:3, :3]])
    R = Slerp([0.0, 1.0], rots)([a]).as_matrix()[0]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (1 - a) * poses[j - 1][:3, 3] + a * poses[j][:3, 3]
    return T


def accumulate_motion(clouds, times_ns, poses, at_ns, window_s=2.0):
    """각 프레임을 세상→기준(at) 라이다 프레임으로 정렬 후 병합. clouds=[(stamp_ns, xyzi)]."""
    if not clouds:
        return np.empty((0, 4))
    T_ref_inv = se3_inv(pose_at(times_ns, poses, at_ns))
    w = int(window_s * 1e9)
    out = []
    for s, xyzi in clouds:
        if abs(s - at_ns) > w or len(xyzi) == 0:
            continue
        T = T_ref_inv @ pose_at(times_ns, poses, s)   # lidar@s → lidar@ref
        P = transform(T, xyzi[:, :3])
        out.append(np.hstack([P, xyzi[:, 3:4]]))
    return np.vstack(out) if out else np.empty((0, 4))
