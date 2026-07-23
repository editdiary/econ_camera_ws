#!/usr/bin/env python3
"""rosbag2(mcap)의 /unilidar/cloud 가 제대로 찍혔는지 빠르게 점검.

bag_extract.py(rosbag2_py 읽기)와 bev_grid.py(numpy-only 헤드리스 PNG)를 참고.
LIO/tf 없이 센서 프레임 그대로 보므로 "라이다가 실제 구조를 담았는지" 확인용이다.

산출물(OUT):
  - lidar_bev_accum.png    : 시작 몇 초 누적(정지 구간 가정) top-down 높이맵
  - lidar_bev_single.png   : 포인트 가장 많은 단일 스캔 top-down 높이맵
  - 콘솔에 프레임 수/포인트 수/rate/공간 범위/판정 요약

사용:
  source /opt/ros/humble/setup.bash
  source <ws>/install/setup.bash
  python3 check_lidar_bag.py <rosbag2_디렉터리> [--out DIR] [--accum-sec 3] [--res 0.05]
"""
import argparse
import os
import struct
import sys
import zlib

import numpy as np

CLOUD_TOPIC = "/unilidar/cloud"


def turbo(t):
    t = np.clip(t, 0, 1)
    r = np.clip(1.5 - abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - abs(4 * t - 1), 0, 1)
    return (np.stack([r, g, b], -1) * 255).astype(np.uint8)


def write_png(path, img):
    h, w, _ = img.shape

    def ch(t, dd):
        c = t + dd
        return struct.pack(">I", len(dd)) + c + struct.pack(">I", zlib.crc32(c) & 0xffffffff)

    body = b"".join(b"\x00" + img[i].tobytes() for i in range(h))
    open(path, "wb").write(b"\x89PNG\r\n\x1a\n"
        + ch(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + ch(b"IDAT", zlib.compress(body, 9)) + ch(b"IEND", b""))


def bev_heightmap(xyz, res):
    """top-down (x=가로, y=세로) 셀별 최대높이(z) 컬러맵. bev_grid.py 방식."""
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    xlo, xhi = np.percentile(x, [0.3, 99.7])
    ylo, yhi = np.percentile(y, [0.3, 99.7])
    W = max(int((xhi - xlo) / res) + 1, 1)
    H = max(int((yhi - ylo) / res) + 1, 1)
    ix = ((x - xlo) / res).astype(int)
    iy = ((yhi - y) / res).astype(int)
    m = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    ix, iy, zz = ix[m], iy[m], z[m]
    cell = iy * W + ix
    hmax = np.full(W * H, -np.inf)
    np.maximum.at(hmax, cell, zz)
    occ = np.isfinite(hmax)
    img = np.zeros((W * H, 3), np.uint8)
    if occ.any():
        lo, hi = np.percentile(hmax[occ], [2, 98])
        img[occ] = turbo((hmax[occ] - lo) / ((hi - lo) or 1))
    return img.reshape(H, W, 3), W, H, int(occ.sum())


def cloud_to_xyz(msg):
    """PointCloud2 → 유한한 (N,3) float64. x,y,z(float32) 필드 오프셋으로 파싱."""
    off = {f.name: f.offset for f in msg.fields}
    for k in ("x", "y", "z"):
        if k not in off:
            return np.empty((0, 3))
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3))
    step = msg.point_step
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)

    def col(name):
        o = off[name]
        return buf[:, o:o + 4].copy().view(np.float32).reshape(n)

    xyz = np.stack([col("x"), col("y"), col("z")], -1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(1)]


def read_clouds(bag_path, topic):
    """(stamp_ns, xyz) 리스트를 시간순으로 반환."""
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
        msg = deserialize_message(data, PointCloud2)
        out.append((stamp, cloud_to_xyz(msg)))
    return out


def main():
    p = argparse.ArgumentParser(description="LiDAR bag(/unilidar/cloud) 점검 + BEV 미리보기")
    p.add_argument("bag", help="rosbag2 디렉터리 경로 (mcap 이 들어있는 폴더)")
    p.add_argument("--out", default=None, help="PNG 저장 폴더 (기본: bag 폴더)")
    p.add_argument("--topic", default=CLOUD_TOPIC, help=f"포인트클라우드 토픽 (기본 {CLOUD_TOPIC})")
    p.add_argument("--accum-sec", type=float, default=3.0, help="시작 누적 시간(초)")
    p.add_argument("--res", type=float, default=0.05, help="BEV 셀 해상도(m)")
    args = p.parse_args()

    if not os.path.isdir(args.bag):
        print(f"bag 디렉터리가 아닙니다: {args.bag}", file=sys.stderr)
        sys.exit(1)
    out = args.out or args.bag
    os.makedirs(out, exist_ok=True)

    try:
        clouds = read_clouds(args.bag, args.topic)
    except ImportError as e:
        print(f"ROS 임포트 실패 ({e}). setup.bash 를 먼저 source 하세요.", file=sys.stderr)
        sys.exit(1)

    if not clouds:
        print(f"'{args.topic}' 메시지가 bag 에 없습니다. 라이다가 안 찍혔습니다.", file=sys.stderr)
        sys.exit(2)

    counts = np.array([len(xyz) for _, xyz in clouds])
    stamps = np.array([s for s, _ in clouds], dtype=np.float64) / 1e9
    span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
    rate = (len(clouds) - 1) / span if span > 0 else 0.0
    gaps = np.diff(stamps) if len(stamps) > 1 else np.array([0.0])

    print("=" * 60)
    print(f"토픽: {args.topic}")
    print(f"프레임(메시지) 수 : {len(clouds)}")
    print(f"시간 길이         : {span:.1f} s   (평균 {rate:.2f} Hz)")
    print(f"프레임 간격        : 평균 {gaps.mean()*1000:.0f} ms, 최대 {gaps.max()*1000:.0f} ms")
    print(f"포인트/프레임      : 최소 {counts.min()}, 평균 {counts.mean():.0f}, 최대 {counts.max()}")
    empty = int((counts == 0).sum())
    print(f"빈 프레임          : {empty} / {len(clouds)}")

    # 시작 누적
    t0 = stamps[0]
    accum = [xyz for (s, xyz), t in zip(clouds, stamps) if t - t0 <= args.accum_sec and len(xyz)]
    if accum:
        acc = np.concatenate(accum)
        rng = acc.max(0) - acc.min(0)
        print(f"누적 범위(첫 {args.accum_sec:.0f}s, {len(accum)}프레임 {len(acc)}점): "
              f"X {rng[0]:.1f}m, Y {rng[1]:.1f}m, Z {rng[2]:.1f}m")
        img, W, H, occ = bev_heightmap(acc, args.res)
        path = os.path.join(out, "lidar_bev_accum.png")
        write_png(path, img)
        print(f"  -> {path}  (grid {W}x{H}, occupied cells {occ})")

    # 포인트 최다 단일 스캔
    bi = int(counts.argmax())
    single = clouds[bi][1]
    if len(single):
        img, W, H, occ = bev_heightmap(single, args.res)
        path = os.path.join(out, "lidar_bev_single.png")
        write_png(path, img)
        print(f"단일 스캔 #{bi} ({len(single)}점) -> {path}  (grid {W}x{H}, occupied {occ})")

    # 간단 판정
    ok = (empty == 0 and counts.mean() > 100 and rate > 0.5)
    print("-" * 60)
    print("판정:", "정상으로 보입니다 ✅" if ok else "확인 필요 ⚠️ (빈 프레임/저포인트/낮은 rate)")
    print("BEV PNG 를 열어 실제 구조(벽·바닥·물체)가 보이는지 눈으로 확인하세요.")
    print("=" * 60)


if __name__ == "__main__":
    main()
