"""rosbag2(mcap) → 동기 세트별 JPEG 추출 도구.

capture_node 가 4대를 frame_sync(sub-ms) 로 녹화한 CompressedImage(JPEG) bag 을 읽어,
각 프레임의 header.stamp 기준으로 4대를 한 세트로 묶어 다음 구조로 저장한다:

    out/
      frame_000000/  cam0.jpg cam1.jpg cam2.jpg cam3.jpg
      frame_000001/  ...
      sets.csv       # idx, stamp{dev}..., spread_ms

페이로드가 이미 JPEG라 디코드 없이 그대로 파일로 쓴다. 동기 그룹핑 로직은 순수 함수
group_synchronized 로 분리(테스트 대상). ROS(rosbag2_py) 임포트는 extract 안에서 지연
로딩하므로, 순수 로직은 ROS 없이도 import/test 가능하다.

실행:
    ros2 run econ_camera_ros bag_extract <bag_dir> -o out
    # 또는  python3 -m econ_camera_ros.bag_extract <bag_dir> -o out
"""

import argparse
import os
import re

_TOPIC_RE = re.compile(r"^/camera(\d+)/image_raw/compressed$")


def group_synchronized(frames_by_dev, tol=0.001):
    """카메라별 (stamp, payload) 리스트를 stamp 기준 동기 세트로 묶는다(순수 함수).

    devs[0] 를 anchor 로, 나머지 카메라에서 anchor stamp 에 가장 가까운 프레임을 tol 이내로
    찾아 전체 카메라가 채워지면 한 세트로 emit 한다. tol 은 프레임 주기(30fps→33ms)보다
    작아야 하며(기본 1ms), frame_sync 동기에선 실제 편차가 sub-ms 라 이 값으로도 여유롭다. 두 anchor 는
    최소 한 주기(~33ms) 떨어져 있고 tol < 주기/2 이므로 한 프레임이 두 세트에 재사용되지 않는다.

    Args:
        frames_by_dev: {dev: [(stamp_seconds, payload), ...]}
        tol: 동기 허용오차(초)
    Returns:
        anchor stamp 오름차순 list[{dev: (stamp, payload)}]. 완성 세트만 포함.
    """
    devs = sorted(frames_by_dev)
    if not devs:
        return []
    seq = {d: sorted(frames_by_dev[d], key=lambda f: f[0]) for d in devs}
    anchor, others = devs[0], devs[1:]
    ptr = {d: 0 for d in others}
    out = []
    for a in seq[anchor]:
        a_stamp = a[0]
        chosen = {anchor: a}
        matched_idx = {}
        for d in others:
            lst = seq[d]
            i = ptr[d]
            while (i + 1 < len(lst)
                   and abs(lst[i + 1][0] - a_stamp) <= abs(lst[i][0] - a_stamp)):
                i += 1
            ptr[d] = i  # 단조 전진(워밍업 지터로 뒤처진 프레임 재스캔 방지)
            if i < len(lst) and abs(lst[i][0] - a_stamp) <= tol:
                chosen[d] = lst[i]
                matched_idx[d] = i
            else:
                break
        if len(chosen) == len(devs):
            out.append(chosen)
            for d in others:  # 매칭된 프레임 소비 → 다음 anchor 가 재사용 못 함
                ptr[d] = matched_idx[d] + 1
    return out


def _read_frames(bag_path):
    """bag 을 읽어 {dev: [(stamp, jpeg_bytes), ...]} 로 반환(ROS 지연 임포트)."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from sensor_msgs.msg import CompressedImage

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id="mcap"),
                ConverterOptions("", ""))
    frames_by_dev = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = _TOPIC_RE.match(topic)
        if not m:
            continue
        dev = int(m.group(1))
        msg = deserialize_message(data, CompressedImage)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frames_by_dev.setdefault(dev, []).append((stamp, bytes(msg.data)))
    return frames_by_dev


def extract(bag_path, out_dir, tol=0.001, limit=None):
    """bag 을 읽어 동기 세트별 JPEG + sets.csv 로 저장. (완성 세트 수, 카메라별 프레임 수) 반환."""
    frames_by_dev = _read_frames(bag_path)
    counts = {d: len(v) for d, v in sorted(frames_by_dev.items())}
    sets = group_synchronized(frames_by_dev, tol=tol)
    if limit is not None:
        sets = sets[:limit]

    devs = sorted(frames_by_dev)
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "sets.csv"), "w") as csvf:
        csvf.write("idx," + ",".join(f"stamp{d}" for d in devs) + ",spread_ms\n")
        for idx, s in enumerate(sets):
            fdir = os.path.join(out_dir, f"frame_{idx:06d}")
            os.makedirs(fdir, exist_ok=True)
            stamps = []
            for d in devs:
                stamp, payload = s[d]
                with open(os.path.join(fdir, f"cam{d}.jpg"), "wb") as f:
                    f.write(payload)
                stamps.append(stamp)
            spread_ms = (max(stamps) - min(stamps)) * 1000.0
            csvf.write(f"{idx}," + ",".join(f"{st:.9f}" for st in stamps)
                       + f",{spread_ms:.3f}\n")
    return len(sets), counts


def main():
    p = argparse.ArgumentParser(
        description="rosbag2(mcap) → 동기 세트별 JPEG 추출")
    p.add_argument("bag", help="rosbag2 디렉터리 경로")
    p.add_argument("-o", "--out", default="extracted", help="출력 디렉터리(기본 extracted)")
    p.add_argument("--tolerance", type=float, default=0.001,
                   help="동기 허용오차(초, 기본 0.001)")
    p.add_argument("--limit", type=int, default=None, help="처음 N 세트만 추출")
    a = p.parse_args()

    n, counts = extract(a.bag, a.out, tol=a.tolerance, limit=a.limit)
    print(f"카메라별 프레임 수: {counts}")
    print(f"동기 세트 {n}개 → {a.out}/frame_*/  (+ {a.out}/sets.csv)")


if __name__ == "__main__":
    main()
