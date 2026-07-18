"""bag_extract 산출물(frame_NNNNNN/cam{d}.jpg + sets.csv) → Kalibr 데이터셋 폴더.

Kalibr(kalibr_bagcreater)는 cam{d}/<timestamp_ns>.png 폴더 구조를 ROS1 bag으로 변환한다.
한 동기 세트의 4대는 같은 anchor stamp(ns)를 파일명으로 공유하므로 카메라 간 연관이 자명하다.
캘리브 안정화를 위해 30fps 세트를 ~4Hz로 다운샘플한다.

순수 로직(select_by_rate/stamp_to_ns/parse_set_stamps)은 cv2 없이 테스트 가능하고,
디코드/복사(build_dataset)는 cv2를 지연 임포트한다.

실행:
    python3 -m econ_camera_ros.kalibr_bridge <extracted_dir> -o dataset --rate 4.0
    ros2 run econ_camera_ros kalibr_bridge <extracted_dir> -o dataset
"""

import argparse
import os


def select_by_rate(stamps, target_hz):
    """stamp 오름차순 리스트에서 목표 레이트로 다운샘플한 인덱스(시간 기반 greedy)."""
    if not stamps:
        return []
    period = 1.0 / target_hz
    kept = [0]
    last = stamps[0]
    for i in range(1, len(stamps)):
        if stamps[i] - last >= period:
            kept.append(i)
            last = stamps[i]
    return kept


def stamp_to_ns(stamp_seconds):
    """초 단위 stamp → 나노초 정수(파일명용)."""
    return int(round(stamp_seconds * 1e9))


def parse_set_stamps(csv_text):
    """sets.csv 텍스트에서 세트별 anchor stamp(첫 stamp 열)를 순서대로 반환."""
    lines = [l for l in csv_text.splitlines() if l.strip()]
    out = []
    for line in lines[1:]:  # 헤더 스킵
        cols = line.split(",")
        out.append(float(cols[1]))
    return out


def build_dataset(extracted_dir, out_dir, target_hz=4.0, devs=(0, 1, 2, 3)):
    """extracted_dir → out_dir/cam{d}/<ns>.png. 유지 세트 수 반환(cv2 지연 임포트)."""
    import cv2

    with open(os.path.join(extracted_dir, "sets.csv")) as f:
        stamps = parse_set_stamps(f.read())
    kept = select_by_rate(stamps, target_hz)
    for d in devs:
        os.makedirs(os.path.join(out_dir, f"cam{d}"), exist_ok=True)
    for idx in kept:
        ns = stamp_to_ns(stamps[idx])
        for d in devs:
            src = os.path.join(extracted_dir, f"frame_{idx:06d}", f"cam{d}.jpg")
            dst = os.path.join(out_dir, f"cam{d}", f"{ns}.png")
            img = cv2.imread(src)
            if img is None:
                raise FileNotFoundError(src)
            cv2.imwrite(dst, img)
    return len(kept)


def main():
    p = argparse.ArgumentParser(description="extracted → Kalibr 데이터셋 폴더")
    p.add_argument("extracted", help="bag_extract 출력 디렉터리")
    p.add_argument("-o", "--out", default="dataset", help="Kalibr 데이터셋 출력(기본 dataset)")
    p.add_argument("--rate", type=float, default=4.0, help="다운샘플 목표 Hz(기본 4.0)")
    a = p.parse_args()
    n = build_dataset(a.extracted, a.out, target_hz=a.rate)
    print(f"유지 세트 {n}개 → {a.out}/cam{{0..3}}/<ns>.png")


if __name__ == "__main__":
    main()
