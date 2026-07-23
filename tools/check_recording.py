#!/usr/bin/env python3
"""녹화된 bag(mcap)의 무결성을 프레임 타임스탬프로 판정하는 현장용 검사 도구.

리플레이/모니터 화면의 버벅임은 전송·표시 단계 문제라 녹화 품질과 무관하다.
녹화 품질은 저장된 프레임의 간격으로만 판단해야 하므로, 이 스크립트는
bag 안의 프레임 타임스탬프를 직접 읽어 카메라별 FPS·간격·끊김·4대 정렬을 검사한다.

사용:
    python3 tools/check_recording.py <bag_dir 또는 .mcap 경로> [--fps 30]

의존성: pip install mcap  (ROS 환경 불필요)
"""
import argparse
import glob
import os
import statistics
import sys

from mcap.reader import make_reader


def find_mcaps(path):
    if os.path.isfile(path) and path.endswith(".mcap"):
        return [path]
    if os.path.isdir(path):
        return sorted(glob.glob(os.path.join(path, "*.mcap")))
    return []


def collect_log_times(mcaps):
    """토픽 -> 정렬된 log_time(ns) 리스트."""
    per = {}
    for mcap in mcaps:
        with open(mcap, "rb") as f:
            for _schema, channel, message in make_reader(f).iter_messages():
                per.setdefault(channel.topic, []).append(message.log_time)
    for topic in per:
        per[topic].sort()
    return per


def analyze(ts, fps):
    """단일 토픽 타임스탬프 리스트 -> 통계 dict."""
    n = len(ts)
    period_ms = 1000.0 / fps
    gap_ms = period_ms * 1.5  # 프레임 1장 이상 누락 기준
    dur = (ts[-1] - ts[0]) / 1e9 if n > 1 else 0.0
    d = sorted((b - a) / 1e6 for a, b in zip(ts, ts[1:])) if n > 1 else [0.0]
    gaps = [x for x in d if x > gap_ms]
    return {
        "n": n,
        "dur": dur,
        "fps": (n / dur) if dur > 0 else 0.0,
        "mean": statistics.mean(d),
        "med": statistics.median(d),
        "p95": d[min(int(len(d) * 0.95), len(d) - 1)],
        "max": max(d),
        "gaps": gaps,
        "gap_ms": gap_ms,
    }


def main():
    ap = argparse.ArgumentParser(description="녹화 bag 무결성 검사")
    ap.add_argument("bag", help="bag 디렉터리 또는 .mcap 파일 경로")
    ap.add_argument("--fps", type=float, default=30.0, help="기대 FPS (기본 30)")
    ap.add_argument("--pattern", default="camera",
                    help="카메라 토픽 판별 문자열 (기본 'camera')")
    args = ap.parse_args()

    mcaps = find_mcaps(args.bag)
    if not mcaps:
        print(f"[에러] mcap을 찾지 못함: {args.bag}", file=sys.stderr)
        return 2

    per = collect_log_times(mcaps)
    cam_topics = sorted(t for t in per if args.pattern in t)
    other = sorted(t for t in per if args.pattern not in t)

    if not cam_topics:
        print(f"[경고] '{args.pattern}' 토픽 없음. 전체 토픽: {sorted(per)}")
        cam_topics = sorted(per)

    print(f"bag: {args.bag}   기대 FPS: {args.fps}")
    print("-" * 96)
    stats = {}
    for topic in cam_topics:
        s = analyze(per[topic], args.fps)
        stats[topic] = s
        print(f"{topic}")
        print(f"    n={s['n']}  dur={s['dur']:.2f}s  fps={s['fps']:.2f}  "
              f"간격 mean={s['mean']:.1f} med={s['med']:.1f} "
              f"p95={s['p95']:.1f} max={s['max']:.1f}ms  "
              f"끊김(>{s['gap_ms']:.0f}ms)={len(s['gaps'])}")
    for topic in other:
        n = len(per[topic])
        dur = (per[topic][-1] - per[topic][0]) / 1e9 if n > 1 else 0.0
        rate = n / dur if dur > 0 else 0.0
        print(f"{topic}  (참고)  n={n} dur={dur:.2f}s rate={rate:.2f}Hz")

    # ---- 판정 ----
    print("-" * 96)
    problems = []
    counts = {t: stats[t]["n"] for t in cam_topics}
    if len(set(counts.values())) > 1:
        spread = max(counts.values()) - min(counts.values())
        # 30fps에서 1~2프레임 차이는 정상(시작/종료 타이밍). 3장 초과면 경고.
        if spread > 2:
            problems.append(f"카메라 간 프레임 수 불일치 {spread}장: {counts}")

    for topic, s in stats.items():
        if s["fps"] < args.fps * 0.95:
            problems.append(f"{topic}: FPS 낮음 {s['fps']:.2f} < {args.fps}")
        if s["gaps"]:
            worst = ", ".join(f"{g:.0f}ms" for g in sorted(s["gaps"], reverse=True)[:5])
            problems.append(f"{topic}: 끊김 {len(s['gaps'])}회 (최대 {worst})")

    if problems:
        print("판정: ❌ 문제 발견")
        for p in problems:
            print(f"  - {p}")
        return 1
    print(f"판정: ✅ 정상 — 카메라 {len(cam_topics)}대 모두 "
          f"~{args.fps:.0f}fps 연속, 끊김 없음, 프레임 수 정렬")
    return 0


if __name__ == "__main__":
    sys.exit(main())
