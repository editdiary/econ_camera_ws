#!/usr/bin/env python
"""Kalibr 자체 AprilGrid 검출기로 이미지별 검출 코너 수를 리포트(+선택 필터)한다.

왜 Kalibr 검출기인가:
  촘촘한 AprilGrid 는 cv2.aruco·pupil-apriltags 같은 '낱개 태그' 검출기로는 안 읽혀서
  OpenCV 로는 재현이 불가능했다(docs/CALIBRATION.md §7). 그래서 진단도 반드시 캘리브와
  '동일한' 경로(detector.findTargetNoTransformation)로 해야 결과가 일치한다.

왜 필요한가(크래시 배경):
  kalibr_calibrate_cameras 는 intrinsic 초기화 때 관측 하나로 solvePnP(DLT)를 도는데
  DLT 는 최소 6점이 필요하다. 코너<6 인 '간당간당한' 프레임이 초기화에 뽑히면
      RuntimeError: DLT algorithm needs at least 6 points ... 'count' is 5
  로 캘리브 전체가 죽는다(Kalibr 4.2 에 가드 없음). 어떤 프레임이 문제인지 보이지 않는 게
  핵심 고통이라, 이 도구가 이미지별 코너 수를 뽑아 그 프레임을 콕 집어낸다.

동작:
  캘리브와 동일한 단일스레드 검출 루프로 토픽별 이미지마다 코너 수를 세어
  detect_report.csv 로 남기고, 기본으로 --min-corners 미만 프레임을 dataset_rejected/ 로
  '이동'(삭제 아님)한다. 이동 후엔 calib.bag 이 낡으므로 제거하여 run_kalibr.sh 가 재생성하게 한다.

컨테이너 안에서 실행(호스트엔 kalibr 파이썬 모듈 없음). run_detect_report.sh 가 감싼다.
"""
import argparse
import os

import numpy as np

import aslam_cameras_april as acv_april  # noqa: F401  (aprilgrid 타깃 검출기 등록)
import aslam_cv_backend as acvb
import kalibr_common as kc
import kalibr_camera_calibration as kcc


def cam_dir_from_topic(topic):
    """'/cam0/image_raw' -> 'cam0' (dataset 하위 폴더명)."""
    parts = [p for p in topic.split("/") if p]
    return parts[0]


def extract_counts(bag, topic, target_config):
    """토픽의 이미지별 (index, corners) 리스트. 검출 실패 프레임은 corners=0.

    kalibr_common.TargetExtractor.extractCornersFromDataset 의 단일스레드 분기와 동일한
    호출을 쓴다(검출 결과가 캘리브와 일치하도록).
    """
    reader = kc.BagImageDatasetReader(bag, topic, bag_from_to=None, bag_freq=None)
    cam = kcc.CameraGeometry(acvb.DoubleSphere, target_config, reader, verbose=False)
    detector = cam.ctarget.detector
    rows = []
    for idx, (timestamp, image) in enumerate(cam.dataset.readDataset()):
        success, obs = detector.findTargetNoTransformation(timestamp, np.array(image))
        n = len(obs.getCornersImageFrame()) if success == 1 else 0
        rows.append((idx, n))
    return rows


def main():
    p = argparse.ArgumentParser(description="Kalibr 검출기 이미지별 코너 수 리포트(+필터)")
    p.add_argument("--bag", required=True, help="calib.bag 경로")
    p.add_argument("--target", required=True, help="aprilgrid.yaml 경로")
    p.add_argument("--dataset", required=True, help="dataset 디렉터리(cam0..3/<ns>.png)")
    p.add_argument("--topics", nargs="+", required=True, help="검사할 토픽들")
    p.add_argument("--out", required=True, help="CSV 출력 경로")
    p.add_argument("--min-corners", type=int, default=8,
                   help="이 값 미만 프레임을 제외(크래시 경계는 6; 안정성 위해 기본 8)")
    p.add_argument("--report-only", action="store_true",
                   help="리포트만. 지정 없으면 min-corners 미만 프레임을 dataset_rejected/ 로 이동")
    a = p.parse_args()

    dataset = a.dataset.rstrip("/")
    target_config = kc.CalibrationTargetParameters(a.target)

    csv_lines = ["topic,cam,index,filename,corners"]
    total_moved = {}
    any_moved = False

    for topic in a.topics:
        camdir = cam_dir_from_topic(topic)
        print("\n=== %s ===" % topic)
        rows = extract_counts(a.bag, topic, target_config)

        files = sorted(os.listdir(os.path.join(dataset, camdir)))
        if len(files) != len(rows):
            print("  WARN: %s 파일수(%d) != 검출수(%d) — 인덱스↔파일 정렬 불확실"
                  % (camdir, len(files), len(rows)))

        counts = [n for _, n in rows]
        nonzero = [n for n in counts if n > 0]
        zero = counts.count(0)
        below6 = sum(1 for n in counts if 0 < n < 6)
        below_min = sum(1 for n in counts if n < a.min_corners)
        lo = min(nonzero) if nonzero else 0
        med = int(np.median(nonzero)) if nonzero else 0
        print("  검출 %d/%d | 코너 min=%d median=%d | 0검출 %d장 | 0<코너<6 %d장 | 코너<%d %d장"
              % (len(nonzero), len(rows), lo, med, zero, below6, a.min_corners, below_min))

        worst = sorted(((n, i) for i, n in rows if n > 0))[:8]
        if worst:
            def fn(i):
                return files[i] if i < len(files) else "idx%d" % i
            print("  최소 코너 프레임: " + ", ".join("%s(%d)" % (fn(i), n) for n, i in worst))

        for i, n in rows:
            name = files[i] if i < len(files) else ""
            csv_lines.append("%s,%s,%d,%s,%d" % (topic, camdir, i, name, n))

        if not a.report_only:
            rej_dir = os.path.join(dataset + "_rejected", camdir)
            moved = 0
            for i, n in rows:
                if n < a.min_corners and i < len(files):
                    os.makedirs(rej_dir, exist_ok=True)
                    os.rename(os.path.join(dataset, camdir, files[i]),
                              os.path.join(rej_dir, files[i]))
                    moved += 1
            total_moved[camdir] = moved
            any_moved = any_moved or moved > 0
            print("  → %d장 이동(<%d코너) → %s_rejected/%s"
                  % (moved, a.min_corners, os.path.basename(dataset), camdir))

    with open(a.out, "w") as f:
        f.write("\n".join(csv_lines) + "\n")
    print("\nCSV 저장: %s" % a.out)

    if not a.report_only:
        print("필터 완료. 이동 요약: %s" % total_moved)
        if any_moved and os.path.exists(a.bag):
            os.remove(a.bag)
            print("낡은 %s 제거 — run_kalibr.sh 재실행 시 재생성됨." % a.bag)


if __name__ == "__main__":
    main()
