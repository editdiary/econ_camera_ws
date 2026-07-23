"""Kalibr camchain(yaml) → 프로젝트 calib.yaml 사이드카.

Kalibr가 출력한 camera_model·intrinsics·상대자세(T_cn_cnm1 선형 체인)를 읽어,
카메라별 intrinsic과 cam0 기준 누적 extrinsic(T_camN_cam0)으로 정리한다.
검증(A)은 Kalibr 리포트의 재투영 RMS를 그대로 기록한다.

순수 변환(camchain_to_calib)만 단위 테스트하고, yaml 입출력은 main에서 수동 검증.

실행:
    python3 -m econ_camera_ros.calib_convert camchain.yaml --model ds -o calib.yaml \
        --rms cam0=0.31 cam1=0.29 cam2=0.33 cam3=0.30
"""

import argparse
import json


def _mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def camchain_to_calib(camchain, model_chosen, reproj_rms=None, topic_to_dir=None):
    """Kalibr camchain dict → 프로젝트 calib dict.

    topic_to_dir({rostopic: 방향})가 주어지면 각 카메라를 그 rostopic 의 방향으로 relabel
    (Kalibr cam 순서 가정 제거). 기준은 정렬상 첫 카메라(cams[0])의 방향. None 이면 기존 camN 라벨.
    """
    cams = sorted(camchain)
    if topic_to_dir is not None:
        labels = {name: topic_to_dir[camchain[name]["rostopic"]] for name in cams}
    else:
        labels = {name: name for name in cams}
    ref = labels[cams[0]]
    cameras, extrinsics = {}, {}
    cum = [[1 if i == j else 0 for j in range(4)] for i in range(4)]  # T_ref_ref = I
    for idx, name in enumerate(cams):
        c = camchain[name]
        label = labels[name]
        cameras[label] = {
            "camera_model": c["camera_model"],
            "intrinsics": list(c["intrinsics"]),
            "distortion_model": c.get("distortion_model", "none"),
            "distortion_coeffs": list(c.get("distortion_coeffs", [])),
            "resolution": list(c["resolution"]),
        }
        if idx > 0:
            cum = _mat_mul(c["T_cn_cnm1"], cum)  # T_cn_c0 = T_cn_cnm1 @ T_cnm1_c0
        extrinsics[f"T_{label}_{ref}"] = cum
    return {
        "model_chosen": model_chosen,
        "cameras": cameras,
        "extrinsics": extrinsics,
        "verification": {"reproj_rms_px": dict(reproj_rms or {})},
    }


def main():
    import yaml
    from econ_camera_ros.cam_layout import order_from_json

    p = argparse.ArgumentParser(description="Kalibr camchain → calib.yaml")
    p.add_argument("camchain", help="Kalibr가 출력한 *-camchain.yaml")
    p.add_argument("--model", required=True, help="채택 모델 라벨(ds/eucm)")
    p.add_argument("-o", "--out", default="calib.yaml")
    p.add_argument("--orientation",
                   help="orientation.json — 주면 cam0~3 을 방향명(front/right/rear/left)으로 라벨")
    p.add_argument("--rms", nargs="*", default=[],
                   help="Kalibr 리포트 재투영 RMS: 키=값 ... (--orientation 시 방향명, 아니면 camN)")
    a = p.parse_args()

    with open(a.camchain) as f:
        camchain = yaml.safe_load(f)
    topic_to_dir = None
    if a.orientation:
        with open(a.orientation) as f:
            order = order_from_json(json.load(f))
        topic_to_dir = {f"/cam{idx}/image_raw": d for idx, d in order}
    rms = {}
    for kv in a.rms:
        k, v = kv.split("=")
        rms[k] = float(v)
    calib = camchain_to_calib(camchain, a.model, rms, topic_to_dir)
    with open(a.out, "w") as f:
        yaml.safe_dump(calib, f, sort_keys=False)
    print(f"calib 작성 → {a.out} (model={a.model}, cams={list(calib['cameras'])})")


if __name__ == "__main__":
    main()
