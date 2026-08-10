"""슬래브 라벨 IO: 8필드 보존 PCD, self 마스크 PNG, 프레임 표본.

map.pcd 를 open3d 로 읽고 쓰면 intensity·curvature 가 사라진다. Point-LIO 산출은
FIELDS 가 8개라 mapping/pcd_denoise.py 의 바이너리 직접 IO 를 재사용한다.
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent.parent / "mapping"))
from cloud_io import load_tum                        # noqa: E402
from pcd_denoise import read_pcd_raw, write_pcd_raw  # noqa: E402

_MASK_FILES = {"front": "mask_front.png", "right": "mask_right.png",
               "left": "mask_left.png", "rear": "mask_rear.png"}


def load_map(map_dir):
    """map_clean.pcd + trajectory.tum → (head, arr, xyz(N,3), times_ns, poses)."""
    md = pathlib.Path(map_dir)
    head, arr = read_pcd_raw(str(md / "map_clean.pcd"))
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=-1).astype(np.float64)
    times_ns, poses = load_tum(str(md / "trajectory.tum"))
    return head, arr, xyz, times_ns, poses


def write_points(path, head, arr, idx, xyz_body):
    """arr[idx] 를 복제하고 x/y/z 를 body 좌표로 덮어 쓴 뒤 저장. 나머지 필드 보존."""
    sub = arr[np.asarray(idx)].copy()
    sub["x"] = xyz_body[:, 0].astype(sub["x"].dtype)
    sub["y"] = xyz_body[:, 1].astype(sub["y"].dtype)
    sub["z"] = xyz_body[:, 2].astype(sub["z"].dtype)
    write_pcd_raw(str(path), head, sub)


def load_labelmap(mask_dir):
    """labelmap.txt → {클래스명: (R,G,B)}. background 는 뺀다.

    형식(CVAT 내보내기): `# label:color_rgb:parts:actions` 주석 뒤로 `name:R,G,B::` 줄들.
    """
    out = {}
    for line in (pathlib.Path(mask_dir) / "labelmap.txt").read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        name, _, rest = line.partition(":")
        rgb = rest.split(":")[0]
        if name == "background" or not rgb:
            continue
        out[name] = tuple(int(v) for v in rgb.split(","))
    return out


def load_self_masks(mask_dir, use_names=("front", "left", "right"),
                    classes=("table",), tol=10):
    """{name: (H,W) bool} True=무효. 선택한 클래스 색의 화소만 무효로 본다.

    폴더가 없으면 {}, 개별 파일이 없으면 그 이름을 건너뛴다.

    기본이 table 만인 이유: handle·human 은 이미지에서의 위치가 프레임마다 달라진다
    (수집자가 화면을 확인하려 몸을 기울이고, 회전 구간에서 위치가 바뀌고, 턱에 걸려
    흔들린다). 정적 이미지 마스크는 없는 자리를 가리고(데이터 손실) 있는 자리를 놓친다
    (틀린 라벨). 물리적 위치는 body 프레임에서 늘 같으므로 slab_label.self_box_mask 가
    대신 덮는다.

    tol 은 색 비교 허용오차. 실측 마스크는 고유색이 2~4개뿐이라 정확히 일치하지만,
    나중에 안티에일리어싱된 내보내기가 와도 경계가 새지 않게 여유를 둔다.
    """
    import cv2
    d = pathlib.Path(mask_dir)
    out = {}
    if not d.is_dir():
        return out
    cmap = load_labelmap(d)
    want = [np.array(cmap[c], int) for c in classes if c in cmap]
    if not want:
        raise ValueError(f"labelmap.txt 에 {tuple(classes)} 중 아무 클래스도 없습니다: {d}")
    for name in use_names:
        p = d / _MASK_FILES[name]
        if not p.is_file():
            continue
        bgr = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        rgb = bgr[:, :, ::-1].astype(int)
        m = np.zeros(rgb.shape[:2], bool)
        for c in want:
            m |= np.abs(rgb - c).max(axis=2) <= tol
        out[name] = m
    return out


def sample_frames_gray(extract_dir, cam_idx, n=40):
    """추출 프레임에서 n 장을 균등 표본해 그레이스케일 (F,H,W) uint8 로."""
    import cv2
    ed = pathlib.Path(extract_dir)
    frames = sorted(ed.glob("frame_*"))
    if not frames:
        raise FileNotFoundError(f"frame_* 폴더가 없습니다: {ed}")
    sel = [frames[int(round(t))] for t in
           np.linspace(0, len(frames) - 1, min(n, len(frames)))]
    out = []
    for d in sel:
        img = cv2.imread(str(d / f"cam{cam_idx}.jpg"), cv2.IMREAD_GRAYSCALE)
        if img is not None:
            out.append(img)
    if not out:
        raise FileNotFoundError(f"cam{cam_idx}.jpg 를 못 읽었습니다: {ed}")
    return np.stack(out)
