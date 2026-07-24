"""calib.yaml 의 라이다 extrinsic 읽기/쓰기(카메라 블록은 건드리지 않음)."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

_KEY = "T_front_lidar"


def load_T_front_lidar(calib_path):
    doc = yaml.safe_load(Path(calib_path).read_text())
    mat = (doc.get("extrinsics") or {}).get(_KEY)
    return None if mat is None else np.array(mat, dtype=np.float64)


def write_extrinsic(calib_path, T_front_lidar, meta):
    """T_front_lidar(4x4) 를 extrinsics 에, meta 를 verification.cam_lidar 에 병합 저장."""
    p = Path(calib_path)
    doc = yaml.safe_load(p.read_text())
    doc.setdefault("extrinsics", {})[_KEY] = np.asarray(T_front_lidar, float).tolist()
    ver = doc.setdefault("verification", {})
    ver["cam_lidar"] = {**ver.get("cam_lidar", {}), **meta}
    p.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False))
