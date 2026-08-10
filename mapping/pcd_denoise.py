#!/usr/bin/env python3
"""map.pcd 고립 노이즈 제거 (Point-LIO 재실행 불필요, 기존 map.pcd 후처리).

원본은 절대 덮어쓰지 않고 `map_clean.pcd` 를 옆에 만든다.

## 무엇을 지우나 — "고립점"만

맵의 떨어진 점들은 한 종류가 아니다. DBSCAN(eps=0.3m) 으로 보면 세 계층으로 갈린다:

  A 본체        90~99%
  B 대형분리    500점+   **실구조물**. raws3 의 24,158점 군집은 13m x 42m 천장이다.
                         "최대 군집만 남기기" 를 하면 천장이 통째로 날아간다. 절대 금지.
  C/D 중소형    6~499점  사람/이동물체 잔상일 수도, 얇은 실구조물일 수도. 판단 보류.
  E 고립        반경 0.3m 에 이웃이 거의 없는 점. z=+35.7 같은 극단 flyer 가 전부 여기.

이 도구는 **E 만** 지운다. B/C/D 는 손대지 않는다.

## 고립도 척도

k번째 최근접 이웃 거리 `d_k`. 원본 full-res 전수 측정 결과(8개 맵 공통):

    d4 중앙값 3.1cm / p99 12~13cm / p99.9 25~31cm / 최대 1~20m

실구조물은 30cm 안에 이웃이 있고 노이즈는 미터 단위로 떨어진, 깨끗한 이봉 분포다.
그래서 기본 임계 `--dist 0.3` 은 p99.9 언저리이며 실구조물을 거의 건드리지 않는다.

주의: 임계값은 **점 밀도에 의존**한다. voxel 다운샘플한 클라우드에 그대로 쓰면 안 된다
(밀도가 10배 낮아져 필터가 무력화되거나 과도해진다). 이 도구는 항상 원본 해상도에 건다.

## 바닥 보호

라이다가 마스트에 달려 바닥을 거의 못 본다 → 바닥 점이 원래 희소 → 밀도/고립도 필터가
바닥을 우선적으로 삭제한다(실측: 하위 5% 낮은 점 제거율이 전체 평균의 2.3배, z min -0.90 -> -0.22).

그래서 `--protect-below` 아래의 점은 **면제가 아니라 임계를 `--protect-factor` 배로 완화**한다.
완전 면제로 하면 낮은 위치의 진짜 flyer 까지 무조건 보존되므로, 완화만 해서 판단 여지를 남긴다.

참고: 낮은 z 꼬리가 곧 노이즈인 것은 아니다. rawos4 의 z<-1.0 점 3,630개는 d4 중앙값 8.8cm 에
97.5% 가 30cm 안에 이웃이 있고 1,015점 군집을 이루는 **응집 구조**다(고립점 아님). 이 도구는
E 계층만 지우므로 그런 건 손대지 않는다 — z min 이 그대로 남는 게 정상이다. 그것이 실제 지형인지
LIO 수직 드리프트인지는 고립도로 판별할 수 없는 별개 문제다.

사용법:
  python3 mapping/pcd_denoise.py data/sj_bags/260722/maps/raws1_mapping/map.pcd
  python3 mapping/pcd_denoise.py data/sj_bags/260722/maps/*_mapping/map.pcd --dry-run
"""
import argparse
import pathlib
import re

import numpy as np

# PCD TYPE 문자 + SIZE -> numpy 타입
_PCD_T = {("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4", ("I", 8): "i8",
          ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("U", 8): "u8",
          ("F", 4): "f4", ("F", 8): "f8"}


def read_pcd_raw(path):
    """DATA binary PCD -> (header_lines, structured ndarray). 모든 필드를 보존한다.

    open3d 로 읽고 쓰면 x/y/z(+color/normal) 만 남고 intensity·curvature 가 사라진다.
    Point-LIO 산출 map.pcd 는 FIELDS 가 8개(intensity 포함)라 그대로 쓰면 데이터 손실이다.
    """
    with open(path, "rb") as f:
        head, meta = [], {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"헤더에서 DATA 를 못 찾음: {path}")
            head.append(line)
            s = line.decode("ascii", "replace").strip()
            if s.startswith("#"):
                continue
            key, _, val = s.partition(" ")
            meta[key] = val
            if key == "DATA":
                break
        if meta.get("DATA", "").strip() != "binary":
            raise ValueError(f"DATA binary 만 지원 (현재 '{meta.get('DATA')}'): {path}")

        names = meta["FIELDS"].split()
        sizes = [int(x) for x in meta["SIZE"].split()]
        types = meta["TYPE"].split()
        counts = [int(x) for x in meta.get("COUNT", " ".join(["1"] * len(names))).split()]
        n = int(meta["POINTS"])

        fmt = []
        for nm, sz, tp, ct in zip(names, sizes, types, counts):
            base = _PCD_T.get((tp, sz))
            if base is None:
                raise ValueError(f"미지원 필드 타입 {tp}{sz} ({nm}) in {path}")
            fmt.append((nm, base) if ct == 1 else (nm, base, ct))
        arr = np.frombuffer(f.read(np.dtype(fmt).itemsize * n), dtype=np.dtype(fmt), count=n)
    return head, arr


def write_pcd_raw(path, head, arr):
    """read_pcd_raw 로 읽은 헤더/배열을 그대로 다시 쓴다. WIDTH/POINTS 만 새 개수로 갱신."""
    n = len(arr)
    out = []
    for line in head:
        s = line.decode("ascii", "replace")
        if re.match(r"^(WIDTH|POINTS)\s", s):
            out.append(f"{s.split()[0]} {n}\n".encode("ascii"))
        else:
            out.append(line)
    with open(path, "wb") as f:
        f.writelines(out)
        f.write(np.ascontiguousarray(arr).tobytes())


def knn_dist(P, k=4):
    """각 점의 k번째 최근접 이웃 거리 d_k (자기 자신 제외). shape (N,)."""
    from scipy.spatial import cKDTree
    # query k+1: 첫 열은 자기 자신(거리 0)
    d, _ = cKDTree(P).query(P, k=k + 1, workers=-1)
    return d[:, k]


def resolve_z(P, spec):
    """'p1' 같은 퍼센타일 표기 또는 절대 z 값 -> float."""
    s = str(spec).strip()
    if s.lower().startswith("p"):
        return float(np.percentile(P[:, 2], float(s[1:])))
    return float(s)


def isolated_mask(P, k=4, dist=0.3, protect_below=None, protect_factor=3.0, dk=None):
    """제거할 점의 bool 마스크. True = 고립 노이즈.

    protect_below 아래 점은 임계를 protect_factor 배로 완화(면제 아님).
    dk 를 넘기면 k-NN 재계산을 건너뛴다(대형 클라우드에서 두 번 돌면 그만큼 느려진다).
    """
    if dk is None:
        dk = knn_dist(P, k=k)
    thr = np.full(len(P), float(dist))
    if protect_below is not None:
        thr[P[:, 2] < protect_below] = dist * protect_factor
    return dk > thr


def denoise_file(src, out_name="map_clean.pcd", k=4, dist=0.3,
                 protect_below="p1", protect_factor=3.0, dry_run=False):
    """map.pcd 하나를 정제해 같은 폴더에 out_name 으로 저장. 통계 dict 반환.

    intensity/normal/curvature 등 원본의 모든 필드를 보존한다.
    """
    src = pathlib.Path(src)
    head, arr = read_pcd_raw(src)
    P = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
    if len(P) == 0:
        raise ValueError(f"빈 클라우드: {src}")
    # NaN/Inf 가 섞이면 KDTree 가 죽는다. 애초에 유효점이 아니므로 먼저 떨군다.
    finite = np.isfinite(P).all(axis=1)
    n_bad = int((~finite).sum())
    if n_bad:
        arr, P = arr[finite], P[finite]

    zp = resolve_z(P, protect_below) if protect_below is not None else None
    dk = knn_dist(P, k=k)                       # 한 번만 계산해 마스크와 통계에 함께 쓴다
    rm = isolated_mask(P, k=k, dist=dist, protect_below=zp,
                       protect_factor=protect_factor, dk=dk)
    keep = ~rm
    Q = P[keep]

    st = {
        "name": src.parent.name.replace("_mapping", ""),
        "n_in": len(P), "n_out": int(keep.sum()), "n_rm": int(rm.sum()),
        "n_nonfinite": n_bad,
        "fields": list(arr.dtype.names),
        "z_protect": zp,
        "z_in": (P[:, 2].min(), P[:, 2].max()),
        "z_out": (Q[:, 2].min(), Q[:, 2].max()),
        # 보호가 실제로 몇 점을 살렸는지 = 완화 없었으면 지워졌을 낮은 점 수
        "saved_by_protect": int(((P[:, 2] < zp) & ~rm & (dk > dist)).sum())
                            if zp is not None else 0,
        "dst": None,
    }
    if not dry_run:
        dst = src.parent / out_name
        write_pcd_raw(dst, head, arr[keep])
        st["dst"] = dst
    return st


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pcd", nargs="+", help="map.pcd 경로(복수 가능)")
    ap.add_argument("--out-name", default="map_clean.pcd",
                    help="원본과 같은 폴더에 쓸 파일명 (기본 map_clean.pcd). 원본은 안 건드린다")
    ap.add_argument("-k", type=int, default=4, help="고립도에 쓸 이웃 번호 (기본 4)")
    ap.add_argument("--dist", type=float, default=0.3,
                    help="d_k 가 이 값(m)을 넘으면 고립으로 판정 (기본 0.3 = 실측 p99.9)")
    ap.add_argument("--protect-below", default="p1",
                    help="이 z 아래는 임계 완화. 'p1' 등 퍼센타일 또는 절대값. none=끔 (기본 p1)")
    ap.add_argument("--protect-factor", type=float, default=3.0,
                    help="보호 구간의 임계 배율 (기본 3.0)")
    ap.add_argument("--dry-run", action="store_true", help="쓰지 않고 통계만 출력")
    a = ap.parse_args()

    pb = None if str(a.protect_below).lower() == "none" else a.protect_below
    print(f"k={a.k}  dist={a.dist}m  protect_below={pb}  factor={a.protect_factor}"
          f"{'  [DRY-RUN]' if a.dry_run else ''}\n")
    hdr = (f"{'map':10s} {'입력':>10s} {'제거':>14s} {'출력':>10s} "
           f"{'보호z':>7s} {'보호로살림':>9s} {'z범위 변화':>26s}")
    print(hdr)
    print("-" * len(hdr))
    seen_fields = set()
    for p in a.pcd:
        st = denoise_file(p, out_name=a.out_name, k=a.k, dist=a.dist,
                          protect_below=pb, protect_factor=a.protect_factor,
                          dry_run=a.dry_run)
        zi, zo = st["z_in"], st["z_out"]
        print(f"{st['name']:10s} {st['n_in']:10d} "
              f"{st['n_rm']:7d}({st['n_rm']/st['n_in']*100:5.3f}%) {st['n_out']:10d} "
              f"{st['z_protect']:+7.2f} {st['saved_by_protect']:9d} "
              f"  [{zi[0]:+.2f},{zi[1]:+.2f}] -> [{zo[0]:+.2f},{zo[1]:+.2f}]")
        if st["n_nonfinite"]:
            print(f"{'':10s} └ NaN/Inf 로 버린 점 {st['n_nonfinite']}개")
        seen_fields.add(tuple(st["fields"]))
    for fs in seen_fields:
        print(f"\n보존된 필드({len(fs)}): {' '.join(fs)}")
    if not a.dry_run:
        print(f"원본 map.pcd 는 그대로. 정제본은 각 폴더의 {a.out_name}")


if __name__ == "__main__":
    main()
