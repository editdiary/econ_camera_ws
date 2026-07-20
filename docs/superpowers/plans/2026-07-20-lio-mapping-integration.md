# LIO 오프라인 매핑 통합 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 녹화된 bag을 Point-LIO(ROS2)로 후처리하여 `{맵 PCD + 궤적 TUM + 미리보기}`를 한 번에 뽑는 오프라인 도구를 ws에 통합한다.

**Architecture:** `point_lio_ros2`를 `src/point_lio`로 벤더링(소스 무수정)하고, `mapping/` 디렉터리에 오케스트레이터 스크립트(`lio_map_bag.sh`) + 궤적 로거(`pose_logger.py`) + 시각화(`pcd_preview.py`/`bev_grid.py`)를 둔다. 실시간 수집·녹화 코드는 전혀 건드리지 않는다(`ros2 bag play` 기반 오프라인 처리).

**Tech Stack:** ROS2 Humble, colcon/ament_cmake(point_lio), rclpy(pose_logger), numpy(시각화), bash(오케스트레이션), pytest.

**설계 스펙:** `docs/superpowers/specs/2026-07-20-lio-mapping-integration-design.md`

## Global Constraints

- **벤더 소스 무수정**: `src/point_lio`의 C++/`config`/`launch`는 변경하지 않는다(기본 L2 설정으로 검증됨).
- **실시간 수집·녹화 코드 무수정**: `src/econ_camera_ros/`의 `capture_node.py`·`gst_builder.py`·`record*.launch.py`·`web_monitor_node.py`·`bag_extract.py`는 한 줄도 바꾸지 않는다.
- **궤적 포맷 = TUM**: 정확히 `t tx ty tz qx qy qz qw`(공백 구분, timestamp 초 단위, position xyz 다음 quaternion xyzw 순서).
- **헤드리스 호환**: 시각화는 numpy만 사용(matplotlib/open3d/DISPLAY 불필요).
- **빌드 선결 조건**: `ros-humble-pcl-ros`, `ros-humble-pcl-conversions`(apt, 시스템에 이미 설치됨 — 검증 완료).
- **회귀 금지**: 기존 카메라 pytest 18개(`cd src/econ_camera_ros && python3 -m pytest test/`) 그대로 통과.
- **브랜치/커밋**: `feat/lio-mapping-integration`에서 진행, 커밋까지만(병합·푸시는 사용자). 모든 커밋 trailer:
  `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`

---

## File Structure

- `src/point_lio/` — 벤더 패키지(복사, `.git` 제거, 소스 무수정). colcon 기본 포함.
- `mapping/lio_map_bag.sh` — bag→산출물 오케스트레이터.
- `mapping/pose_logger.py` — `/aft_mapped_to_init` 구독 → `trajectory.tum`(rclpy 노드).
- `mapping/tum.py` — TUM 라인 포맷 순수 함수(rclpy 비의존, 테스트 대상).
- `mapping/pcd_preview.py` — 원시점 top-down/side PNG(numpy).
- `mapping/bev_grid.py` — 셀 집계 height/density PNG(numpy).
- `mapping/test/test_pose_logger.py` — `tum.odom_to_tum_line` pytest.
- `docs/MAPPING.md` — 운영 가이드.
- `.gitignore` — 매핑 런타임 산출물 무시.
- `CLAUDE.md`, `docs/USAGE.md` — 반영.

---

## Task 1: point_lio 벤더링 + 빌드

**Files:**
- Create: `src/point_lio/` (from `third_party/point_lio_ros2/`)
- Modify: `.gitignore`

**Interfaces:**
- Produces: colcon 패키지 `point_lio` (실행파일 `pointlio_mapping`, launch `mapping_unilidar_l2.launch.py`, config `unilidar_l2.yaml`) — Task 4가 `ros2 launch point_lio ...`로 소비.

- [ ] **Step 1: 벤더 clone을 src/로 복사(.git·대용량 pcd 제외)**

```bash
cd ~/Desktop/econ_camera_ws
rsync -a --exclude='.git' --exclude='PCD/scans.pcd' \
    third_party/point_lio_ros2/ src/point_lio/
```

- [ ] **Step 2: 복사 검증(.git 없음, 패키지 메타 존재)**

```bash
test ! -e src/point_lio/.git && echo "no .git OK"
test -f src/point_lio/package.xml && test -f src/point_lio/CMakeLists.txt && echo "pkg files OK"
grep -q '<name>point_lio</name>' src/point_lio/package.xml && echo "name OK"
test -f src/point_lio/launch/mapping_unilidar_l2.launch.py && echo "launch OK"
```
Expected: `no .git OK` / `pkg files OK` / `name OK` / `launch OK` 모두 출력.

- [ ] **Step 3: .gitignore에 매핑 런타임 산출물 추가**

`.gitignore` 끝에 다음 블록을 추가(중복 시 생략):

```gitignore
# LIO 매핑 런타임 산출물
/out/
mapping/preview/
src/point_lio/PCD/*.pcd
src/point_lio/Log/*.txt
```

- [ ] **Step 4: 빌드**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select point_lio
```
Expected: `Finished <<< point_lio`, 에러 0 (C++ 템플릿 경고는 허용).

- [ ] **Step 5: 실행파일 산출 확인**

```bash
test -f install/point_lio/lib/point_lio/pointlio_mapping && echo "executable OK"
```
Expected: `executable OK`.

- [ ] **Step 6: 커밋** (원본 clone은 Task 5에서 제거 — 지금은 Task 3 시각화 검증용 scans.pcd 보존)

```bash
git add src/point_lio .gitignore
git commit -m "$(cat <<'EOF'
feat(mapping): point_lio(ROS2) 벤더링 + colcon 빌드

third_party/point_lio_ros2를 src/point_lio로 복사(.git 제거, 소스 무수정).
livox 없이 빌드, pcl_ros/pcl_conversions 의존. bag 후처리용 LIO 노드.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: 궤적 로거(pose_logger) + TUM 포맷 + 테스트

**Files:**
- Create: `mapping/tum.py`, `mapping/pose_logger.py`, `mapping/test/test_pose_logger.py`

**Interfaces:**
- Produces: `tum.odom_to_tum_line(stamp_sec, px,py,pz, qx,qy,qz,qw) -> str`; `pose_logger.py <out.tum>` 실행(=`/aft_mapped_to_init`→TUM 파일). Task 4가 백그라운드로 기동.

- [ ] **Step 1: 실패하는 테스트 작성** (`mapping/test/test_pose_logger.py`)

```python
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from tum import odom_to_tum_line


def test_field_order_and_count():
    line = odom_to_tum_line(1.5, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    parts = line.split()
    assert len(parts) == 8
    # position(xyz) 다음 quaternion(xyzw)
    assert parts[1:4] == [f"{1.0:.6f}", f"{2.0:.6f}", f"{3.0:.6f}"]
    assert parts[4:8] == [f"{0.0:.6f}", f"{0.0:.6f}", f"{0.0:.6f}", f"{1.0:.6f}"]


def test_timestamp_seconds_precision():
    t = 1784544578.963209
    line = odom_to_tum_line(t, 0, 0, 0, 0, 0, 0, 1)
    assert line.split()[0] == f"{t:.9f}"


def test_no_trailing_newline():
    line = odom_to_tum_line(0.0, 0, 0, 0, 0, 0, 0, 1)
    assert not line.endswith("\n")
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd ~/Desktop/econ_camera_ws
python3 -m pytest mapping/test/test_pose_logger.py -q
```
Expected: FAIL — `ModuleNotFoundError: No module named 'tum'`.

- [ ] **Step 3: 순수 함수 구현** (`mapping/tum.py`)

```python
"""TUM 궤적 라인 포맷 (rclpy 비의존, 테스트 가능)."""


def odom_to_tum_line(stamp_sec, px, py, pz, qx, qy, qz, qw):
    """TUM 한 줄: 't tx ty tz qx qy qz qw' (공백 구분, timestamp 초)."""
    return (
        f"{stamp_sec:.9f} "
        f"{px:.6f} {py:.6f} {pz:.6f} "
        f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}"
    )
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
python3 -m pytest mapping/test/test_pose_logger.py -q
```
Expected: `3 passed`.

- [ ] **Step 5: rclpy 로거 노드 구현** (`mapping/pose_logger.py`)

```python
#!/usr/bin/env python3
"""pose_logger: /aft_mapped_to_init (nav_msgs/Odometry) -> TUM 궤적 파일.

사용법: python3 pose_logger.py <out.tum>
point_lio와 함께 기동; SIGINT로 종료(종료 시 flush).
"""
import os
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tum import odom_to_tum_line


class PoseLogger(Node):
    def __init__(self, out_path):
        super().__init__("pose_logger")
        self._f = open(out_path, "w")
        self._n = 0
        self.create_subscription(Odometry, "/aft_mapped_to_init", self._cb, 200)

    def _cb(self, msg):
        st = msg.header.stamp
        t = st.sec + st.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._f.write(odom_to_tum_line(t, p.x, p.y, p.z, q.x, q.y, q.z, q.w) + "\n")
        self._n += 1

    def destroy_node(self):
        try:
            self._f.flush()
            self._f.close()
        finally:
            super().destroy_node()


def main():
    if len(sys.argv) < 2:
        print("usage: pose_logger.py <out.tum>", file=sys.stderr)
        sys.exit(1)
    out_path = sys.argv[1]
    rclpy.init()
    node = PoseLogger(out_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"wrote {node._n} poses to {out_path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: 구문/임포트 스모크 체크**

```bash
source /opt/ros/humble/setup.bash
python3 -c "import ast; ast.parse(open('mapping/pose_logger.py').read()); print('syntax OK')"
python3 -c "import rclpy, nav_msgs.msg; print('deps OK')"
```
Expected: `syntax OK` / `deps OK`.

- [ ] **Step 7: 커밋**

```bash
git add mapping/tum.py mapping/pose_logger.py mapping/test/test_pose_logger.py
git commit -m "$(cat <<'EOF'
feat(mapping): pose_logger + TUM 포맷 (궤적 복원)

/aft_mapped_to_init(Odometry) -> trajectory.tum. 순수 포맷 함수 tum.py
+ pytest 3개. BEV가 필요로 하는 ego-pose 산출.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: 시각화 도구 파라미터화 (pcd_preview / bev_grid)

**Files:**
- Modify: `mapping/pcd_preview.py`, `mapping/bev_grid.py` (현재 scratchpad에서 복사된 하드코딩 경로 버전 → `<pcd> [out_dir]` 인자화)

**Interfaces:**
- Produces: `pcd_preview.py <pcd> [out_dir]` → `bev_topdown.png`,`side_xz.png`; `bev_grid.py <pcd> [out_dir]` → `bev_heightmap.png`,`bev_density.png`. `out_dir` 생략 시 pcd와 같은 폴더. Task 4가 호출.

- [ ] **Step 1: `mapping/pcd_preview.py` 전체 교체(인자화)**

```python
#!/usr/bin/env python3
"""scans.pcd -> top-down(BEV) + side preview PNG (numpy only, headless).
사용법: pcd_preview.py <pcd> [out_dir]"""
import os
import struct
import sys
import zlib

import numpy as np

PCD = sys.argv[1]
OUT_DIR = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(PCD))
os.makedirs(OUT_DIR, exist_ok=True)

with open(PCD, "rb") as f:
    raw = f.read()
hdr_end = raw.find(b"DATA binary\n") + len(b"DATA binary\n")
header = raw[:hdr_end].decode("ascii", "replace")
sizes, counts, npts = [], [], 0
for line in header.splitlines():
    p = line.split()
    if not p:
        continue
    if p[0] == "SIZE":
        sizes = list(map(int, p[1:]))
    elif p[0] == "COUNT":
        counts = list(map(int, p[1:]))
    elif p[0] == "POINTS":
        npts = int(p[1])
stride = sum(s * c for s, c in zip(sizes, counts))
data = np.frombuffer(raw[hdr_end:hdr_end + npts * stride], dtype=np.uint8).reshape(npts, stride)
xyz = data[:, :12].copy().view(np.float32).reshape(npts, 3).astype(np.float64)
x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
print(f"points={npts}  x[{x.min():.1f},{x.max():.1f}] y[{y.min():.1f},{y.max():.1f}] z[{z.min():.1f},{z.max():.1f}]")


def turbo(t):
    t = np.clip(t, 0, 1)
    r = np.clip(1.5 - abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - abs(4 * t - 1), 0, 1)
    return (np.stack([r, g, b], -1) * 255).astype(np.uint8)


def write_png(path, img):
    h, w, _ = img.shape

    def chunk(typ, d):
        c = typ + d
        return struct.pack(">I", len(d)) + c + struct.pack(">I", zlib.crc32(c) & 0xffffffff)

    body = b"".join(b"\x00" + img[i].tobytes() for i in range(h))
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(body, 9))
           + chunk(b"IEND", b""))
    open(path, "wb").write(png)


def rasterize(a, b, cval, name, res_px=900, pad=0.02):
    alo, ahi = np.percentile(a, [0.5, 99.5])
    blo, bhi = np.percentile(b, [0.5, 99.5])
    da = (ahi - alo) or 1
    db = (bhi - blo) or 1
    alo -= da * pad; ahi += da * pad; blo -= db * pad; bhi += db * pad
    W = res_px
    H = max(60, int(res_px * (bhi - blo) / (ahi - alo)))
    ia = ((a - alo) / (ahi - alo) * (W - 1)).astype(int)
    ib = ((bhi - b) / (bhi - blo) * (H - 1)).astype(int)
    m = (ia >= 0) & (ia < W) & (ib >= 0) & (ib < H)
    ia, ib, cv = ia[m], ib[m], cval[m]
    flat = ib * W + ia
    lo, hi = np.percentile(cv, [2, 98])
    t = (cv - lo) / ((hi - lo) or 1)
    order = np.argsort(cv)
    img = np.zeros((H * W, 3), np.uint8)
    img[flat[order]] = turbo(t[order])
    write_png(f"{OUT_DIR}/{name}.png", img.reshape(H, W, 3))
    print(f"{name}.png  {W}x{H}px  span {ahi-alo:.1f}x{bhi-blo:.1f} m")


rasterize(x, y, z, "bev_topdown")
rasterize(x, z, z, "side_xz")
```

- [ ] **Step 2: `mapping/bev_grid.py` 전체 교체(인자화)**

```python
#!/usr/bin/env python3
"""scans.pcd -> per-cell BEV height map + density map (numpy only, headless).
셀마다 값 하나로 집계 = 실제 BEV 라벨 형식. 사용법: bev_grid.py <pcd> [out_dir]"""
import os
import struct
import sys
import zlib

import numpy as np

PCD = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(PCD))
os.makedirs(OUT, exist_ok=True)
RES = 0.05  # meters per cell

with open(PCD, "rb") as f:
    raw = f.read()
he = raw.find(b"DATA binary\n") + len(b"DATA binary\n")
hdr = raw[:he].decode("ascii", "replace")
sizes, counts, npts = [], [], 0
for line in hdr.splitlines():
    p = line.split()
    if not p:
        continue
    if p[0] == "SIZE":
        sizes = list(map(int, p[1:]))
    elif p[0] == "COUNT":
        counts = list(map(int, p[1:]))
    elif p[0] == "POINTS":
        npts = int(p[1])
stride = sum(s * c for s, c in zip(sizes, counts))
d = np.frombuffer(raw[he:he + npts * stride], np.uint8).reshape(npts, stride)
xyz = d[:, :12].copy().view(np.float32).reshape(npts, 3).astype(np.float64)
x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]


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


xlo, xhi = np.percentile(x, [0.3, 99.7])
ylo, yhi = np.percentile(y, [0.3, 99.7])
W = int((xhi - xlo) / RES) + 1
H = int((yhi - ylo) / RES) + 1
ix = ((x - xlo) / RES).astype(int)
iy = ((yhi - y) / RES).astype(int)
m = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
ix, iy, zz = ix[m], iy[m], z[m]
cell = iy * W + ix

hmax = np.full(W * H, -np.inf)
np.maximum.at(hmax, cell, zz)
occ = np.isfinite(hmax)
img = np.zeros((W * H, 3), np.uint8)
lo, hi = np.percentile(hmax[occ], [2, 98])
img[occ] = turbo((hmax[occ] - lo) / ((hi - lo) or 1))
write_png(f"{OUT}/bev_heightmap.png", img.reshape(H, W, 3))

cnt = np.bincount(cell, minlength=W * H).astype(np.float64)
occ2 = cnt > 0
img2 = np.zeros((W * H, 3), np.uint8)
lc = np.log1p(cnt[occ2])
img2[occ2] = turbo(lc / (lc.max() or 1))
write_png(f"{OUT}/bev_density.png", img2.reshape(H, W, 3))

filled = int(occ.sum())
print(f"grid {W}x{H} @ {RES}m  occupied {filled}/{W*H} = {100*filled/(W*H):.0f}%  max_pts_in_cell={int(cnt.max())}")
```

- [ ] **Step 3: 기존 맵으로 관통 실행(임시 출력 폴더)**

```bash
cd ~/Desktop/econ_camera_ws
python3 mapping/pcd_preview.py third_party/point_lio_ros2/PCD/scans.pcd /tmp/mp_test
python3 mapping/bev_grid.py    third_party/point_lio_ros2/PCD/scans.pcd /tmp/mp_test
ls -la /tmp/mp_test/*.png
```
Expected: `bev_topdown.png`, `side_xz.png`, `bev_heightmap.png`, `bev_density.png` 4개 생성(크기 > 0).

- [ ] **Step 4: 커밋**

```bash
git add mapping/pcd_preview.py mapping/bev_grid.py
git commit -m "$(cat <<'EOF'
feat(mapping): PCD 시각화 도구 인자화 (headless PNG)

pcd_preview(원시점 top-down/side) + bev_grid(셀 집계 height/density).
<pcd> [out_dir] 인자, numpy만으로 헤드리스 렌더.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: 오케스트레이터 lio_map_bag.sh + 관통 검증

**Files:**
- Create: `mapping/lio_map_bag.sh`

**Interfaces:**
- Consumes: `point_lio`(Task 1), `pose_logger.py`(Task 2), `pcd_preview.py`/`bev_grid.py`(Task 3).
- Produces: `<out_dir>/{map.pcd, trajectory.tum, run_info.txt, preview/*.png}`.

- [ ] **Step 1: `mapping/lio_map_bag.sh` 작성**

```bash
#!/bin/bash
# lio_map_bag.sh — bag을 Point-LIO로 매핑해 {map.pcd, trajectory.tum, run_info, preview} 생성
# 사용법: ./mapping/lio_map_bag.sh <bag_path> <out_dir> [--no-preview]
set -uo pipefail

BAG="${1:?사용법: $0 <bag_path> <out_dir> [--no-preview]}"
OUT="${2:?사용법: $0 <bag_path> <out_dir> [--no-preview]}"
PREVIEW=1
[ "${3:-}" = "--no-preview" ] && PREVIEW=0

WS="$(cd "$(dirname "$0")/.." && pwd)"
MAP_DIR="$WS/mapping"
PCD_SRC="$WS/src/point_lio/PCD/scans.pcd"

mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "[lio_map_bag] bag=$BAG  out=$OUT"
rm -f "$PCD_SRC"   # 이전 실행 잔여 맵 제거

# 1) 궤적 로거 (백그라운드)
python3 "$MAP_DIR/pose_logger.py" "$OUT/trajectory.tum" &
LOGGER_PID=$!

# 2) 매핑 노드 (프로세스 그룹 분리)
setsid ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false \
    > "$OUT/point_lio.log" 2>&1 &
NODE_PGID=$!

sleep 6   # 노드 초기화 대기

# 3) bag 재생 (블로킹)
echo "[lio_map_bag] 재생 시작..."
ros2 bag play "$BAG"
echo "[lio_map_bag] 재생 종료"
sleep 3

# 4) 매핑 노드 SIGINT → scans.pcd 저장
kill -INT -"$NODE_PGID" 2>/dev/null || kill -INT "$NODE_PGID" 2>/dev/null || true
for _ in $(seq 1 40); do kill -0 "$NODE_PGID" 2>/dev/null || break; sleep 1; done

# 5) 로거 종료
kill -INT "$LOGGER_PID" 2>/dev/null || true
wait "$LOGGER_PID" 2>/dev/null || true

# 6) 산출물 수집
if [ -f "$PCD_SRC" ]; then
    mv "$PCD_SRC" "$OUT/map.pcd"
else
    echo "[lio_map_bag] 경고: scans.pcd 없음 (맵 미생성)" >&2
fi

# 7) run_info
{
    echo "bag: $BAG"
    echo "git: $(git -C "$WS" rev-parse HEAD 2>/dev/null || echo unknown)"
    echo "poses: $(wc -l < "$OUT/trajectory.tum" 2>/dev/null || echo 0)"
    [ -f "$OUT/map.pcd" ] && echo "map_points: $(grep -m1 '^POINTS' "$OUT/map.pcd" | awk '{print $2}')"
} > "$OUT/run_info.txt"

# 8) 미리보기
if [ "$PREVIEW" = 1 ] && [ -f "$OUT/map.pcd" ]; then
    mkdir -p "$OUT/preview"
    python3 "$MAP_DIR/pcd_preview.py" "$OUT/map.pcd" "$OUT/preview" || true
    python3 "$MAP_DIR/bev_grid.py"   "$OUT/map.pcd" "$OUT/preview" || true
fi

echo "[lio_map_bag] 완료 → $OUT"
ls -la "$OUT"
```

- [ ] **Step 2: 실행 권한 부여 + 구문 체크**

```bash
chmod +x mapping/lio_map_bag.sh
bash -n mapping/lio_map_bag.sh && echo "syntax OK"
```
Expected: `syntax OK`.

- [ ] **Step 3: 정지 테스트 bag으로 관통 실행(백그라운드)**

> ⚠️ 스크립트는 `sleep`+bag 재생(~141s)이라 총 ~2.5분 소요. `run_in_background`로 실행 후 완료 통지를 기다린다.

```bash
cd ~/Desktop/econ_camera_ws
./mapping/lio_map_bag.sh rosbag2_2026_07_20-19_49_12 out/run01
```

- [ ] **Step 4: 산출물 검증**

```bash
echo "--- run_info ---"; cat out/run01/run_info.txt
echo "--- trajectory 첫 줄 ---"; head -1 out/run01/trajectory.tum
echo "--- 필드 수(=8) ---"; head -1 out/run01/trajectory.tum | awk '{print NF}'
echo "--- pose 라인 수(>0) ---"; wc -l < out/run01/trajectory.tum
echo "--- map point 수(>0) ---"; grep -m1 '^POINTS' out/run01/map.pcd
echo "--- preview PNG ---"; ls out/run01/preview/
```
Expected: `run_info.txt`에 poses>0·map_points>0; trajectory 첫 줄 필드 8개; `map.pcd` POINTS>0; preview에 4개 PNG.

- [ ] **Step 5: 커밋**

```bash
git add mapping/lio_map_bag.sh
git commit -m "$(cat <<'EOF'
feat(mapping): lio_map_bag.sh 오케스트레이터 (bag→맵·궤적·미리보기)

pose_logger + point_lio launch + bag play 수명관리(SIGINT로 pcd 저장),
산출물을 <out_dir>/{map.pcd,trajectory.tum,run_info.txt,preview/}로 정리.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: 문서화 + 원본 clone 정리

**Files:**
- Create: `docs/MAPPING.md`
- Modify: `CLAUDE.md`, `docs/USAGE.md`
- Remove: `third_party/point_lio_ros2/` (벤더링 완료 후 원본 clone)

- [ ] **Step 1: `docs/MAPPING.md` 작성**

```markdown
# 오프라인 매핑 가이드 (Point-LIO)

녹화된 bag을 후처리하여 **ego 궤적(pose) + 3D 맵**을 복원한다. 실시간 수집·녹화와 완전 분리된
오프라인 처리이며, BEV 학습 데이터셋의 전제(ego-motion)를 만든다.

## 1. 구성
- `src/point_lio` — Point-LIO(ROS2) 벤더 패키지. LiDAR-Inertial Odometry.
- `mapping/lio_map_bag.sh` — bag → 산출물 오케스트레이터.
- `mapping/pose_logger.py` — `/aft_mapped_to_init` → `trajectory.tum`.
- `mapping/pcd_preview.py`, `mapping/bev_grid.py` — 헤드리스 PNG 시각화(numpy).

## 2. 선결 조건 (최초 1회)
```bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
cd ~/Desktop/econ_camera_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select point_lio
source install/setup.bash
```

## 3. 실행
```bash
./mapping/lio_map_bag.sh <bag_경로> <출력_폴더>
# 예: ./mapping/lio_map_bag.sh rosbag2_2026_07_20-19_49_12 out/run01
```
- `--no-preview` : 미리보기 PNG 생략.
- 소요: bag 재생 시간 + 초기화 ~10초.

## 4. 산출물 (`<출력_폴더>/`)
| 파일 | 내용 |
|---|---|
| `map.pcd` | world 프레임 누적 점군(dense 3D 맵) |
| `trajectory.tum` | ego pose 시퀀스 `t tx ty tz qx qy qz qw` (BEV의 핵심) |
| `run_info.txt` | bag·git·pose 수·map point 수 |
| `preview/*.png` | top-down/side, BEV height/density 미리보기 |

## 5. 시각화
- 헤드리스: `python3 mapping/pcd_preview.py <pcd> [out]`, `mapping/bev_grid.py <pcd> [out]`.
- 디스플레이: `pcl_viewer map.pcd`(`sudo apt install pcl-tools`), 또는 RViz2
  (`ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=true` 실시간).

## 6. 판정 / 주의
- **정지 vs 이동**: 정지 촬영이면 맵에 동심원 링·과밀이 나타난다(정상). 실제 BEV 데이터용
  맵은 **공간을 이동하며** 녹화해야 궤적이 생긴다. 초기 몇 초는 정지(IMU 초기화).
- `trajectory.tum` 라인 수가 0이면 pose 미복원 → bag의 `/unilidar/imu`·`/unilidar/cloud` 확인.
- `map.pcd` 미생성 → `<out>/point_lio.log` 확인(토픽·per-point time 필드).

## 7. 문제해결
| 증상 | 조치 |
|---|---|
| 빌드 시 pcl 못 찾음 | `sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions` |
| `Failed to find match for field 'time'` | bag 점군에 per-point time 필드 없음(L2 정상 출력이면 발생 안 함) |
| 맵이 흐트러짐/드리프트 | 초기 정지 구간 확보, 급격한 이동 자제 |
```

- [ ] **Step 2: `CLAUDE.md` 현재 상태에 매핑 도구 반영**

`## 현재 상태` 섹션의 LiDAR 항목 다음에 아래 bullet 추가:

```markdown
- **오프라인 매핑**(bag→궤적·맵): Point-LIO(ROS2) 벤더링 `src/point_lio` + `mapping/`
  (`lio_map_bag.sh`·`pose_logger.py`·`pcd_preview.py`·`bev_grid.py`). `ros2 bag play` 기반
  후처리라 **실시간 수집·녹화 코드와 완전 분리**. 산출물 = `map.pcd`+`trajectory.tum`(ego-pose,
  BEV 전제)+미리보기. 절차는 `docs/MAPPING.md`.
```

`## 상세 문서` 목록에 추가:

```markdown
- **매핑 가이드**: `docs/MAPPING.md` (오프라인 LIO 실행·산출물·시각화·판정)
- 설계 스펙(매핑): `docs/superpowers/specs/2026-07-20-lio-mapping-integration-design.md`
```

- [ ] **Step 3: `docs/USAGE.md`에 매핑 요약 추가**

USAGE.md 끝(LiDAR 섹션 뒤)에 섹션 추가:

```markdown
## 10. 오프라인 매핑 (bag → 궤적·맵)

녹화된 bag에서 ego 궤적과 3D 맵을 복원한다(BEV 데이터 전제). 상세는 `docs/MAPPING.md`.

```bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions   # 최초 1회
colcon build --packages-select point_lio && source install/setup.bash
./mapping/lio_map_bag.sh <bag_경로> out/run01
```
산출물: `out/run01/{map.pcd, trajectory.tum, run_info.txt, preview/}`.
```

- [ ] **Step 4: 원본 clone 제거**

```bash
cd ~/Desktop/econ_camera_ws
rm -rf third_party/point_lio_ros2
test ! -e third_party/point_lio_ros2 && echo "removed OK"
```
(내용은 `src/point_lio`로 이관 완료. ROS1 `third_party/point_lio_unilidar`는 유지.)

- [ ] **Step 5: 전체 회귀 확인**

```bash
python3 -m pytest mapping/test/ -q
cd src/econ_camera_ros && python3 -m pytest test/ -q; cd ../..
```
Expected: mapping `3 passed`, 카메라 `18 passed`.

- [ ] **Step 6: 커밋**

```bash
git add docs/MAPPING.md CLAUDE.md docs/USAGE.md
git commit -m "$(cat <<'EOF'
docs(mapping): MAPPING.md 가이드 + USAGE/CLAUDE 반영, 원본 clone 정리

오프라인 매핑 실행·산출물·시각화·판정 문서화. third_party/point_lio_ros2
원본 clone 제거(src/point_lio로 이관 완료).

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Self-Review 체크

- **스펙 커버리지**: 벤더링(T1)·궤적 TUM(T2)·시각화(T3)·오케스트레이터+출력레이아웃(T4)·문서+정리(T5) — 스펙 §2~§8 모두 태스크로 매핑됨.
- **플레이스홀더**: 모든 코드/명령/기대결과 구체값으로 기재, TBD 없음.
- **타입 일관성**: `odom_to_tum_line` 시그니처(T2)와 호출부(pose_logger, T2) 일치; `pcd_preview.py`/`bev_grid.py` 인자 `<pcd> [out_dir]`(T3)와 오케스트레이터 호출(T4) 일치; `src/point_lio/PCD/scans.pcd` 경로(T4)와 벤더 config 기본값 일치.
- **제약**: 실시간 코드 무수정·벤더 소스 무수정·TUM 포맷·헤드리스·회귀·커밋 trailer 반영.
```
