# orientation 기반 Kalibr 카메라 순서 자동화 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 카메라 번호↔방향 매핑(`orientation.json`)으로 Kalibr 토픽 순서를 물리 인접 링 순서로 자동 정렬하고, 결과 `calib.yaml`을 방향 이름으로 라벨링한다.

**Architecture:** 순수 로직 모듈(`cam_layout.py`)이 매핑을 링 순서 `[(cam_idx, 방향)]`로 변환·검증한다. `run_kalibr.sh`는 이 모듈의 CLI로 토픽 순서를 유도하고(없으면 기존 cam0~3 폴백), `calib_convert.py`는 같은 모듈로 출력 cam 인덱스를 방향명으로 relabel한다.

**Tech Stack:** Python 3(stdlib json/argparse), bash, pytest, ROS2 Humble colcon 패키지 `econ_camera_ros`.

## Global Constraints

- 링 인접 순서: `["front", "right", "rear", "left"]` (반대편 `front↔rear`, `left↔right`은 안 겹침). 링 첫 요소 `front`가 extrinsic 기준.
- 순수 로직은 pytest로 검증(하드웨어 불필요). CLI 출력·bash 통합은 수동 검증.
- 기존 동작 하위호환: `orientation.json`/`--orientation` 미사용 시 기존 cam0~3 결과 그대로.
- 파일은 관심사별로 작게. 기존 코드 스타일(한국어 주석·docstring) 유지.
- 테스트 실행: `cd src/econ_camera_ros && python3 -m pytest test/`.
- 커밋 메시지 말미: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- 브랜치 병합·푸시는 사용자가 직접(Claude는 커밋까지만).

---

### Task 1: cam_layout 순수 모듈 + CLI

**Files:**
- Create: `src/econ_camera_ros/econ_camera_ros/cam_layout.py`
- Test: `src/econ_camera_ros/test/test_cam_layout.py`

**Interfaces:**
- Consumes: 없음(stdlib만).
- Produces:
  - `RING = ["front", "right", "rear", "left"]` (module constant).
  - `order_from_json(orientation: dict) -> list[tuple[int, str]]` — 링 순서 `[(cam_idx, 방향), ...]`. 검증 실패 시 `ValueError`.
  - CLI: `python3 cam_layout.py <json> --topics` → 링 순서 `/camN/image_raw` 공백구분 1줄. `--map` → `Kalibr cam{i} = {방향} (/cam{idx}/image_raw)` 여러 줄.

- [ ] **Step 1: 실패하는 테스트 작성**

Create `src/econ_camera_ros/test/test_cam_layout.py`:

```python
import pytest
from econ_camera_ros import cam_layout as cl


def test_order_from_json_ring_order():
    orientation = {"cam0": "left", "cam1": "right", "cam2": "front", "cam3": "rear"}
    assert cl.order_from_json(orientation) == [(2, "front"), (1, "right"), (3, "rear"), (0, "left")]


def test_order_from_json_identity_mapping():
    # cam0=front..cam3=left 이면 링 순서 인덱스가 [0,1,2,3] 그대로.
    orientation = {"cam0": "front", "cam1": "right", "cam2": "rear", "cam3": "left"}
    assert cl.order_from_json(orientation) == [(0, "front"), (1, "right"), (2, "rear"), (3, "left")]


def test_order_from_json_missing_direction():
    # rear 누락 + left 중복 → 무효.
    orientation = {"cam0": "left", "cam1": "right", "cam2": "front", "cam3": "left"}
    with pytest.raises(ValueError):
        cl.order_from_json(orientation)


def test_order_from_json_unknown_direction():
    orientation = {"cam0": "left", "cam1": "right", "cam2": "front", "cam3": "top"}
    with pytest.raises(ValueError):
        cl.order_from_json(orientation)


def test_order_from_json_wrong_keys():
    # cam3 없음(cam4 오타).
    orientation = {"cam0": "left", "cam1": "right", "cam2": "front", "cam4": "rear"}
    with pytest.raises(ValueError):
        cl.order_from_json(orientation)
```

- [ ] **Step 2: 실패 확인**

Run: `cd src/econ_camera_ros && python3 -m pytest test/test_cam_layout.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'econ_camera_ros.cam_layout'`

- [ ] **Step 3: 최소 구현 작성**

Create `src/econ_camera_ros/econ_camera_ros/cam_layout.py`:

```python
"""카메라 번호(cam0~3) ↔ 물리 방향 매핑으로 Kalibr 토픽 순서를 링 순서로 정렬.

360° 서라운드 어안 4대는 인접(90° 이웃)만 겹치고 반대편(180°)은 안 겹친다.
Kalibr 다중 카메라 캘리브레이션은 --topics 순서상 연속된 쌍이 겹친다고 가정하므로,
토픽을 물리 인접 링 순서(front-right-rear-left)로 넘겨야 extrinsic 이 풀린다.

orientation.json = {"cam0":"left", "cam1":"right", "cam2":"front", "cam3":"rear"}
(모니터 커버 테스트로 각 cam 의 방향을 확인해 수동 작성).

순수 로직(order_from_json)만 단위 테스트하고, CLI 출력은 run_kalibr.sh 에서 수동 검증.

실행:
    python3 -m econ_camera_ros.cam_layout orientation.json --topics
    python3 -m econ_camera_ros.cam_layout orientation.json --map
"""

import argparse
import json

RING = ["front", "right", "rear", "left"]   # 인접 순서(반대편 미겹침), front = extrinsic 기준
_CAMS = ["cam0", "cam1", "cam2", "cam3"]


def order_from_json(orientation):
    """{"cam0":"left", ...} → 링 순서 [(cam_idx, direction), ...]. 검증 포함."""
    if sorted(orientation) != _CAMS:
        raise ValueError(f"orientation 키는 정확히 {_CAMS} 여야 함 (받음: {sorted(orientation)})")
    dirs = [orientation[c] for c in _CAMS]
    if sorted(dirs) != sorted(RING):
        raise ValueError(f"orientation 값은 {RING} 각 1개씩이어야 함 (받음: {dirs})")
    dir_to_idx = {orientation[c]: int(c[3:]) for c in _CAMS}
    return [(dir_to_idx[d], d) for d in RING]


def main():
    p = argparse.ArgumentParser(description="orientation.json → Kalibr 토픽 순서")
    p.add_argument("orientation", help="cam↔방향 매핑 json")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--topics", action="store_true", help="링 순서 /camN/image_raw 목록")
    g.add_argument("--map", action="store_true", help="Kalibr cam인덱스 ↔ 방향 ↔ 토픽 표")
    a = p.parse_args()

    with open(a.orientation) as f:
        order = order_from_json(json.load(f))
    if a.topics:
        print(" ".join(f"/cam{idx}/image_raw" for idx, _ in order))
    else:
        for i, (idx, d) in enumerate(order):
            print(f"Kalibr cam{i} = {d} (/cam{idx}/image_raw)")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 통과 확인**

Run: `cd src/econ_camera_ros && python3 -m pytest test/test_cam_layout.py -v`
Expected: PASS (5 passed)

- [ ] **Step 5: CLI 수동 확인**

```bash
cd src/econ_camera_ros
printf '{"cam0":"left","cam1":"right","cam2":"front","cam3":"rear"}' > /tmp/orient.json
python3 econ_camera_ros/cam_layout.py /tmp/orient.json --topics
python3 econ_camera_ros/cam_layout.py /tmp/orient.json --map
```
Expected:
```
/cam2/image_raw /cam1/image_raw /cam3/image_raw /cam0/image_raw
Kalibr cam0 = front (/cam2/image_raw)
Kalibr cam1 = right (/cam1/image_raw)
Kalibr cam2 = rear (/cam3/image_raw)
Kalibr cam3 = left (/cam0/image_raw)
```

- [ ] **Step 6: 커밋**

```bash
git add src/econ_camera_ros/econ_camera_ros/cam_layout.py src/econ_camera_ros/test/test_cam_layout.py
git commit -m "feat(calib): cam_layout — orientation.json → 링 순서 토픽 정렬

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: calib_convert 방향명 relabel

**Files:**
- Modify: `src/econ_camera_ros/econ_camera_ros/calib_convert.py`
- Test: `src/econ_camera_ros/test/test_calib_convert.py` (기존 파일에 테스트 추가)

**Interfaces:**
- Consumes: `cam_layout.order_from_json` (Task 1).
- Produces: `camchain_to_calib(camchain, model_chosen, reproj_rms=None, names=None)` — `names`가 리스트면 정렬된 cam0..N의 i번째를 `names[i]`로 relabel(방향명 키, `T_<dir>_<names[0]>` extrinsic). `None`이면 기존 `camN` 동작.

- [ ] **Step 1: 실패하는 테스트 추가**

Append to `src/econ_camera_ros/test/test_calib_convert.py`:

```python
def test_camchain_to_calib_with_direction_names():
    I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    t01 = [[1, 0, 0, 0.1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    camchain = {
        "cam0": _cam("ds", [0.1, 0.6, 700, 700, 640, 360], "/cam2/image_raw"),
        "cam1": _cam("ds", [0.1, 0.6, 701, 701, 641, 361], "/cam1/image_raw", t01),
    }
    names = ["front", "right", "rear", "left"]
    out = cvt.camchain_to_calib(camchain, "ds", None, names)
    # cam0→front, cam1→right 로 라벨. 기준은 names[0]=front.
    assert set(out["cameras"]) == {"front", "right"}
    assert out["extrinsics"]["T_front_front"] == I
    assert out["extrinsics"]["T_right_front"] == t01
```

- [ ] **Step 2: 실패 확인**

Run: `cd src/econ_camera_ros && python3 -m pytest test/test_calib_convert.py::test_camchain_to_calib_with_direction_names -v`
Expected: FAIL — `TypeError: camchain_to_calib() takes ... positional arguments` (4번째 인자 미지원)

- [ ] **Step 3: 최소 구현 — `camchain_to_calib`에 `names` 인자 추가**

In `src/econ_camera_ros/econ_camera_ros/calib_convert.py`, replace the `camchain_to_calib` function (lines 22-44) with:

```python
def camchain_to_calib(camchain, model_chosen, reproj_rms=None, names=None):
    """Kalibr camchain dict → 프로젝트 calib dict.

    names 가 리스트면 정렬된 cam0..N 을 names[i] 로 relabel(방향명 키·T_<dir>_<names[0]>).
    None 이면 기존 camN 라벨.
    """
    cams = sorted(camchain)
    labels = names if names is not None else cams
    ref = labels[0]
    cameras, extrinsics = {}, {}
    cum = [[1 if i == j else 0 for j in range(4)] for i in range(4)]  # T_ref_ref = I
    for idx, name in enumerate(cams):
        c = camchain[name]
        label = labels[idx]
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
```

- [ ] **Step 4: 통과 + 회귀 확인**

Run: `cd src/econ_camera_ros && python3 -m pytest test/test_calib_convert.py -v`
Expected: PASS (기존 3개 + 신규 1개 모두 통과 — `names=None` 기본이라 기존 테스트 회귀 없음)

- [ ] **Step 5: `main()`에 `--orientation` 옵션 배선**

In `calib_convert.py`, add `import json` at the top (아래 `import argparse` 다음 줄):

```python
import argparse
import json
```

Then replace `main()` (from `def main():` to end of file) with:

```python
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
    names = None
    if a.orientation:
        with open(a.orientation) as f:
            names = [d for _, d in order_from_json(json.load(f))]
    rms = {}
    for kv in a.rms:
        k, v = kv.split("=")
        rms[k] = float(v)
    calib = camchain_to_calib(camchain, a.model, rms, names)
    with open(a.out, "w") as f:
        yaml.safe_dump(calib, f, sort_keys=False)
    print(f"calib 작성 → {a.out} (model={a.model}, cams={list(calib['cameras'])})")


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: 전체 테스트 통과 확인**

Run: `cd src/econ_camera_ros && python3 -m pytest test/ -v`
Expected: PASS (기존 18개 + 신규 6개 = 24개 통과)

- [ ] **Step 7: 커밋**

```bash
git add src/econ_camera_ros/econ_camera_ros/calib_convert.py src/econ_camera_ros/test/test_calib_convert.py
git commit -m "feat(calib): calib_convert --orientation 으로 방향명 라벨링

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: run_kalibr.sh 통합 + 예시 파일

**Files:**
- Modify: `calibration/run_kalibr.sh`
- Create: `calibration/orientation.example.json`

**Interfaces:**
- Consumes: `cam_layout.py` CLI(`--topics`, `--map`) (Task 1). 상대경로 `SCRIPT_DIR/../src/econ_camera_ros/econ_camera_ros/cam_layout.py`.
- Produces: `orientation.json`이 있으면 링 순서 `--topics`로 Kalibr 실행 + 끝에 매핑 표 출력. 없으면 기존 cam0~3.

- [ ] **Step 1: 예시 파일 생성**

Create `calibration/orientation.example.json`:

```json
{
  "cam0": "left",
  "cam1": "right",
  "cam2": "front",
  "cam3": "rear"
}
```

- [ ] **Step 2: `run_kalibr.sh` 상단 — TOPICS 유도 로직으로 교체**

In `calibration/run_kalibr.sh`, replace lines 14-16:

```bash
DATA_DIR="$(realpath "${1:?usage: run_kalibr.sh <DATA_DIR>}")"
WS="/catkin_ws"   # 컨테이너 내 catkin workspace (빌드 로그로 확인됨)
TOPICS="/cam0/image_raw /cam1/image_raw /cam2/image_raw /cam3/image_raw"
```

with:

```bash
DATA_DIR="$(realpath "${1:?usage: run_kalibr.sh <DATA_DIR>}")"
WS="/catkin_ws"   # 컨테이너 내 catkin workspace (빌드 로그로 확인됨)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LAYOUT="$SCRIPT_DIR/../src/econ_camera_ros/econ_camera_ros/cam_layout.py"

# orientation.json 이 있으면 링 순서(front-right-rear-left)로 토픽 정렬(포트 재연결로 cam
# 번호가 바뀌어도 인접 카메라가 --topics 순서상 연속되게). 없으면 기존 cam0~3 순서.
if [ -f "$DATA_DIR/orientation.json" ]; then
  TOPICS="$(python3 "$LAYOUT" "$DATA_DIR/orientation.json" --topics)"
  echo "orientation.json 적용 → Kalibr 토픽 순서: $TOPICS"
else
  TOPICS="/cam0/image_raw /cam1/image_raw /cam2/image_raw /cam3/image_raw"
  echo "orientation.json 없음 → 기본 순서 cam0~3 사용"
fi
```

- [ ] **Step 3: `run_kalibr.sh` 말미 — 매핑 표 출력 추가**

In `calibration/run_kalibr.sh`, replace the last line:

```bash
echo "DONE: /data 에 ds/eucm camchain·results·report 생성."
```

with:

```bash
echo "DONE: /data 에 ds/eucm camchain·results·report 생성."
if [ -f "$DATA_DIR/orientation.json" ]; then
  echo "── Kalibr cam 인덱스 ↔ 방향 (results.txt 대조 / calib_convert --rms 입력용) ──"
  python3 "$LAYOUT" "$DATA_DIR/orientation.json" --map | sed 's/^/  /'
fi
```

- [ ] **Step 4: 구문 검사 + 폴백 스모크 테스트(Docker 불필요)**

`bash -n`으로 구문만 검사하고, orientation 유도 로직만 분리 확인:

```bash
bash -n calibration/run_kalibr.sh && echo "syntax OK"
# 유도 로직 단독 확인(실제 Kalibr/Docker 없이 TOPICS 계산만):
LAYOUT="calibration/../src/econ_camera_ros/econ_camera_ros/cam_layout.py"
printf '{"cam0":"left","cam1":"right","cam2":"front","cam3":"rear"}' > /tmp/orient.json
python3 "$LAYOUT" /tmp/orient.json --topics
python3 "$LAYOUT" /tmp/orient.json --map
```
Expected: `syntax OK` + Task 1 Step 5와 동일한 topics/map 출력.

> 실제 Docker 관통(orientation.json을 DATA_DIR에 두고 `sudo bash calibration/run_kalibr.sh <DATA_DIR>`)은 카메라 촬영 데이터가 필요하므로 실기에서 수동 검증한다.

- [ ] **Step 5: 커밋**

```bash
git add calibration/run_kalibr.sh calibration/orientation.example.json
git commit -m "feat(calib): run_kalibr.sh orientation.json 링 순서 자동 적용 + 매핑 표

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: 문서(CALIBRATION.md) 갱신

**Files:**
- Modify: `docs/CALIBRATION.md`

**Interfaces:**
- Consumes: Task 1~3의 최종 사용법.
- Produces: orientation 단계가 반영된 운영 가이드.

- [ ] **Step 1: TL;DR에 orientation 단계 삽입**

In `docs/CALIBRATION.md`, in the TL;DR code block, replace these two lines:

```bash
ros2 run econ_camera_ros kalibr_bridge extracted -o dataset --rate 4.0
cp calibration/aprilgrid.yaml .
```

with:

```bash
ros2 run econ_camera_ros kalibr_bridge extracted -o dataset --rate 4.0
cp calibration/aprilgrid.yaml .
cp calibration/orientation.example.json orientation.json   # 커버 테스트로 확인한 방향으로 편집
```

And replace the `calib_convert` line in TL;DR:

```bash
ros2 run econ_camera_ros calib_convert calib-camchain-ds.yaml \
  --model ds -o calib.yaml --rms cam0=.. cam1=.. cam2=.. cam3=..
```

with:

```bash
ros2 run econ_camera_ros calib_convert calib-camchain-ds.yaml \
  --model ds -o calib.yaml --orientation orientation.json \
  --rms front=.. right=.. rear=.. left=..
```

- [ ] **Step 2: §3(데이터셋) 뒤에 orientation 소절 추가**

In `docs/CALIBRATION.md`, immediately after the §3 code block (the `kalibr_bridge` + `cp calibration/aprilgrid.yaml .` block, ending with the `→ dataset/...` 설명 줄), add:

````markdown

### 3.1 카메라 방향 매핑 (`orientation.json`) — 포트 재연결 대비

포트 연결에 따라 cam 번호(`/dev/video0~3`)가 바뀔 수 있다. 360° 어안 4대는 **인접(90°)만
겹치고 반대편은 안 겹치므로**, Kalibr `--topics` 순서상 연속된 카메라가 물리적으로 인접해야
extrinsic 이 풀린다. 이를 매 촬영마다 신경 쓰지 않도록 **방향 매핑 파일**을 둔다.

```bash
cp calibration/orientation.example.json orientation.json   # dataset/ 옆에 두고 편집
```

**어느 cam이 어느 방향인지 확인:** `monitor`(2×2 화면)를 켜고 카메라를 하나씩 손으로 가려,
어두워지는 칸의 cam 번호를 그 방향에 적는다.

```json
{"cam0": "left", "cam1": "right", "cam2": "front", "cam3": "rear"}
```

- 값은 정확히 `front/right/rear/left` 각 1개씩. `run_kalibr.sh`가 이 파일을 읽어 링 순서
  `front→right→rear→left`로 토픽을 정렬한다. **파일이 없으면 기존 cam0~3 순서로 폴백**.
- 실행 끝에 `Kalibr cam{i} = 방향 (/camN)` 매핑 표가 출력된다 → `results.txt`(Kalibr는
  cam0~3 라벨)를 방향과 대조하고, `calib_convert --rms` 입력에 사용.
````

- [ ] **Step 3: §4 실행 설명에 매핑 표 언급 추가**

In `docs/CALIBRATION.md` §4, after the sentence describing the outputs (`calib-camchain-eucm.yaml ...` 블록 뒤), add a line:

```markdown
> `orientation.json`을 두면 링 순서로 자동 정렬되고, 실행 끝에 **Kalibr cam↔방향 매핑 표**가
> 출력된다. `results.txt`의 `cam0~3`은 이 표 기준(링 순서: front·right·rear·left)임에 유의.
```

- [ ] **Step 4: §6(`calib.yaml` 생성)을 방향명 흐름으로 갱신**

In `docs/CALIBRATION.md` §6, replace the command block:

```bash
ros2 run econ_camera_ros calib_convert calib-camchain-ds.yaml \
  --model ds -o calib.yaml \
  --rms cam0=6.0 cam1=1.6 cam2=3.0 cam3=3.0
```

with:

```bash
ros2 run econ_camera_ros calib_convert calib-camchain-ds.yaml \
  --model ds -o calib.yaml --orientation orientation.json \
  --rms front=1.6 right=3.0 rear=3.0 left=6.0     # 매핑 표 보고 방향별로
```

And after the `**calib.yaml 구조:**` example block, add a note:

```markdown
> `--orientation`을 주면 `cameras`·`extrinsics` 키가 `cam0~3` 대신 **방향명**(front/right/rear/
> left)이 되고 기준은 `front`(`T_right_front` 등)다. 카메라 번호가 바뀌어도 결과 해석이
> 안 흔들린다. 생략하면 기존 `camN` 라벨 그대로.
```

- [ ] **Step 5: §7 도구 레퍼런스 표에 cam_layout 추가**

In `docs/CALIBRATION.md` §7 표에, `aprilgrid.yaml` 행 아래에 추가:

```markdown
| `cam_layout` | `econ_camera_ros/` | orientation.json → 링 순서 토픽 정렬(run_kalibr·calib_convert 공유) |
| `orientation.example.json` | `calibration/` | cam↔방향 매핑 템플릿(복사·편집) |
```

- [ ] **Step 6: 문서 확인 + 커밋**

Run: `grep -n "orientation" docs/CALIBRATION.md | head`
Expected: TL;DR·§3.1·§4·§6·§7에 orientation 언급이 보임.

```bash
git add docs/CALIBRATION.md
git commit -m "docs(calib): orientation.json 방향 매핑 단계·방향명 라벨 반영

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Self-Review 결과

- **Spec coverage:** cam_layout 모듈(§구성1)=Task1, calib_convert relabel(§구성3)=Task2, run_kalibr.sh+예시(§구성2,4)=Task3, 문서(§문서)=Task4, 데이터 흐름·오류처리·테스트(§)=Task1~2 테스트로 커버. 누락 없음.
- **Placeholder scan:** 코드·명령·기대출력 모두 구체값. 문서 `--rms .. ` 는 사용자 실측 입력값 자리로 의도된 것(플레이스홀더 아님).
- **Type consistency:** `order_from_json` 반환 `list[(int,str)]`을 Task2 main(`[d for _,d in ...]`)·Task3 CLI가 동일하게 소비. `camchain_to_calib(camchain, model, rms, names)` 4-인자 시그니처가 Task2 테스트·main 호출과 일치. `RING`·`_CAMS` 상수명 일관.
- **CLI 범위:** spec의 `--names`는 소비처가 없어 `--map`으로 대체(run_kalibr는 `--topics`·`--map`만 사용, calib_convert는 모듈 import). YAGNI 반영.
