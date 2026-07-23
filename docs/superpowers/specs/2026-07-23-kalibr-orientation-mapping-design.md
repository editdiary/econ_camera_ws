# orientation 기반 Kalibr 카메라 순서 자동화 설계

## 배경 / 문제

포트 연결 순서에 따라 카메라 번호(`/dev/video0~3` → `cam0~3` 토픽)가 세션마다 바뀔 수
있다. 그런데 Kalibr 다중 카메라 캘리브레이션은 `--topics`에 준 **순서대로 연속된 카메라
쌍이 물리적으로 겹친다고 가정**하고 baseline을 초기화한다. 우리 리그는 360° 서라운드
어안(약 180° FOV) 4대이며, **인접(90° 이웃)만 겹치고 반대편(180°)은 안 겹친다**.

따라서 번호가 뒤섞이면 `run_kalibr.sh`의 고정 순서(`cam0→cam1→cam2→cam3`)가 실제 물리
인접과 어긋나, 연속 쌍이 안 겹쳐 **extrinsic 계산이 실패**할 수 있다.

**목표:** 카메라 번호 ↔ 물리 방향 매핑 파일(`orientation.json`)을 근거로 (1) Kalibr에 넘길
토픽 순서를 링 순서로 자동 정렬하고, (2) 결과물(`calib.yaml`)을 번호가 아닌 **방향 이름**
(front/right/rear/left)으로 라벨링해, 재배선에도 결과가 강건하게 한다.

## 확정된 전제

- 물리 배치 = 360° 서라운드, 방향 4개가 90°씩. 어안 약 180° FOV.
- **인접(겹침) 링 순서 = `front → right → rear → left`**. 반대편(`front↔rear`,
  `left↔right`)은 안 겹침.
- 링 첫 요소 `front`가 extrinsic 기준(`T_*_front`)이 된다(물리적으로 고정된 자연 기준).
- `orientation.json`은 **수동 작성**한다(모니터 커버 테스트로 각 cam의 방향을 확인).

## 구성 요소

### 1. 신규 `src/econ_camera_ros/econ_camera_ros/cam_layout.py` (순수 로직)

```python
RING = ["front", "right", "rear", "left"]   # 인접 순서(반대편 미겹침), front = extrinsic 기준

def order_from_json(orientation: dict) -> list[tuple[int, str]]:
    """{"cam0":"left", ...} → 링 순서 [(cam_idx, direction), ...]. 검증 포함."""
```

- 반환 예: `{cam0:left, cam1:right, cam2:front, cam3:rear}`
  → `[(2,"front"), (1,"right"), (3,"rear"), (0,"left")]`
- CLI(`__main__`):
  - `python3 cam_layout.py orientation.json --topics`
    → `/cam2/image_raw /cam1/image_raw /cam3/image_raw /cam0/image_raw`
  - `python3 cam_layout.py orientation.json --names` → `front right rear left`
- **stdlib(json)만 사용** → `run_kalibr.sh`가 ROS 소싱 없이 파일 경로로 직접 호출 가능.

### 2. `calibration/run_kalibr.sh` 수정

- `DATA_DIR/orientation.json`이 **있으면**: `cam_layout.py --topics`로 `TOPICS` 유도.
- **없으면**: 기존 `/cam0../cam3` 순서로 폴백(하위호환).
- 스크립트 위치 기준 상대경로로 `cam_layout.py`를 찾는다
  (`SCRIPT_DIR/../src/econ_camera_ros/econ_camera_ros/cam_layout.py`).
- 실행 끝에 **매핑 표 출력**: `Kalibr cam0=front(/cam2)  cam1=right(/cam1) ...`
  → 사용자가 `results.txt`(Kalibr는 cam0~3으로 라벨)를 방향과 대조할 수 있게.

### 3. `src/econ_camera_ros/econ_camera_ros/calib_convert.py` 수정

- 순수 함수 시그니처에 라벨 인자 추가:
  `camchain_to_calib(camchain, model_chosen, reproj_rms=None, topic_to_dir=None)`.
  - `topic_to_dir`({rostopic: 방향})가 주어지면 각 카메라를 그 **rostopic 의 방향**으로 relabel
    (Kalibr cam 순서 가정 제거 → 순서가 어긋나도 올바르게 라벨). 기준은 정렬상 첫 카메라의 방향.
    → `cameras` 키·`extrinsics` 키(`T_<dir>_<ref>`)가 방향명이 됨.
  - `topic_to_dir=None`이면 **기존 cam0.. 동작 그대로**.
- `main()`에 `--orientation orientation.json` 옵션 추가:
  - 주면 `cam_layout.order_from_json`으로 `{"/cam{idx}/image_raw": 방향}` dict 를 만들어
    `topic_to_dir`로 전달.
  - `--rms`도 이때는 방향명으로 받음(`--rms front=1.6 right=..`).
  - 안 주면 기존대로 `camN` 사용.

### 4. 신규 `calibration/orientation.example.json`

사용자가 복사·편집하는 템플릿(`aprilgrid.yaml`을 DATA_DIR로 복사하는 흐름과 동일):

```json
{
  "cam0": "left",
  "cam1": "right",
  "cam2": "front",
  "cam3": "rear"
}
```

## 데이터 흐름

```
[커버 테스트] monitor 켜고 각 카메라를 손으로 가려 → cam 번호 ↔ 방향 확정
   │  cp calibration/orientation.example.json <DATA_DIR>/orientation.json  # 편집
run_kalibr.sh → cam_layout(--topics)로 링 순서 TOPICS → Kalibr(ds/eucm)
   │  끝에 "Kalibr camN = 방향(/camX)" 매핑 표 출력
calib_convert --orientation → calib.yaml (방향명 키, T_*_front, rms)
```

## 오류 처리

`order_from_json`이 다음을 **명확한 에러(ValueError, 한국어 메시지)로 거부**:

- 키가 정확히 `cam0, cam1, cam2, cam3`이 아님.
- 값이 `{front, right, rear, left}` 정확히 1개씩이 아님(누락·중복·오타·대소문자).

`run_kalibr.sh`는 `set -e`라 python이 비정상 종료하면 즉시 중단된다(부분 실패 방지).

## 테스트 (하드웨어 불필요)

- `test/test_cam_layout.py`:
  - 정상 매핑 → 기대 순서 `[(2,"front"),(1,"right"),(3,"rear"),(0,"left")]`.
  - 방향 누락 / 중복 / 오타 / 잘못된 키 → 각각 `ValueError`.
- `test/test_calib_convert.py`(기존 확장):
  - `names` 전달 시 `cameras`·`extrinsics`가 방향 키·`T_<dir>_front`로 나오는지.
  - `names=None`이면 기존 `camN` 결과 유지(회귀 방지).

## 문서

- `docs/CALIBRATION.md`: orientation.json 단계(커버 테스트 → 작성 → 자동 순서 → 방향 라벨
  결과) 추가. 기존 워크플로우(§1~6)에 삽입하고 TL;DR 갱신.

## 범위 밖 (YAGNI)

- `orientation.json` **자동 생성**(커버 테스트 툴화) — 수동 작성으로 충분.
- 4방향 외 임의 배치/카메라 개수 — 이 리그는 360° 4대 고정.
