# BEV Auto-Label 품질 개선 설계 (수직성·카메라 FoV·정밀 ego)

> **목적**: `docs/BEV_AUTOLABEL.md` PoC의 라벨 품질(특히 **obstacle 경계 정확도**)을 개선한다.
> PoC에서 "이미지엔 길이 보이는데 라벨은 ignore", "장애물이 있는데 빈칸"으로 나오던 오류를 줄인다.
> 본 문서는 **변경분만** 규정하고, 바뀌지 않는 부분(BEV 규격·좌표계·입력 데이터·투영 체인)은 `BEV_AUTOLABEL.md`를 따른다.
> 작성일 2026-07-27, 브랜치 `feat/bev-autolabel`.

---

## 0. 문제 정의 (개선 대상)

PoC(`BEV_AUTOLABEL.md` 부록)는 다음 방식이었다:
- obstacle = `z ∈ [floor+0.1, floor+1.0]` **고정 낮은밴드** 셀 카운트, floor = **전역** 2-퍼센타일.
- visible = ego에서 **360° LiDAR ray-cast**(카메라와 무관), 첫 장애물까지.
- ego(카트) 제거 = body 좌표 **반경 0.65m** 지속성 self-mask.

관측된 불만(사용자 검증):
- **obstacle 경계 부정확** — 작물행이 통째로 누락되거나(고정밴드가 상단만 잡힌 작물을 놓침), floor가 기울면 밴드가 어긋남.
- **"길인데 ignore"** — 부실/엉뚱한 obstacle 셀이 ray-cast를 막아 그 뒤가 가려짐 처리됨.
- **"안 보이는 후방까지 drivable"** — 360° ray-cast가 카메라 시야 밖(후방)도 관측으로 처리.
- **가까운 기둥이 ego로 오인 삭제** — self-mask 반경 0.65m가 0.7m 옆 작물벽/기둥을 먹음.

## 1. 개선 원칙 (확정)

1. **라벨은 LiDAR+맵 기하로만 생성**한다. 이미지에서 바닥/작물을 의미 분할해 라벨에 쓰지 않는다
   (그건 학습 대상 모델 자체라 순환). 이미지는 **observed(FoV) 마스크**와 **사람 검수**에만 쓴다.
2. **drivable은 "바닥을 봐서"가 아니라 "장애물이 없고 + 보이니까"** 로 정의한다(LiDAR 바닥 사각 대응).
   → 근접 바닥은 LiDAR 점이 0이라 obstacle이 안 생기고, 카메라가 보므로 drivable로 확정된다.
   한계: 수직 구조가 짧은 **키 작은 장애물**(연석·박스)은 LiDAR가 못 잡아 오검(drivable)될 수 있다(온실 통로에선 드묾).
3. **"보이는 곳만"** — observed는 **카메라 FoV ∩ 가림 없음**으로 정의한다. 나머지는 ignore.

## 2. 알고리즘 변경 (PoC Step 4~12 교체)

BEV 규격(80×80, `XF=3.0/XR=1.0/YH=2.0/RES=0.05`, ego셀 (60,40)), self-mask **지속성 판정 구조**,
좌표계·투영 체인(`calibration/cam_lidar/chain.py`)은 **유지**.

### A. obstacle — 수직성(column) 테스트 + 국소 floor
- **국소 floor 격자**: 전역 2-퍼센타일 대신, BEV를 ~1.0m 윈도로 나눠 각 윈도의 저-퍼센타일 z로
  floor 격자를 만들고 셀 단위로 보간. 바닥이 기울어도 판정 밴드가 어긋나지 않는다.
- **셀별 column 판정**: 셀에 속한 점들의 z에 대해
  `(z_min ≤ floor_cell + 0.3m)` **AND** `(z_max − z_min ≥ 0.5m)` 이면 obstacle 후보.
  → 바닥까지 이어지는 수직구조(작물·기둥)만 잡고, 높이만 떠 있는 **천장·캐노피는 배제**(z_min 높음),
  **단발 지면 노이즈**도 배제(수직 extent 없음).
- 셀당 최소 점수 임계 + `morphologyEx(OPEN)→(CLOSE)` (3×3)로 스펙클·틈 정리(경계 보존 위해 약하게).
- **근거**: 작물은 수직이라 상단만 잡혀도 그 (x,y)=밑동 (x,y). "낮은 점 존재"는 천장 배제용 게이트로만 쓰고,
  자격 셀은 상단 점 포함해 footprint를 찍는다. 맵은 다시점 누적이라 근접 밑동이 다른 시점에서 채워진다.

### B. observed — 카메라 FoV ∩ 가림 없음 (360° ray-cast 대체)
- **FoV 마스크**: 각 BEV 셀 중심을 지면점 `(x, y, floor_cell)`으로 두고, 3대 어안(front/left/right)에
  `chain.project`로 역투영. 이미지 경계 안 & DS 유효 시야각(θ) 안이면 그 카메라 FoV에 포함.
  `fov = front ∪ left ∪ right`. (rear 카메라는 사용 안 함 → 후방은 자동 미포함.)
- **가림 마스크**: ego셀 (60,40)에서 각 셀로 0.5° 간격 ray-cast, 도중 obstacle을 만나면 그 뒤는 미방문.
  `raycast_visible`. (섬 방지 위해 ray-cast 단독, `BEV_AUTOLABEL.md §Step10` 교훈 유지.)
- **`observed = fov AND raycast_visible`**. 카메라 없는 후방·가려진 뒤쪽은 ignore.

### C. ego 정밀 제거
- self-mask 지속성 판정(150 pose 샘플, >60% 지속, 0.15m 복셀)은 유지하고 **반경만 0.65m → 0.28m**
  (40×40cm의 반대각 ≈ 0.283m). 카트만 잡고 0.7m 옆 작물벽/기둥은 반경 밖이라 보존.

### D. 전방 corridor (drivable prior) — 유지
- 시각 `t` ±20s 궤적의 **전방(x ≥ -0.2)** pose마다 반경 0.45m 원 + ego 반경 0.3m 원을 drivable로.
- 단, **`observed` 안에서만** 적용(안 보이는 전방까지 강제 drivable 하지 않음).

### E. 라벨 조립 + 정리
- 기본 `2(ignore)`.
- `observed & obstacle → 0(obstacle)`, `observed & ~obstacle → 1(drivable)`, `corridor & observed → 1`.
- **마무리(신규)**: ego셀과 연결된 drivable 성분만 남겨 잔여 pocket/섬 제거(PoC §12 TODO 반영).

## 3. 산출물 (2단계)

### 단계 1 — 품질 검증 (먼저)
- PoC 시각화 스크립트를 위 알고리즘으로 교체. 입력 = LIO 맵 폴더 + 추출 이미지 폴더 + calib.
- 출력(검증용): **3이미지 + BEV 합성 PNG**(0=빨강/1=초록/2=회색) + `overlay_diag`(LiDAR-on-image 높이색).
- 검증 프레임: `raws3_mapping` 기준 **900·2000·2500·4850**(멀티통로·중앙·중앙분기·교차부).
  여기서 의도대로 나올 때까지 파라미터 튜닝. **튜닝된 파라미터는 본 문서 §6에 기록**.
- **품질 판정 기준**: ① obstacle이 작물행 경계를 잘 물음 ② 카메라 안 보이는 후방/가림이 회색
  ③ 통로가 초록 ④ 가까운 기둥이 ego로 안 지워짐.

### 단계 2 — CLI (품질 확정 후)
- `bev_autolabel` CLI: `--map-dir --extract-dir --calib --orient --out --kf-step`(기본 0.4m 이동) + 파라미터 플래그.
- 키프레임: 이동거리 **0.4m** 간격.
- **출력 1 샘플** `sample_NNNNNN/`:
  - `label.png` — 깨끗한 80×80 **인덱스 팔레트 PNG**(픽셀 인덱스=클래스 0/1/2, 팔레트가 색 매핑 →
    화면에선 색으로 구분되고 값은 0/1/2 그대로), **오버레이 없음**(재라벨링 원본).
  - `review.png` — label 확대 + **미터 축·0.5m 격자·ego 위치·전방 화살표** 오버레이 + 3카메라 이미지(거리 판단용 검수뷰).
  - `cam_front.jpg / cam_left.jpg / cam_right.jpg` — sets.csv stamp 매칭 원본.
  - `meta.json` — ego-pose(world_T_body), stamp, BEV 규격(RES 등), calib 참조 경로.
- `dataset.csv` — 샘플 인덱스 ↔ stamp ↔ pose 요약.

## 4. 검증 방법

- **순수 로직 pytest**: 수직성 판정, ray-cast, ego 마스크, 좌표변환/셀 인덱싱을 하드웨어 없이 단위 검증
  (기존 `src/econ_camera_ros/test/`·`calibration/*/test_*.py` 관례 따름).
- **라벨 품질**: §3 단계1의 4프레임 육안 검증(판정 기준 위 참조).
- **환경 핀**: numpy<1.25 / scipy 시스템 고정 유지. PCD는 open3d 0.18(user-site). user-site에 numpy≥2 금지
  (`BEV_AUTOLABEL.md §7-I`).

## 5. 의존성·경로

- 모듈: `calibration/cam_lidar/{chain,cloud_io,calib_io}.py`, `calibration/verify/ds_model.py`(`load_rig`). (본 브랜치에 존재 확인됨.)
- calib: `data/calib_260723/calib.yaml` + `orientation.json`(`cam0=front,cam1=right,cam2=rear,cam3=left`).
- 검증 데이터: LIO 맵 `data/sj_bags/260722/raws3_mapping/`(map.pcd + trajectory.tum),
  추출 이미지 폴더(sets.csv 포함) — 구현 착수 시 raws3에 대응하는 추출 폴더 확정.

## 6. 튜닝 파라미터 기록 (구현 중 갱신)

| 이름 | 초기값 | 의미 |
|---|---|---|
| floor 윈도 | 1.0 m | 국소 floor 격자 |
| column z_min 게이트 | floor_cell + 0.3 m | 바닥까지 이어짐 |
| column 수직 extent | ≥ 0.5 m | 수직구조 판정 |
| obstacle 셀 임계 | ≥ 2 점 | + morph open/close(3×3) |
| ego 반경 | 0.28 m | 40×40 반대각, 카트만 |
| corridor 반경 | 0.45 m(궤적)/0.3 m(ego) | observed 내에서만 |
| ray-cast | 0.5° 간격 | 가림 |
| 키프레임 | 0.4 m 이동 | CLI |

> 표의 값은 단계1 튜닝 후 확정치로 갱신한다.
