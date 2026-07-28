# BEV 자동 라벨링 파이프라인 (Camera→BEV Occupancy 데이터셋)

> **이 문서의 목적**: 4대 어안 이미지로 **BEV 주행가능(occupancy) 맵**을 추론하는 모델(BEVFormer류)의
> **학습 정답(auto-label)** 을, 현장에서 수집한 ROS bag + 그 bag의 LIO 맵으로부터 자동 생성하는
> 파이프라인을 상세히 기록한다. 새 세션에서 바로 이어받아 작업할 수 있도록 규격·단계·특이사항을 모두 적는다.
> PoC로 실데이터 검증 완료(2026-07-24). **CLI 구현·다중 bag 검증 완료(2026-07-27)** →
> 실행법·PoC 대비 변경사항은 아래 **§A(실행 가이드)**, 상세 설계·근거는
> `docs/superpowers/specs/2026-07-27-bev-autolabel-quality-improvement-design.md`. §1~§10은 개념·PoC 서술이며,
> 최종 구현은 §A의 변경사항 표를 따른다(수치가 다르면 §A·스펙이 최신).

---

## 0. 한눈 요약

- **입력**: ① 카메라4+LiDAR가 함께 녹화된 bag(`record_all.launch.py` 산출), ② 그 bag을 Point-LIO로 매핑한 폴더
  (`map.pcd` + `trajectory.tum`), ③ 캘리브(`calib.yaml` + `orientation.json`).
- **출력(1 샘플)**: `{3장 이미지(front/left/right), BEV 라벨(80×80), ignore 마스크, calib, ego-pose}`.
- **라벨 3-클래스**: `0=obstacle(주행불가)`, `1=drivable(주행가능)`, `2=ignore(가려짐/미관측, 학습 제외)`.
- **핵심 아이디어**: LiDAR는 **바닥을 못 보고 장애물(수직 구조)만 잘 본다**. 그래서
  **"장애물 footprint = obstacle, 그 외 보이는 곳 = drivable, 가려진 곳 = ignore"** 로 정의한다.
- **사람 검수**: auto-label은 초안이다. 이후 사람이 BEV 위에서 검수·보정해 최종 데이터셋을 만든다.

---

## A. 실행 가이드 (CLI) & 구현 상태 — **먼저 읽기**

### A.0 구현 상태 (2026-07-27)
- 파이프라인 CLI 구현 완료: **`calibration/bev_autolabel/`**
  - `bev_label.py`(순수 라벨 로직), `bev_io.py`(맵·stamp·이미지 IO), `render.py`(색칠·미터축 검수뷰),
    `verify_labels.py`(**단계1** 검증 CLI), `generate.py`(**단계2** 데이터셋 CLI), `test_bev_label.py`(21 테스트).
  - 재사용: `calibration/cam_lidar/{chain,cloud_io,calib_io}.py`, `calibration/verify/ds_model.py`.
- 검증: raws3 16프레임 + 타 bag 4종(raws1/raws2/rawos2/rawos4, with/without-sun) 일관 확인. 순수 로직 테스트 21+18+7 pass.

### A.1 PoC(§6) 대비 최종 변경사항 — **§6·§8보다 이 표가 최신**
| 항목 | PoC 서술(§6) | 최종 구현 |
|---|---|---|
| obstacle 판정 | 고정 낮은밴드 `[floor+0.1, 1.0]` | **수직성 테스트**: 셀의 `z_min ≤ floor_cell+z_gate`(기본 0.3) **AND** `z_max−z_min ≥ 0.5m` |
| floor | 전역 z 2퍼센타일 | **국소 격자**(~1m 윈도별 2퍼센타일, 빈 윈도 전역 백필) |
| observed(보이는 곳) | 360° ray-cast 단독 | **카메라 FoV(front/left/right 지면점 투영) ∩ ray-cast 가림** |
| ego self 반경 | <0.65 m | **<0.28 m**(40×40cm 반대각; 옆 기둥 보존) |
| corridor(주행 궤적) | 전방만, observed 게이트 | **무조건 drivable**(ground-truth) — 마스트-아래 ego 지면점이 카메라 FoV 가장자리라 observed가 ego에서만 False가 되는 artifact 때문에 게이트 시 `keep_ego_connected`가 drivable을 전멸시킴 |
| 마무리 | (TODO) | **ego 연결 성분만 남김**(`keep_ego_connected`) 구현 |
| min_pts/min_extent | 실질 레버로 서술 | 이 데이터에선 무효(맵 조밀·작물 키큼). **z_gate가 유일한 실질 레버** |

### A.2 사전 준비 (bag당 1회)
1. **매핑**: bag → `map.pcd` + `trajectory.tum` (Point-LIO, `docs/MAPPING.md`).
2. **이미지 추출**: bag → `frame_NNNNNN/cam{0..3}.jpg` + `sets.csv`
   ```bash
   source /opt/ros/humble/setup.bash
   python3 src/econ_camera_ros/econ_camera_ros/bag_extract.py <bag폴더> -o <추출폴더> [--limit N]
   ```
3. **calib**: `calib.yaml`(DS intrinsic 4대 + `T_cam_front` + **`T_front_lidar`**) + `orientation.json`.
   `T_front_lidar` 키가 없으면 CLI가 명확한 메시지로 즉시 종료된다(Cam-LiDAR 캘리브 선행 필요).

### A.3 단계1 — 품질 검증뷰 (특정 프레임 몇 개)
장면↔라벨 육안 확인용 합성 PNG(3이미지 + BEV, 미터축·0.5m 격자·ego 40×40 박스). ROS 소스 불필요.
```bash
cd calibration/bev_autolabel
mkdir -p <출력폴더>            # verify_labels 는 --out 을 자동 생성하지 않음
python3 verify_labels.py \
  --map-dir     ../../data/sj_bags/260722/maps/raws3_mapping \
  --extract-dir ../../data/extracted/raws3 \
  --calib       ../../data/calib_260723/calib.yaml \
  --orient      ../../data/calib_260723/orientation.json \
  --frames      900 2000 2500 4850 \
  --out         ../../data/bev/review/raws3
```
→ `<출력>/bev_review_NNNNNN.png`. (z_gate=0.3 고정)

### A.4 단계2 — 데이터셋 일괄 생성 (키프레임 전체)
```bash
cd calibration/bev_autolabel
python3 generate.py \
  --map-dir     ../../data/sj_bags/260722/maps/raws3_mapping \
  --extract-dir ../../data/extracted/raws3 \
  --calib       ../../data/calib_260723/calib.yaml \
  --orient      ../../data/calib_260723/orientation.json \
  --out         ../../data/bev/dataset/raws3 \
  --kf-step 0.4        # 키프레임 이동거리 간격(m)
  # --z-gate 0.3       # obstacle 바닥근접 여유(기본 0.3; 0.15면 통로 더 개방)
  # --limit 3          # 스모크: 앞 N개만
```
→ `<출력>/sample_NNNNNN/` 마다:
- **`label.png`** — 순수 class(0/1/2) **인덱스 팔레트**(오버레이 없음) = **재라벨링 원본**.
- **`review.png`** — ego 40×40 박스·미터축 검수뷰(사람 검수용).
- **`cam_{front,left,right}.jpg`** — 원본 3이미지.
- **`meta.json`** — pose(`world_T_body`)·stamp·BEV 규격·사용 파라미터(z_gate·kf_step).
- 그리고 최상위 **`dataset.csv`**(sample↔frame_idx↔stamp). 이미지 결손 키프레임은 건너뛰고 번호는 연속 유지.

### A.5 파라미터 조정
`--z-gate`(obstacle 바닥근접 여유)만 실질적 레버다. 0.6은 통로 위 캐노피 오검(비추), 0.15는 통로 개방, 0.3 절충(기본).
`min_pts`·`min_extent`는 이 환경에선 무효. BEV 범위·해상도(XF/XR/YH/RES)는 `bev_label.BevSpec` 기본값 고정.

### A.6 단계3 — 이미지 마스크 IPM 투영 + LiDAR 융합 검수 (선택; 더 정확한 라벨용)
`generate.py`로 만든 **데이터셋(sample) 위에서** 동작한다(`ipm_review.py`). 사람이 각 sample 의
`cam_{front,left,right}.jpg`에 그린 drivable 마스크를 **바닥 평면(z=−H)** 에 역투영(IPM)해 BEV로 올리고,
그 sample 의 `meta.json`(stamp·z_gate·calib)으로 **LiDAR 라벨을 재구성**해 겹쳐 본다.
- **왜 H(카메라 바닥 위 높이)가 필요**: 마스트 LiDAR는 바닥을 못 봐 H를 못 준다 → **자로 실측**(예: 렌즈 0.87m). `--cam-height`.
- **마스크 레이아웃**(데이터셋 sample 구조 미러링): `<mask-dir>/sample_NNNNNN/cam_{front,left,right}.png`(흰=drivable).
  마스크 있는 sample 만 처리(부분 라벨 OK). calib·orient·z_gate 는 sample `meta.json` 에서 자동으로 읽음(플래그로 덮어쓰기 가능).
- **annotation tool 연동(파일명 충돌 해결)**: sample마다 `cam_front/left/right` 이름이 겹쳐 한 폴더에 못 모은다.
  `dataset_flatten.py`로 펼치고(export) 되돌린다(gather):
  ```bash
  # 1) 펼치기(업로드용): <flat>/sample_NNNNNN__cam_{name}.jpg (이름 유일)
  python3 dataset_flatten.py export --dataset-dir ../../data/bev/dataset/temp_raws1 --out <flat_imgs>
  # 2) annotation tool에서 마스크 작업 → export
  # 3) 되돌리기: 마스크 → annotations/<bag>/sample_NNNNNN/cam_{name}.png
  python3 dataset_flatten.py gather --flat-masks <flat_masks> --out ../../data/bev/annotations/temp_raws1
  ```
  gather는 파일명에 `sample_NNNNNN__cam_<name>` 만 있으면 툴이 접미사를 붙여도 인식, 마스크는 >0 을 drivable로 이진화.
- **CVAT 연동**: CVAT는 마스크를 0/1 이진이 아니라 "Segmentation mask 1.1"(클래스별 RGB 컬러 + `labelmap.txt`)로 내보낸다.
  색은 프로젝트 설정마다 달라지므로 `gather`(>0 이진화) 대신 `gather-cvat` 로 labelmap의 클래스 색을 정확히 매칭한다:
  ```bash
  # CVAT 내보내기: "Segmentation mask 1.1" (labelmap.txt + SegmentationClass/ 포함)
  python3 dataset_flatten.py gather-cvat \
    --cvat-dir ../../data/bev/annotations/temp_raws1/cvat_label \
    --out      ../../data/bev/annotations/temp_raws1        # --class 기본 drivable
  ```
  `SegmentationClass/*.png` 에서 labelmap의 `drivable` 색과 일치하는 픽셀만 255로 만들어 `sample_NNNNNN/cam_{name}.png` 생성(다른 클래스 오염 없음).
```bash
cd calibration/bev_autolabel
python3 ipm_review.py \
  --dataset-dir ../../data/bev/dataset/raws1 \
  --map-dir     ../../data/sj_bags/260722/maps/raws1_mapping \
  --mask-dir    ../../data/bev/annotations/raws1 \
  --cam-height  0.87        # --near 2.0(하늘 후보 승격 반경), --out(미지정 시 각 sample 폴더에 기록)
```
→ 각 `sample_NNNNNN/` 안에 추가: `review_combined.png`(상단 3이미지+하단 LiDAR/IPM/융합 3-패널; 격자·범례 포함),
`label_fused.png`(**장식 없는 80×80 다색 카테고리 맵** — 겹침/불일치를 색으로 구분, **label tool에 바로 로드**; 색 의미는 review_combined 범례),
`meta_review.json`(카테고리 셀 수). (LiDAR-only 0/1/2 는 generate 산출 `label.png` 그대로 사용.)
- **검수뷰 색**: 밝은초록=둘 다 drivable / 빨강=LiDAR 장애물 / **주황=이미지바닥∩LiDAR장애물(→obstacle 채택, "잎 밑 바닥" 함정)** / 하늘=이미지 후보바닥(근거리 승격·원거리 ignore) / 어두운초록=LiDAR만 drivable / 회색=미확정.
- **한계**: IPM은 평면 가정이라 원거리·측면 부정확·수직물체 번짐, LiDAR는 국소 drift 가능 — **둘 다 100% 아님 → 사람이 최종 판단**. 일치 셀은 자동 확정, 불일치만 검수.

---

## 1. 태스크 정의

- **모델 입력**: 어안 카메라 **3대(front/left/right)** 이미지. **rear는 제외**(이유 §7-A).
- **모델 출력/정답**: ego 중심 top-down **BEV occupancy**(주행가능/불가/무시).
- **환경**: 온실(작물이 수직으로 길게 늘어진 이랑 구조), 저속 로봇.
- **전략**: 우선 **단일 환경 파이프라인 PoC** → 성능 좋은 일반화 모델은 데이터 더 모아 추후.
  (BEVFormer류는 데이터 대량 필요. 지금은 1환경이라 과적합 전제, train/val 분할로 파이프라인 검증 목적.)

---

## 2. 입력 데이터 (구체 경로는 PoC 기준 예시)

| 항목 | PoC 경로 | 내용 |
|---|---|---|
| bag | `data/sj_bags/260722/bags/record-all_with-sun_3` | 카메라4(`/dev/video0~3`)+LiDAR(`/unilidar/cloud`) 동기 녹화 |
| 추출 이미지 | `data/extracted/raws3/` | `bag_extract`로 뽑은 `frame_NNNNNN/cam{0..3}.jpg` + `sets.csv`(타임스탬프) |
| LIO 맵 | `data/sj_bags/260722/maps/raws3_mapping/` | `map.pcd`(월드 밀집 클라우드, ~263만점), `trajectory.tum`(pose 46.5만, 궤적 162m) |
| 캘리브 | `data/calib_260723/calib.yaml` | DS intrinsic 4대 + 카메라간 extrinsic(`T_cam_front`) + **`T_front_lidar`** |
| 카메라 방향 | `data/calib_260723/orientation.json` | 카메라 idx↔front/right/rear/left 매핑 |

- **폴더 규칙(bag별 3쌍)**: `sj_bags/260722/bags/<원본bag>` ↔ `sj_bags/260722/maps/<name>_mapping`(LIO 산출)
  ↔ `data/extracted/<name>`(추출 이미지). `<name>`: `raws{N}`=with-sun, `rawos{N}`=without-sun.
  라벨 1건 = `--map-dir maps/<name>_mapping` + `--extract-dir extracted/<name>` 쌍을 같은 `<name>`으로 맞춘다.
- **맵 건강성 확인 필수**: `trajectory.tum` 총 길이를 실제 온실과 대조(궤적 붕괴 시 라벨 오염).
  PoC 맵은 길이 162.4m·범위 ~12m×35m·z변동 0.58m(평평) = 건강. (참고: `docs/MAPPING.md`, 온실 판정 기준)

---

## 3. 좌표계·규약

- **ego 프레임 = LIO body(=LiDAR) 프레임**: `x=전방, y=좌, z=상`. `trajectory.tum`의 pose = `world_T_body`.
- **BEV 표시**: 전방(+x)=이미지 위, 좌(+y)=이미지 왼쪽.
- **카메라**: Double Sphere(DS) 어안 모델. `T_front_lidar`=LiDAR→front, `T_cam_front[name]`=front→각 카메라.
  투영 체인: `pixel = cam.project(T_cam_front · T_front_lidar · P_lidar)` (`calibration/cam_lidar/chain.py:project`).
- **시간 동기**: 카메라 stamp(`sets.csv`의 `stamp0`)와 `trajectory.tum` 시각은 **동일 절대시계**(둘 다 Unix epoch).
  → `pose_at(times, poses, cam_stamp_ns)` 로 프레임 시각의 pose를 바로 보간(수동 오프셋 불필요).
  (참고: PoC bag에서 LiDAR가 카메라보다 ~4.7s 먼저 시작하지만, 절대시각 보간이라 문제없음.)

---

## 4. BEV 규격 (해상도·범위) — **중요**

전진 주행 로봇에 맞춘 **전방 편향 비대칭** 그리드:

| 파라미터 | 값 | 의미 |
|---|---|---|
| 전방 `XF` | **3.0 m** | ego 앞으로 보는 거리 |
| 후방 `XR` | **1.0 m** | ego 뒤로 보는 거리(짧게) |
| 좌우 `YH` | **±2.0 m** | 좌/우 각 2m |
| 해상도 `RES` | **0.05 m/cell** | |
| 그리드 | **80 × 80** (`NX×NY`) | x범위 4m, y범위 4m |
| ego 셀 | row=`60`, col=`40` | x=0(전방3m 위, 후방1m 아래), y=0(좌우 중앙) → **ego는 아래쪽 1/4 지점** |

- 저속 로봇이라 주변 3~4m면 충분. 중앙배치(8×8m) 대비 **ignore 낭비↓·전방 감독밀도↑**.
- 라벨 셀 값: `0=obstacle, 1=drivable, 2=ignore`.

---

## 5. 카메라 선택 (입력)

- **front / left / right 3대만** 모델 입력으로 사용. **rear 제외**.
- 라벨은 **LiDAR+맵 기반**이라 카메라 선택과 **무관**(360° 라벨). 카메라 선택은 모델 입력에만 영향.
- rear 제외 이유는 §7-A(운영자 상주) 참조. 향후 운영자 없는 자율수집 데이터에선 rear 복귀 가능.

---

## 6. Auto-Label 생성 파이프라인 (단계별 상세)

> ⚠️ **아래는 PoC 서술이다.** 최종 구현은 **§A.1 변경사항 표**를 따른다(수직성 obstacle·국소 floor·카메라 FoV observed·
> self 반경 0.28·corridor 무조건 drivable·ego 연결정리). 수치가 다르면 §A·스펙이 최신.

### Step 0. 로드
- `map.pcd`(월드 클라우드), `trajectory.tum`(`load_tum`→ `times_ns`, `poses`),
  `calib.yaml`+`orientation.json`(`load_rig`→ `rig`; `load_T_front_lidar`→ `T_front_lidar`).

### Step 1. self/rig 마스크 (맵당 1회) — 카트/테이블/마스트 제거
카메라 리그를 실은 **카트·테이블·센서 마스트·운영자 손**은 센서에 붙어 다녀서 **body 좌표에서 늘 같은 자리**에 있다.
- 궤적에서 **~150개 pose 균등 샘플**.
- 각 pose에서 **맵의 근처 점(수평 반경 1.7m)을 body 프레임으로 변환**.
- **수평 반경 < 0.65m & |z| < 1.5m** 인 점만 **0.15m 복셀**로 양자화, pose별 1회 카운트.
- **> 60% pose에서 점유된 복셀 = self(rig)** 로 판정 → `SELF` 집합.
- ⚠️ **반경·지속성 조건이 핵심**: 온실 통로 폭이 일정해 작물벽이 body ±0.7m에 지속적으로 나타남 →
  반경/지속성 제한 없이 하면 **작물벽까지 self로 오인해 삭제됨**(실증된 함정). 반경<0.65m로 카트만 잡는다.

### Step 2. 키프레임 선택 (최종 규칙은 구현 시 확정)
- 후보: **이동거리 0.3~0.5m 간격** 또는 **0.5~1s 간격**. (라벨이 글로벌 맵 기반이라 프레임별 누적 window 튜닝 불필요.)

각 키프레임(카메라 stamp `t_ns`)에 대해 Step 3~9 수행:

### Step 3. pose
- `T_wb = pose_at(times, poses, t_ns)` (world_T_body), `T_bw = inv(T_wb)`.

### Step 4. 지역 크롭 + ego 변환
- ego 위치 근처 **월드 수평 6m 이내** 맵 점을 골라 `P = T_bw · p` 로 ego(body) 프레임 변환.

### Step 5. self 제거
- Step 1의 `SELF` 복셀에 속한 점 제거(카트/테이블/마스트 제거).

### Step 6. BEV 범위 크롭
- `x∈[-1, 3], y∈[-2, 2]` 인 점만 유지.

### Step 7. 바닥 높이 추정
- `floor = (크롭 점 z의 2 퍼센타일)`. (LiDAR가 바닥을 성글게 봐도 최저부 근사.)

### Step 8. 장애물 footprint (지면 투영) — **낮은 밴드만**
- **장애물 밴드 = z ∈ [floor+0.1, floor+1.0]** (로봇 높이 이하). 각 점의 (x,y)를 셀로 → 카운트.
- **셀당 ≥2점 → 장애물 후보**, `morphologyEx(OPEN)` 후 `(CLOSE)` (3×3)로 스펙클 제거·틈 메움 → `obstacle`.
- ⚠️ **낮은 밴드만 쓰는 이유**: 위쪽(천장·통로 위 캐노피/파이프)을 지면에 투영하면 **주행가능한 바닥이 obstacle로 오인**됨.
  수직 작물행은 낮은 부분도 같은 (x,y) 셀에 투영되므로 footprint는 유지된다.

### Step 9. 전방 궤적 corridor (drivable 덮어쓰기)
- 시각 `t` **±20s** 의 궤적 pose를 ego로 변환.
- **전방(x ≥ -0.2)** 궤적점마다 **반경 0.45m 원**을 drivable로 마킹 + ego 자기 자리 **반경 0.3m 원**.
- ⚠️ **전방만** 하는 이유: 운영자가 궤적을 따라 맵에 박제됨. 전방 corridor로 **운영자 자국을 지워** 전방 통로를
  drivable로 만든다. 후방은 덮어쓰지 않아 **운영자/장애물이 그대로 남게** 한다.

### Step 10. 가림 처리 (visible-only) — ego ray-cast
- `obs_rc = obstacle & ~corridor`.
- ego 셀(60,40)에서 **0.5° 간격 광선**을 쏴, 각 광선을 **첫 장애물 셀까지만** `visible=True`, 그 뒤는 미방문(=가려짐).
- ⚠️ **ray-cast 단독** 사용(과거 `raycast AND cam_fov` 는 중간이 FoV밖일 때 **끊긴 초록 "섬"** 을 만들어 폐기).
  광선은 ego와 항상 연결되므로 섬이 안 생긴다.

### Step 11. 라벨 조립
- 기본값 `2(ignore)`.
- `visible & ~obs_rc → 1(drivable)`, `visible & obs_rc → 0(obstacle)`, `corridor → 1(drivable)`.

### Step 12. (마무리 정리 — 구현 시 추가 권장)
- **ego와 연결된 drivable 성분만 남기기**(작은 pocket/스펙클 제거). PoC 스크립트엔 아직 미포함.

### Step 13. 샘플 저장
- 그 시각의 **front/left/right 이미지**(`sets.csv`로 stamp 매칭) + **BEV 라벨** + **ignore 마스크** + calib + ego-pose 를 한 샘플로 저장.
- 구체 폴더/네이밍 규칙은 **구현 시 별도 확정**(미정).

---

## 7. 특이사항·주의 (반드시 숙지) — **가장 중요한 섹션**

### A. 운영자/카트 오염 (수집 방식 artifact)
- PoC 데이터는 **사람이 카트를 밀고** 촬영 → rear 이미지에 사람 상주, 맵에 사람·카트가 궤적따라 박제됨.
- 처리: ① **rear 카메라 입력 제외**(사람이 지배하는 입력 제거) ② **self-mask로 카트 근접부 제거**(§Step1)
  ③ **전방만 corridor**(§Step9) ④ **전방편향(후방1m)** 이라 1m+ 뒤의 운영자는 라벨 영역 밖.
- **자율주행 수집 데이터엔 운영자가 없으므로 이 오염 자체가 사라진다.** 근본 해결 = 운영자 없는 수집.

### B. **LiDAR 바닥 사각 (가장 중요한 하드웨어 특성)**
- Unitree L2는 회전형이라 **아래쪽 블라인드 콘**이 있어 **가까운 바닥에 점이 거의 안 찍힘**.
  이미지에 LiDAR를 overlay하면 **점이 상단(천장·먼 작물)에만** 있고 **근접 바닥엔 점 0개**(overlay_diag로 확증).
- 그래서 drivable을 **"바닥을 봐서"가 아니라 "장애물이 없어서"** 로 정의한다. 장애물(수직 구조)은 LiDAR가 잘 본다.

### C. 천장/오버행 오투영
- 높은 점(천장·통로 위 파이프/캐노피)을 지면에 투영하면 주행가능 바닥이 obstacle로 오인 → **낮은 밴드([floor+0.1,1.0])만** 사용.

### D. 가림(occlusion)은 visible-only
- 작물행 뒤 등 **지금 못 보는 영역은 ignore(회색)**. 맵은 옆 통로에서 그 뒤를 봤어도, 현재 시점 카메라는 못 보므로
  라벨하지 않는다(모델에 근거 없는 정답을 주지 않음). ray-cast로 구현.

### E. 전방 동적 물체 (미해결·향후)
- 정적 rear 처리는 **앞에서 걸어오는 사람** 같은 전방 동적 물체를 못 막는다. PoC 데이터엔 없어 보이나,
  일반화 시 **동적 물체 검출/제거**가 필요.

### F. 맵 품질 의존
- 라벨 정확도는 **LIO 맵 정확도에 종속**. 매핑 전 궤적 길이·형태를 실제와 대조(§2, `docs/MAPPING.md`).

### G. 데이터 규모
- 단일 환경·상관 높은 프레임 → 일반화 모델은 **여러 bag/환경 확충 전제**. PoC는 파이프라인·형태 검증용.

### H. 어안→BEV 모델 입력 준비
- BEVFormer류는 핀홀 가정 → 어안(DS)은 **가상 핀홀 언디스토션**(주변부 FoV 손실 감수) 또는 **DS-aware 샘플링** 한 단계 필요.

### I. 환경 핀 (numpy/scipy)
- 이 Jetson은 **시스템 scipy가 numpy<1.25 고정**. `pip --user`로 numpy≥2(opencv-python≥4.10 등) 끌어오면 scipy가 깨져
  파이프라인 전체 중단(`ValueError: numpy.dtype size changed`). user-site에 numpy≥2/opencv-python≥4.10 넣지 말 것.
  PCD 읽기는 **open3d 0.18(user-site, numpy1.x ABI 호환)** 사용.

---

## 8. 파라미터 요약표

| 이름 | 값 | 위치/의미 |
|---|---|---|
| `XF, XR, YH` | 3.0, 1.0, 2.0 m | BEV 전/후/좌우 범위 |
| `RES` | 0.05 m | 셀 크기 (그리드 80×80) |
| ego 셀 | (60, 40) | x=0,y=0 위치 |
| self 반경/지속성 | **<0.28 m** / >60% pose | 카트 판정(40×40cm 반대각) |
| self 복셀 `VOX` | 0.15 m | self-mask 양자화 |
| self 샘플 pose 수 | 150 | 균등 샘플 |
| 지역 크롭 반경 | 6 m (수평) | 맵→ego 프리필터 |
| `floor` | **국소(~1m 윈도) z 2 퍼센타일** | 바닥 높이 격자 |
| 장애물 판정 | **수직성**: z_min ≤ floor+`z_gate`(0.3) AND extent ≥ 0.5 m | 천장·캐노피 배제 |
| 장애물 셀 임계 | ≥2 점(이 데이터선 무효) | + morph open/close(3×3) |
| corridor 궤적창 | ±20 s | 전방(x≥-0.2)만 |
| corridor 반경 | 0.45 m(궤적) / 0.3 m(ego) | **무조건 drivable**(관측 게이트 없음) |
| observed | **카메라 FoV ∩ ray-cast**(0.5°, 첫 장애물까지) | 보이는 곳만 |
| 클래스 | 0=obstacle,1=drivable,2=ignore | |
| 입력 카메라 | front/left/right | rear 제외 |

---

## 9. 검증 방법

- **LiDAR-on-image overlay**(`overlay_diag.py` 참조): 맵 점을 3대 이미지에 **높이색**(파랑=바닥, 빨강=장애물밴드,
  초록=위)으로 투영. 정합·바닥사각·밴드 적정성 육안 확인.
- **BEV 다양성 확인**: 통로 초입(개활지)·중앙(좁은 회랑)·끝(교차부 Y분기)에서 라벨이 장면과 맞는지.
  PoC 검증 프레임: 900(멀티통로), 2500(중앙), 4850(교차부), 2000(중앙+분기).
- 판정: 초록(drivable)이 통로에, 빨강(obstacle)이 작물행/장비에, 회색(ignore)이 가림/미관측에 오면 양호.

---

## 10. 구현 완료 (2026-07-27) & 향후

- CLI 구현 완료: **`calibration/bev_autolabel/`** (실행법 §A). 단계1 `verify_labels.py` + 단계2 `generate.py`.
- 출력 규칙 확정: 샘플당 `label.png`(class 인덱스 팔레트) + `review.png`(검수뷰) + `cam_{front,left,right}.jpg` +
  `meta.json`, 최상위 `dataset.csv`. 키프레임 = 이동거리 0.4m 간격.
- 의존 모듈(`calibration/cam_lidar`, `calibration/verify`)은 이 브랜치(`feat/bev-autolabel`)에 포함됨.
- **향후**: ①여러 bag/환경 확충(일반화) ②전방 동적물체(§7-E) 처리 ③어안→모델입력 언디스토션(§7-H)
  ④사람 검수·재라벨링 툴로 최종 데이터셋 확정.

---

## 부록: PoC 참조 구현 (footprint + ray-cast, 최종본)

> 세션 스크래치패드에서 검증한 스크립트. **경로는 하드코딩(PoC용)**, CLI화 시 인자로 파라미터화할 것.
> 시각화(3이미지+BEV 합성 PNG) 포함. 라벨 생성 핵심은 `bev()` 함수의 Step 4~12.

```python
#!/usr/bin/env python3
"""장애물 footprint + ray-cast 가림. LiDAR 낮은밴드 점을 지면투영=obstacle, 보이는 빈 곳=drivable, 가림=ignore."""
import sys, pathlib, csv
from collections import Counter
import numpy as np, open3d as o3d, cv2
from scipy.spatial import cKDTree

CAM_LIDAR = pathlib.Path("/home/cv_pretest/Desktop/econ_camera_ws/calibration/cam_lidar")
sys.path.insert(0, str(CAM_LIDAR.parent / "verify")); sys.path.insert(0, str(CAM_LIDAR))
from cloud_io import load_tum, pose_at
from ds_model import load_rig

MAPDIR = pathlib.Path("<LIO 맵 폴더>")          # map.pcd, trajectory.tum
EXTRACT = pathlib.Path("<추출 이미지 폴더>")     # frame_NNNNNN/cam{idx}.jpg, sets.csv
CALIB = "<calib.yaml>"; ORIENT = "<orientation.json>"
USE = ["front", "left", "right"]                # rear 제외
XF, XR, YH, RES = 3.0, 1.0, 2.0, 0.05
NX = int((XF + XR) / RES); NY = int(2 * YH / RES); R_EGO = int(XF / RES); C_EGO = int(YH / RES); SC = 400 // NX
VOX = 0.15

p = np.asarray(o3d.io.read_point_cloud(str(MAPDIR / "map.pcd")).points)
times, poses = load_tum(str(MAPDIR / "trajectory.tum"))
tpos = np.array([P[:3, 3] for P in poses])
rig = load_rig(CALIB, ORIENT); name2idx = {v: k for k, v in rig.idx_to_name.items()}
stamp = {}
with open(EXTRACT / "sets.csv") as f:
    for r in csv.DictReader(f):
        stamp[int(r["idx"])] = int(round(float(r["stamp0"]) * 1e9))

def _key3(C):
    return (C[:, 0] + 100) * 1_000_000 + (C[:, 1] + 100) * 1000 + (C[:, 2] + 100)

# --- self/rig 마스크(카트/테이블): body 좌표서 반경<0.65m·>60% pose 지속 ---
_tree = cKDTree(p[:, :2]); _cnt = Counter(); _samp = np.linspace(0, len(poses) - 1, 150).astype(int)
for _i in _samp:
    _Tbw = np.linalg.inv(poses[_i]); _idx = _tree.query_ball_point(poses[_i][:3, 3][:2], r=1.7)
    if not _idx:
        continue
    _Pb = (_Tbw[:3, :3] @ p[_idx].T).T + _Tbw[:3, 3]
    _b = (np.hypot(_Pb[:, 0], _Pb[:, 1]) < 0.65) & (np.abs(_Pb[:, 2]) < 1.5)
    for _k in set(_key3(np.floor(_Pb[_b, :3] / VOX).astype(int)).tolist()):
        _cnt[_k] += 1
SELF = np.array([k for k, c in _cnt.items() if c / len(_samp) > 0.6], np.int64)

def to_ego(Tbw, W):
    return (Tbw[:3, :3] @ W.T).T + Tbw[:3, 3]

def rc(x, y):
    return int((XF - x) / RES), int((YH - y) / RES)

def raycast(obs):
    vis = np.zeros((NX, NY), bool)
    for a in np.deg2rad(np.arange(0, 360, 0.5)):
        dr, dc = np.cos(a), np.sin(a)
        for rr in np.arange(0.0, NX + NY, 0.5):
            r = int(round(R_EGO + dr * rr)); c = int(round(C_EGO + dc * rr))
            if not (0 <= r < NX and 0 <= c < NY):
                break
            vis[r, c] = True
            if obs[r, c]:
                break
    return vis

def bev(t_ns):
    Tbw = np.linalg.inv(pose_at(times, poses, t_ns)); ctr = pose_at(times, poses, t_ns)[:3, 3]
    near = (np.abs(p[:, 0] - ctr[0]) < 6) & (np.abs(p[:, 1] - ctr[1]) < 6)
    P = to_ego(Tbw, p[near])
    P = P[~np.isin(_key3(np.floor(P[:, :3] / VOX).astype(int)), SELF)]      # self(카트) 제거
    m = (P[:, 0] <= XF) & (P[:, 0] >= -XR) & (np.abs(P[:, 1]) <= YH); P = P[m]
    floor = np.percentile(P[:, 2], 2)
    row = ((XF - P[:, 0]) / RES).astype(int).clip(0, NX - 1); col = ((YH - P[:, 1]) / RES).astype(int).clip(0, NY - 1)
    band = (P[:, 2] > floor + 0.1) & (P[:, 2] < floor + 1.0)                # 낮은 밴드만
    cnt = np.zeros((NX, NY), int); np.add.at(cnt, (row[band], col[band]), 1)
    k = np.ones((3, 3), np.uint8)
    obstacle = cv2.morphologyEx((cnt >= 2).astype(np.uint8), cv2.MORPH_OPEN, k)
    obstacle = cv2.morphologyEx(obstacle, cv2.MORPH_CLOSE, k).astype(bool)  # 지면투영 footprint
    # 전방(x>=-0.2) 궤적 corridor = drivable 덮어쓰기(운영자 자국 제거)
    tw = np.abs(times - t_ns) < int(20e9)
    TE = to_ego(Tbw, tpos[tw]); TE = TE[(TE[:, 0] <= XF) & (TE[:, 0] >= -XR) & (np.abs(TE[:, 1]) <= YH)]
    corridor = np.zeros((NX, NY), np.uint8)
    for x, y, _ in TE:
        if x < -0.2:
            continue
        cv2.circle(corridor, rc(x, y)[::-1], max(1, int(0.45 / RES)), 1, -1)
    cv2.circle(corridor, (C_EGO, R_EGO), int(0.3 / RES), 1, -1)
    corridor = corridor.astype(bool)
    obs_rc = obstacle & ~corridor
    visible = raycast(obs_rc)                                              # ego 가림(첫 장애물 앞까지)
    label = np.full((NX, NY), 2, np.uint8)                                 # 2=ignore
    label[visible & ~obs_rc] = 1                                           # drivable
    label[visible & obs_rc] = 0                                            # obstacle
    label[corridor] = 1
    # TODO(구현): ego 연결 성분만 남겨 잔여 pocket 제거
    return label   # (NX,NY) uint8: 0/1/2
```

**시각화**(3이미지+BEV 합성)는 `label`을 색칠(0→빨강, 1→초록, 2→회색)하고 `name2idx[nm]`로 이미지 매칭.
LiDAR-on-image 진단은 별도 `overlay_diag.py`(맵 점을 `chain.project`로 3대 이미지에 높이색 투영).
