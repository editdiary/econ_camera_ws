# BEV 자동 라벨링 파이프라인 (Camera→BEV Occupancy 데이터셋)

> **이 문서의 목적**: 3어안 이미지로 **BEV occupancy** 를 추론하는 모델의 **학습 정답
> (auto-label)** 을, 현장에서 수집한 ROS bag + 그 bag 의 LIO 맵에서 자동 생성하는 파이프라인을
> 기록한다. 새 세션에서 바로 이어받아 작업할 수 있도록 규격·단계·특이사항을 모두 적는다.
>
> **현행판은 아래 §B 슬래브 라벨이다** — 실행 가이드이자 규격이 §B 하나에 다 있다.
> 설계 근거·실측값은 `docs/superpowers/specs/2026-08-10-bev-slab-label-design.md`.
>
> **부록 A(구판, 2026-07)** 는 초기 경로다(`generate.py`, 단일 `label.png` 0/1/2, 80×80).
> LiDAR 라벨 로직은 §B 가 대체했으므로 **새 데이터를 만들 때 따라 하지 말 것**. 다만 IPM
> 투영(`ipm.py`)·좌표 규약(§3)·BEV 규격(§4)·특이사항(§7)은 §B 도 그대로 쓰고 있고,
> `data/bev/dataset/` 의 기존 80×80 산출물을 해석하려면 필요하므로 보관한다.

---

## 0. 한눈 요약

- **입력**: ① 카메라4+LiDAR 가 함께 녹화된 bag(`record_all.launch.py` 산출), ② 그 bag 을
  Point-LIO 로 매핑한 폴더(**`map_clean.pcd`** + `trajectory.tum`, `docs/MAPPING.md`),
  ③ 캘리브(`calib.yaml` + `orientation.json`, **`T_front_lidar` 포함**), ④ 카트 이미지 마스크.
- **출력(1 샘플)**: `{3장 이미지(front/left/right), occupancy.png, visibility.png, IPM RGB
  캔버스, CVAT 보정 base(overlay.png), 검수뷰(review.png), meta.json}`. 기본 **120×120**.
- **라벨 2채널**: `occupancy.png` = `0=obstacle` / `1=drivable`,
  `visibility.png` = `0=unseen` / `1=visible`. 학습에서 visibility 를 loss 마스크로 쓰면
  미관측 영역이 자동 배제되므로 occupancy 에 unknown 클래스를 두지 않는다.
  구판의 3-클래스 단일 `label.png`(0/1/2)를 대체한 것이다.
- **핵심 아이디어**: LiDAR 는 **바닥을 못 보고 장애물(수직 구조)만 잘 본다**. 그래서 로봇이
  통과해야 하는 높이 구간(**슬래브**)에 점이 서 있으면 obstacle, 그 외 보이는 곳은 drivable
  로 정의한다. **바닥 자체는 LiDAR 에 안 찍히므로**, 카메라를 지면 평면에 **IPM 투영**해
  BEV 배경(위에서 본 주행면 모습)을 만든다.
- **사람 검수**: auto-label 은 초안이다. 사람은 **카메라에 마스크를 그리지 않고**, IPM 배경
  위에 obstacle 만 얹힌 `overlay.png` 를 **BEV 위에서 보정만** 한다. 보정 대상은 **occupancy
  하나뿐** — visibility 는 보정본으로 raycast 를 다시 돌리면 재생성된다.

---

## §B 슬래브 라벨 — **현행 실행 가이드**

`map_clean.pcd` 에서 키프레임별 BEV occupancy + visibility 를 만든다. **LiDAR 라벨은 이
경로가 현행판**이고, 구판(부록 A §A)의 `generate.py` 단일 라벨(`label.png`, 0/1/2)을
대체한다. IPM RGB 투영(`ipm.py`)은 구판 것을 그대로 재사용한다.

구성 모듈: `slab_label.py`(순수 로직) · `slab_io.py`(8필드 보존 PCD·마스크·표본 IO) ·
`slab_render.py`(PNG 저장·검수뷰) · `generate_slab.py`(생성 CLI) · `slab_sheet.py`(검수 시트) ·
`gather_slab.py`(검수용 모음). 재사용: `bev_label.raycast_visible` · `ipm.py` ·
`calibration/cam_lidar/{chain,cloud_io,calib_io}.py` · `calibration/verify/ds_model.py`.

설계 근거·실측값: `docs/superpowers/specs/2026-08-10-bev-slab-label-design.md`

**BEV 범위는 구판과 다르다.** `generate_slab.py` 의 기본값은 `--xf 4.0 --xr 2.0 --yh 3.0` →
**120×120**(`RES`=0.05 고정, 구판과 동일). 구판 `generate.py` 의 기본은
`--xf 3.0 --xr 1.0 --yh 2.0` = **80×80** — 서로 다른 그리드다. 두 CLI 의 산출물을 한 학습
데이터셋에 섞지 말 것(진짜 값은 각 sample 의 `meta.json`→`bev`→`NX/NY` 로 확인).

### 실행

    cd calibration/bev_autolabel
    python3 generate_slab.py \
      --map-dir ../../data/sj_bags/260722/maps_selfmask/raws3_mapping \
      --extract-dir ../../data/extracted/raws3 \
      --calib ../../data/calib_260723/calib.yaml \
      --orient ../../data/calib_260723/orientation.json \
      --self-mask-dir ../../data/calib_260723/self_mask \
      --out ../../data/bev/slab/raws3

#### 키프레임 간격 — 구간별로 다르게 줄 수 있다

`--kf-step` 은 단일값(`0.4`, 기본) 외에 **구간별 스케줄**을 받는다:

    --kf-step 0:0.5,0.2:1.5,0.7:0.5     # 0~20% 0.5m / 20~70% 1.5m / 70~100% 0.5m

`시작비율:step[m]` 쌍을 쉼표로 잇는다. **시작비율은 누적 이동거리 기준**이다 — 프레임
순번이면 정지·서행 구간에서 경계가 공간상 앞으로 밀린다. 첫 구간은 0 에서 시작해야 하고
비율은 오름차순이어야 한다(아니면 즉시 중단 — 앞 구간이 통째로 빠지는 건 대개 오타다).

구현은 구간별로 프레임을 잘라 `select_keyframes` 를 따로 돌리고 이어붙이는 것이라
`sample_NNNNNN` 번호가 전 구간에 걸쳐 연속으로 매겨진다(두 번 돌려 수동 병합할 필요 없음).
**경계에서는 직전 키프레임 기억이 끊기므로 step 보다 가까운 쌍이 경계마다 하나 생길 수
있다.** 실행 로그에 구간별 `프레임 N → 키프레임 M` 이 찍히므로 의도한 분배인지 확인한다.
`meta.json`→`params`→`kf_step` 에는 입력 문자열이 그대로 기록된다(단일값도 문자열).

IPM 관련 옵션: `--cam-height 0.87`(IPM 지면 평면용 카메라 렌즈 높이[m] 실측값, IPM 정확도의
핵심) · `--blend nearest`(기본, 셀별 최근접 1대)|`average` · `--alpha 0.30`(obstacle
오버레이 불투명도, IPM 이 잘 보이도록 낮게 잡음) · `--no-ipm`(IPM 생성을 꺼서 LiDAR
라벨만 빠르게 뽑을 때).

### 산출물

    sample_NNNNNN/{slab.pcd, crop.pcd(--save-crop), occupancy.png, visibility.png,
                   ipm_rgb.png, overlay.png, review.png,
                   cam_{front,left,right}.jpg, meta.json} + dataset.csv

`ipm_rgb.png` — 3어안을 지면 평면(z0=C_z−cam_height)에 IPM 투영한 BEV RGB 캔버스(NX×NY,
빈 셀=0). LiDAR 는 바닥을 못 봐 occupancy/visibility 만으로는 바닥이 실제로 통로인지
사람이 판단할 수 없다 — 바닥 모습은 이 캔버스로만 보인다.

`overlay.png` — `sr.blend_slab(ipm_rgb, occupancy, alpha)`. **네이티브 해상도(NX×NY)·
장식 없음** = CVAT annotation base. **obstacle(occ=0) 셀만** 빨강을 `alpha`(기본 0.30)로
얹고 drivable 은 IPM 원본 그대로 둔다 — 잉크는 보정이 필요한 곳에만, 바닥 판단 근거인
IPM 은 최대한 살린다.

**보정 대상은 occupancy 하나뿐이다.** visibility 는 보정된 occupancy 로
`bev_label.raycast_visible` 을 다시 돌리면 재생성되므로 annotation base 에 실을 이유가
없다(오히려 사람이 고칠 대상을 헷갈리게 만든다). visibility 는 `review.png` 왼쪽 4색
패널과 `visibility.png` 로 계속 확인할 수 있다.

> 이전 판은 `visibility=1` 셀만 칠했다. raycast 가 장애물 **앞면에서 멈추므로** 보이는
> 셀은 거의 전부 drivable 이고, raws3 98샘플 실측에서 빨강이 칠해진 셀은 obstacle 전체의
> **4%**(전체 캔버스의 0.9%)뿐이었다 — 사실상 obstacle 이 안 보이는 base 였다.

검수 산출물은 `slab_sheet.py` 로 따로 만든다(생성 CLI 가 자동 실행하지 않는다):

    cd calibration/bev_autolabel && python3 slab_sheet.py ../../data/bev/slab/raws3

→ 같은 폴더에 `_sheet_review.png`(궤적 전체 15장 격자)·`_sheet_stack.png`(occupancy/
visibility/review 나란히)·`_stats.txt`(샘플별 z_ref·obstacle·visible·cam_ok·reach).

`occupancy.png` = 0 obstacle / 1 drivable. `visibility.png` = 0 unseen / 1 visible.
학습에서 visibility 를 loss 마스크로 쓰면 미관측 영역이 자동 배제된다. occupancy 에
unknown 클래스를 두지 않는 이유가 이것이다.

`review.png` 는 **상단에 원본 3어안(left/front/right) 스트립** + 하단에 BEV 두 패널을
나란히(`4color(occ+vis)` | `ipm+occupancy`, 각 패널 제목 표시). 왼쪽 4색: 초록=보이는
drivable(신뢰 영역), 빨강=보이는 장애물 표면, 갈색=가려진 장애물, 검정=미관측. 오른쪽은
`overlay.png` 를 확대 격자·미터축·ego 마커를 얹어 검수하기 쉽게 만든 것(`--no-ipm` 이면
오른쪽 패널 없이 왼쪽 4색 단독). 원본·4색 라벨·IPM 바닥을 한 장에서 대조할 수 있어야
라벨이 진짜 맞는지 사람이 판단할 수 있다.

ego 주변 검은 직사각형은 **정상**이다 — 카메라가 수평 바깥을 봐서 생기는 근거리 사각
(실측 가시 시작 0.65~0.80m)과 후방 self 박스가 합쳐진 것이다.

### 사람 검수·보정 — `gather_slab.py` 로 모은다 (`gather_annotations.py` 아님)

    cd calibration/bev_autolabel
    python3 gather_slab.py \
      --dataset ../../data/bev/slab/raws1 \
      --out ../../data/bev/annotations \
      --guided-labels

→ `data/bev/annotations/raws1/{label,review,label_guided}/sample_NNNNNN.png`. `label/`은 각 sample 의
`overlay.png`를 **바이트 그대로 복사**한 것이고(overlay.png 자체가 이미 네이티브 120×120·
장식 없는 annotation base 라 합성할 게 없다. 재인코딩하면 CVAT 마스크가 곧 정답인 base 의
화소가 바뀐다), `review/`는 `review.png` 복사다. `label_guided/`는 같은 해상도에 obstacle
alpha 를 높이고 0.5m grid 를 얹은 참고/대체 annotation base 다. 옵션: `--name`(하위 폴더 이름) ·
`--review-scale`(기본 1=원본 그대로, 0=review 생략) · `--guided-alpha`(기본 0.55).

구판(부록 A)의 `gather_annotations.py`는 슬래브 산출물에 **동작하지 않는다**(설계상 대상 밖이지
버그가 아니다). 그 스크립트는 sample 마다 `label.png`(0/1/2 인덱스 팔레트)를 요구하는데
(`gather_annotations.py:78`) 슬래브 산출물엔 `label.png`가 없다 — `occupancy.png`+
`visibility.png` 두 채널로 나뉘어 있다. 그래서 전 sample이 `skip (missing ipm_rgb/label)`로
건너뛰어지고 `done: 0 images`로 끝난다. 두 포맷은 그리드도 다르므로(80×80 vs 120×120)
한 스크립트로 합치지 않고 따로 둔다.

**`occupancy.png`를 `render.blend_label`에 넣지 말 것.** `blend_label`은 `mm = (label==0)|
(label==1)` 로 0/1 값을 전부 칠한다 — `occupancy.png`는 0/1 두 값뿐이므로 **캔버스 전 셀을**
칠해 IPM 바닥을 덮어버린다. 사람이 보정 근거로 삼을 게 사라진다. 슬래브 데이터셋에는
obstacle 만 얹고 바닥은 남기는 `overlay.png`가 이미 있으므로(§ 산출물, `sr.blend_slab`)
이걸 써야 한다.

궤적 전체를 한 장으로 훑는 검수는 `slab_sheet.py`의 `_sheet_review.png`가 따로 있다.

### 수동 보정본 → 학습용 occupancy/visibility 라벨 생성

CVAT 등에서 `data/bev/annotations/<dataset>/label_guided` 또는 `label` 이미지를 기준으로
occupancy 를 보정한 뒤, export 폴더를 아래 형식으로 둔다.

    data/bev/manual_annotated/<dataset>_120x120_annotation/
      labelmap.txt
      SegmentationClass/sample_NNNNNN.png

`labelmap.txt` 에는 `occupancy` 클래스 색이 있어야 한다. 현재 수동 export 예:

    occupancy:61,61,245::

학습에 쓸 2채널 라벨은 `manual_labels.py` 로 만든다. 폴더명에서 `<dataset>` 을 자동 추론하고,
원본 sample/meta/IPM 은 `data/bev/dataset/<dataset>` 에서 찾는다.

    cd calibration/bev_autolabel
    python3 manual_labels.py \
      --manual-dir ../../data/bev/manual_annotated/raws1_120x120_annotation

    # 원본 sample 이 ../../data/bev/slab/raws1 에 있을 때:
    python3 manual_labels.py \
      --manual-dir ../../data/bev/manual_annotated/raws1_120x120_annotation \
      --dataset-root ../../data/bev/slab

출력:

    data/bev/manual_labels/<dataset>/
      occupancy_npy/*.npy      # uint8 (NX,NY), 0=obstacle, 1=drivable
      visibility_npy/*.npy     # uint8 (NX,NY), 0=unseen, 1=visible
      occupancy_png/*.png      # 같은 class id 를 보존한 indexed PNG
      visibility_png/*.png     # 같은 class id 를 보존한 indexed PNG
      review_png/*.png         # 시각 검수용 4색 occ/vis + IPM overlay
      labels.csv
      README.md

visibility 는 수동 occupancy 에서 `bev_label.raycast_visible` 을 다시 돌려 재생성한다
(기본 `--ray-step 0.25`). 즉 map 기반 occupancy 를 다시 신뢰하지 않는다. 원본 sample 이
기본값(`data/bev/dataset/<dataset>`) 밖에 있으면 `--dataset-root` 에 sample 폴더들의 상위
경로를 넘긴다. 출력 경로나 이름을 바꾸려면 `--out`, `--name` 을 쓴다.

### IPM-only 비디오 진단

라벨 생성 전후로 카메라 3장의 IPM 투영이 시간축 전체에서 잘 맞는지 빠르게 훑고 싶으면
`tools/ipm_video_from_extract.py` 를 쓴다. `bag_extract.py` 산출(`frame_*/camN.jpg`, `sets.csv`)만
소비하고 라벨은 만들지 않는다.

    python3 tools/ipm_video_from_extract.py \
      --extract-dir data/extracted/raws1 \
      --calib data/calib_260723/calib.yaml \
      --orient data/calib_260723/orientation.json \
      --out data/bev/review/raws1_ipm.mp4 \
      --xf 4.0 --xr 2.0 --yh 3.0 \
      --cam-height 0.87 \
      --blend nearest

기본은 BEV 위에 원본 left/front/right 스트립을 붙인다. 서버에서 빠르게 만들 때는
`--pixel-step 2` 이상으로 원본 픽셀 샘플링을 줄일 수 있고, BEV만 보고 싶으면
`--camera-strip none` 을 쓴다.

### 파이프라인

1. pose 로 body 프레임 3D crop (z 무제한) — rawos 의 world z 드리프트(−1.7m)가 여기서 상쇄된다
2. `z_ref` = crop z 하위 1% ≈ LiDAR 수평면 ≈ 실제 지상 0.87m. 슬래브 `[z_ref, z_ref+0.8]`
   = 로봇이 통과해야 하는 높이 구간. 로봇보다 높은 장애물(열린 문·천장·배관)은 자동 배제
3. occupancy: 슬래브를 2D 기둥으로 눌러 셀당 점 ≥ 3 이면 obstacle
4. visibility: ego 셀 2D 360° raycast(첫 obstacle 에서 정지) ∧ 카메라 관측가능성 ∧ ¬self 박스

### 카트 자기 가림 — 이미지 마스크 + self 박스

카메라가 수평 바깥을 보게 장착돼 아래를 못 내려다본다. **실제 지면(z=−0.87)에서 반경 0.5m
완전 사각, 1.0m 까지 부분 사각**이다(기하만으로 가려지는 셀 4.0%). 여기에 카트 자기 몸이
더해지는데, 두 종류를 **다른 방법으로** 처리한다.

**상판·LiDAR 받침판 → 이미지 마스크.** 카메라에 고정돼 위치가 변하지 않으므로 이미지 공간에
칠하는 게 정확하다. `data/calib_260723/self_mask/mask_{front,right,left}.png` + `labelmap.txt`,
1280×720 클래스 색 PNG(`table 250,50,83`). 기본으로 `table` 만 읽는다(`--self-mask-classes`).
어안 유효원 바깥 검은 영역(이미지의 12~18%)은 프레임 표본의 밝기 퍼센타일로 자동 검출한다.

**손잡이·수집자 → body 프레임 self 박스** (`--self-box-near/far/yh`, 기본 0.4/2.1/0.7 = 896셀,
6.2%). 마스크 파일에 `handle`·`human` 도 칠해져 있지만 쓰지 않는다 — 수집자가 화면을 확인하려
몸을 기울이고, 회전 구간에서 위치가 바뀌고, 턱에 걸려 흔들려서 **이미지에서의 위치가 프레임마다
달라진다**. 정적 이미지 마스크는 없는 자리를 가리고(데이터 손실) 있는 자리를 놓친다(틀린 라벨).
물리적 위치는 body 프레임에서 늘 같으므로 박스가 맞다. 기본 박스는 handle·human 이미지 마스크가
죽이던 셀 143개를 100% 포함하고, `map_clean.pcd` 에 남은 수집자 잔재 위치(Point-LIO self mask
박스 x −1.5~−0.45·|y|<0.35, 허위 obstacle 12~185셀)도 완전히 담는다.

이미지 마스크가 없어도 돌아가지만 근거리 visibility 가 낙관적이라는 경고가 찍히고
`meta.json` 의 `self_mask` 가 `null` 이 된다. self 박스는 마스크와 무관하게 항상 적용된다.

### 판정 기준

| 항목 | 기준 |
|---|---|
| `z_ref` | bag·위치와 무관하게 +0.0~+0.1 |
| obstacle 셀 | 20~30% |
| 궤적셀이 obstacle 인 비율 | < 2% (라벨이 실제 주행과 모순되지 않는지) |
| 중앙축 `reach_far` | 중앙값 XF 도달, 4.0m 도달 80% 이상 (`slab_sheet.py` 가 계산) |
| `cam_ok` | 93~95%. 100% 에 가까우면 투영 평면이 틀렸다는 신호 |
| `visible_pct`(sample 전체) | 정상(중앙축 완전 가시) 샘플대엔 18~25%가 뜬다. 하지만 694개 전체 중앙값은 **5.0%**(§ 알려진 한계 — visibility 수율 참고) — 개별 sample 이 이보다 낮다고 곧 불량은 아니다(가려짐을 정직하게 반영한 것). 수율을 가늠할 때는 이 중앙값·`vis<5%` 비율을 같이 봐야 한다 |

### 하지 말 것

- **3D raycast**: ray 원점(body z=0)이 슬래브 밑면에 붙어 있어 수평 ray 가 장애물 아래로
  빠져나간다. '열에 닿은 voxel 하나라도' 기준이면 visibility 가 거의 전역 1 이 된다.
- **`--min-pts` 를 10 이상으로**: 실구조물까지 지운다(raws3 obstacle 23.8%→14.9%).
- **`--pct` 를 5 로**: 슬래브 바닥이 최대 0.25m 들려 실제 하위 점을 잘라먹는다.
- **corridor prior 부활·self 점 추가 제거**: 측정으로 불필요함이 확인됐고, 넓게 **지우면**
  좌우 0.33~0.58m 의 실제 통로 벽을 갉아먹는다. self 박스는 점을 지우지 않고 visibility 만
  0 으로 두므로 이 금지에 걸리지 않는다.
- **`ground_offset` 를 0 으로**: body z=0 은 수평선 평면이라 FoV 가 100% 로 나오고
  사각지대가 전부 사라진다.
- **`handle`·`human` 을 `--self-mask-classes` 에 넣기**: 이미지에서의 위치가 프레임마다
  달라 정적 마스크로는 못 맞힌다. self 박스가 그 역할이다.

### 7개 bag 검증 (2026-08-10)

`data/sj_bags/260722/maps_selfmask/` 7종(raws1-3, rawos1-4) 전부 `--limit` 없이 완주
(실패 샘플 0, `skip (missing image)` 없음). `<name>` 은 bag 이름과 동일, 산출물은
`data/bev/slab/<name>/`.

| bag | n | z_ref 중앙 | obstacle 중앙 | cam_ok 범위 | vis_start 범위* | reach_far 중앙 | 4.0m 도달율 | vis% 중앙† | vis<5% 건수† |
|---|---|---|---|---|---|---|---|---|---|
| raws1 | 98 | +0.045 | 26.9% | 93.5~94.8% | 0.70~0.75m | 2.58m | 21% | 1.5% | 89 |
| raws2 | 101 | +0.049 | 26.8% | 93.5~95.2% | 0.65~0.75m | 4.00m | 62% | 3.7% | 81 |
| raws3 | 98 | +0.048 | 22.2% | 92.8~95.3% | 0.65~0.80m | 4.00m | 85% | 18.7% | 1 |
| rawos1 | 99 | +0.045 | 28.0% | 92.9~95.0% | 0.70~0.80m | 4.00m | 77% | 3.9% | 80 |
| rawos2 | 104 | +0.041 | 28.1% | 92.7~94.8% | 0.70~0.80m | 2.08m | 14% | 1.1% | 91 |
| rawos3 | 96 | +0.045 | 22.5% | 92.9~95.3% | 0.65~0.80m | 4.00m | 82% | 17.6% | 2 |
| rawos4 | 98 | +0.052 | 22.3% | 92.5~94.8% | 0.70~0.80m | 4.00m | 87% | 17.9% | 4 |
| **합계** | **694** | | | | | | | **5.0%** | **348(50%)** |

*중앙축이 완전히 가려져 `reach_far`·`vis_start` 가 둘 다 0.00 이 되는 센티널 샘플은
제외(raws3=0개 ~ rawos2=20개/104). 둘 다 0 인 행은 "근거리부터 보임"이 아니라
"중앙축이 아예 안 보임"이므로 그대로 평균 내면 안 된다.

†`vis%`=`dataset.csv`의 `visible_pct`(sample 전체 대비 가시 셀 비율, `vis_start`와 달리 중앙축이
아닌 BEV 전체 기준). `vis<5% 건수`는 그 sample 의 절반이 통째로 마스킹 대상급으로 어둡다는 뜻.
raws3·rawos3·rawos4(vis% 중앙 17~19%)는 건강한 대역이지만, 나머지 4개 bag(raws1·raws2·
rawos1·rawos2)은 vis% 중앙이 1~4%로 훨씬 낮다 — 이 4개가 694개 중 다수(대략 절반 이상)를
차지해 전체 중앙값을 5.0%까지 끌어내린다. §알려진 한계 — visibility 수율 참고.

`z_ref`·obstacle·`cam_ok`·`vis_start` 는 7개 bag 전부 raws3 기준선 범위 안. rawos 4종의
world z 드리프트는 body crop 이 예상대로 상쇄해 `z_ref` 가 raws 와 같은 대역에 남는다.
`cam_ok` 도 어느 bag 도 100%에 가깝지 않아 투영 평면 오류 신호 없음.

`reach_far` 만 bag 마다 갈린다 — raws3·rawos3·rawos4 는 80%대(82~87%)로 기준을 만족하고,
rawos1 은 77%로 근접, raws2 는 62%, **raws1(21%)·rawos2(14%) 는 크게 미달**한다. 원인은
`_sheet_review.png` 로 확인: raws3·rawos3·rawos4 는 중간 구간 대부분이 폭 넓은 부채꼴
가시 영역(코너 근처를 제외한 통로 폭 상당 부분이 초록)인 반면, raws1·rawos2 는 중간 구간
대부분이 폭이 좁은 한 줄짜리 초록 세로선이다 — 중앙 몇 칸만 보이고 그 옆은 바로 갈색
(가려진 장애물)이라는 뜻으로, 전방 1~3m 안에 실제 장애물(통로 폭이 더 좁거나 화분·잎이
안쪽으로 튀어나온 구간)이 자주 있다는 신호다. `z_ref`·obstacle·`cam_ok` 가 정상 범위인 채
`reach_far` 만 낮으므로 **파이프라인·매핑 결함이 아니라 그 구간 통로 자체가 좁거나 막혀
있다는 실측치**로 판단한다 — 라벨링 대상 bag 이 다른 통로/구간을 지나므로 발생하는
정상적인 bag 간 차이. 학습 시 이 차이를 인지하고 사용해야 한다(예: reach_far 낮은 bag 은
근거리 회피 판단 위주 샘플로 활용).

다만 통로가 좁다는 설명은 원인의 일부다 — raws1·rawos2 의 낮은 reach 샘플 상당수는 ego
셀 자체가 obstacle 이 되어 raycast 가 즉시 끊기는 현상과 겹쳐 있다(매핑 드리프트 + 슬래브
높이의 잎 돌출). 세 원인이 겹친 것이며 어느 하나로 환원되지 않는다 — 상세는 바로 아래
'알려진 한계 — visibility 전역 0 샘플' 절 참고.

### 알려진 한계 — visibility 수율 (전역 0 은 6.5%, 저수율 전반은 훨씬 넓다)

`visibility` 가 전역 0 이라 학습에서 통째로 마스킹되는 샘플이 있다. bag 별 분포:
raws1 14 / raws2 6 / raws3 **0** / rawos1 3 / rawos2 19 / rawos3 2 / rawos4 1(694 중 45개, 6.5%).
그중 ego 셀 자체가 obstacle 인 경우가 25개(3.6%)다.

**이건 전체 그림의 일부일 뿐이다.** "전역 0"만 보면 문제가 6.5%로 보이지만, 694개 전체의
`visible_pct` 중앙값은 **5.0%**이고 **50%(348개)가 5% 미만**이다(§B 7개 bag 검증 표의
`vis% 중앙`·`vis<5% 건수` 컬럼). "정상 샘플"이라 부르는 18~25% 대역(raws3·rawos3·rawos4)에
도달하는 건 26%(181개)뿐이다. 즉 §B 범위로 데이터셋 규모를 가늠하면, 실제 학습에 쓸 만한
고가시 영역은 여기 적힌 것보다 대략 한 자릿수 작게 잡아야 한다 — 특히 raws1·raws2·rawos1·
rawos2 4개 bag은 vis% 중앙이 1~4%대로 낮아 대부분의 샘플이 소량의 좁은 가시 영역만 갖는다.
그렇다고 이 셀들이 틀린 라벨은 아니다 — 가려짐을 정직하게 마스킹한 결과이며, 검증 기준선인
raws3 가 마침 7종 중 가장 건강한 bag이라는 점도 같이 감안해야 한다.

**메커니즘**: `raycast_visible` 은 ego 셀에서 출발해 첫 obstacle 셀에서 멈춘다. 그래서 ego 셀이
obstacle 이면 모든 ray 가 즉시 끊겨 visibility 가 전역 0 이 된다.

**원인은 셋이 겹친 것**이며 기하로 고칠 수 없다.
1. **좁은 통로**: 해당 bag 의 obstacle 셀 비율이 애초에 높다(raws1 26.9%·rawos2 28.1% vs raws3 22.2%).
2. **매핑 드리프트**: 낮은 reach 샘플이 연속 구간으로 뭉쳐 나온다(raws1 66~70·74·78).
3. **잎의 불규칙한 돌출**: 슬래브가 실제 0.87~1.67m 밴드라, 그 높이에서 잎이 통로로 넘어오면
   BEV 에서 통로가 실제보다 좁게 찍힌다. 바닥은 비어 있는데도 그렇다.

**ego 반경을 비우는 수정은 하지 않는다.** 실측: raycast 도달률이 0.9% → (r0.3 비움) 5.9%,
신뢰영역(r>0.7)만 보면 3.9% 다. 정상 샘플이 18~25% 이므로 회복이 미미하다. 지금처럼
`visibility=0` 으로 두면 라벨이 "여기서 학습하지 마라"를 정직하게 말하는데, 4% 짜리 어중간한
라벨로 바꾸면 걸러내기 어려워진다 — 명확한 실패가 애매한 성공이 되는 쪽이 QC 에 더 나쁘다.

**대응**: `overlay.png`(IPM+obstacle) 위에서 사람이 **occupancy 를** 보정한다. 통로를 잘못 막은
빨강을 지우면 되고, 바닥은 IPM 원본이 그대로 보이므로 판단 근거가 있다. **visibility 는 보정
대상이 아니다** — 고친 occupancy 로 `raycast_visible` 을 다시 돌리면 따라온다. 즉 여기 적힌
전역 0 문제도 occupancy 보정으로 함께 풀린다. 찾는 방법:

    awk '$4+0 < 1.0 {print $1, $4}' data/bev/slab/<name>/_stats.txt   # vis% < 1.0

**reach 가 낮은 bag 의 해석**: raws1 21%·rawos2 14% 는 4.0m 도달률이 낮지만, 그중 대부분이
이 현상이다(raws1 낮은 reach 20건 중 14건, rawos2 23건 중 19건). 통로가 좁은 것도 사실이고
그 위에 드리프트·잎 돌출이 겹친 결과다 — 어느 하나로 환원되지 않는다.

### 알려진 한계 — IPM 과 라벨 그리드의 정합 오차

의도적으로 손대지 않은 `ipm.py`(§ Constraints)에 이미 있던 특성 셋. 서로 다른 증상이지만
전부 같은 이음매(라벨 격자 ↔ IPM 투영)에서 나온다.

- **관측가능 판정과 IPM 투영이 FoV/마스크 경계에서 어긋날 수 있다.** `camera_observable`(정투영)과
  `ipm_project_rgb`(역투영)은 서로 다른 계산 경로라, 경계 부근 셀은 `visibility=1`인데 IPM
  데이터가 없어(검정) `overlay.png`에서 색이 무데이터 검정 위에 얹힐 수 있다.
- **반 셀(2.5cm) 오프셋**: `bev_label.rc_of`는 `np.floor`로, `ipm.ipm_project_rgb`는
  `np.round`로 좌표→셀 인덱스를 계산한다(`RES`=0.05m 기준 반 셀 차이). `overlay.png`가 사람이
  직접 그리는 CVAT base이므로, 라벨 경계와 IPM 바닥 텍스처가 딱 그 셀만큼 어긋나 보일 수 있다.
- **지면 평면 기준점이 두 곳**: `--ground-offset`(관측가능성 판정, 기본 0.87)과 `--cam-height`
  (IPM 투영, 기본 0.87)는 이름은 다르지만 같은 지면을 가리키려는 값이다. 그런데 전자는
  `z_ref − ground_offset`(**sample 별** LiDAR 하위1% 기준)로, 후자는 `C_z − cam_height`
  (**카메라 광중심 고정** 기준)로 계산돼 기준면 자체가 다르다. 694개 샘플의 `z_ref` 실측 범위는
  −0.090~+0.116m 로, 두 평면이 최대 ~12cm 벌어질 수 있다는 뜻이다. 둘 다 기본값이 같다고
  한쪽만 조정하면(예: `--cam-height`만 바꾸기) 이 간극이 더 벌어진다.

셋 다 라벨 자체를 틀리게 만들진 않지만, 사람이 `overlay.png`를 보고 손으로 보정할 때 "라벨
경계가 왜 바닥 텍스처와 안 맞지"라고 헷갈릴 수 있는 지점이다 — 보정 시 셀 1개 안팎의 어긋남은
이 정합 오차이지 라벨 오류가 아니라는 점을 감안한다.

---

# 부록 A. 구판 경로 (2026-07, 보관용)

> **여기부터 문서 끝까지는 구판이다.** LiDAR 라벨은 위 §B 슬래브 라벨이 대체했다 —
> 새 데이터를 만들 때 아래 §A 의 `verify_labels.py`·`generate.py`·`gather_annotations.py`
> 경로를 따라 하지 말 것. 그리드도 80×80 이라 §B 의 120×120 과 섞이지 않는다.
>
> **그래도 남겨두는 이유** ① `data/bev/dataset/` 에 있는 기존 80×80 산출물을 해석하려면
> 필요하다. ② **좌표 규약(§3)·BEV 규격(§4)·카메라 선택(§5)·특이사항(§7)·환경 핀(§7.I)은
> §B 도 그대로 쓴다** — §B 본문이 이 절들을 참조하는 것은 이 때문이다. ③ IPM 투영(`ipm.py`)
> 은 §B 가 그대로 재사용한다.
>
> **구판에만 있고 §B 에는 없는 것**: 수직성(`--z-gate`) obstacle 판정, 국소 floor 추정,
> **corridor 무조건 drivable**, `keep_ego_connected`. §B 는 이 중 corridor prior 를
> 의도적으로 쓰지 않는다(§B "하지 말 것" 참조 — 부활 금지).

---

## A. 실행 가이드 (CLI) & 구현 상태 — **구판**

### A.0 구현 상태 (2026-07-28)
- 파이프라인 CLI 구현 완료: **`calibration/bev_autolabel/`**
  - `bev_label.py`(순수 라벨 로직), `bev_io.py`(맵·stamp·이미지 IO), `ipm.py`(**이미지→지면 IPM RGB 투영**),
    `render.py`(색칠·미터축 검수뷰·IPM 오버레이뷰), `verify_labels.py`(**단계1** 검증 CLI),
    `generate.py`(**단계2** 데이터셋+IPM CLI), `gather_annotations.py`(**단계3** CVAT 업로드용 오버레이 모음),
    `test_bev_label.py`·`test_ipm.py`·`test_render.py`(29 테스트).
  - 재사용: `calibration/cam_lidar/{chain,cloud_io,calib_io}.py`, `calibration/verify/ds_model.py`.
- **폐기(2026-07-28)**: 카메라-마스킹 경로 `ipm_review.py`(마스크 IPM+융합)·`dataset_flatten.py`(export/gather-cvat).
  마스크를 사람이 카메라에 그리는 대신 `generate.py`가 IPM 배경+라벨을 미리 얹어 주므로 불필요해짐(§A.6).
- 검증: raws3 16프레임 + 타 bag 4종(raws1/raws2/rawos2/rawos4, with/without-sun) 일관 확인. 순수 로직 테스트 29+18+7 pass.

### A.1 PoC(§6) 대비 최종 변경사항 — **§6·§8보다 이 표가 최신**
| 항목 | PoC 서술(§6) | 최종 구현 |
|---|---|---|
| obstacle 판정 | 고정 낮은밴드 `[floor+0.1, 1.0]` | **수직성 테스트**: 셀의 `z_min ≤ floor_cell+z_gate`(기본 0.3) **AND** `z_max−z_min ≥ 0.5m` |
| floor | 전역 z 2퍼센타일 | **국소 격자**(~1m 윈도별 2퍼센타일, 빈 윈도 전역 백필). 윈도는 **ego 미터좌표에 고정**(격자 범위와 무관) |
| observed(보이는 곳) | 360° ray-cast 단독 | **카메라 FoV(front/left/right 지면점 투영) ∩ ray-cast 가림** |
| ego self 반경 | <0.65 m | **<0.28 m**(40×40cm 반대각; 옆 기둥 보존) |
| corridor(주행 궤적) | 전방만, observed 게이트 | **무조건 drivable**(ground-truth) — 마스트-아래 ego 지면점이 카메라 FoV 가장자리라 observed가 ego에서만 False가 되는 artifact 때문에 게이트 시 `keep_ego_connected`가 drivable을 전멸시킴 |
| 마무리 | (TODO) | **ego 연결 성분만 남김**(`keep_ego_connected`) 구현 |
| min_pts/min_extent | 실질 레버로 서술 | 이 데이터에선 무효(맵 조밀·작물 키큼). **z_gate가 유일한 실질 레버** |

### A.2 사전 준비 (bag당 1회)
1. **매핑**: bag → `map.pcd`(+`pcd_denoise.py`로 `map_clean.pcd`) + `trajectory.tum`
   (Point-LIO, `docs/MAPPING.md`). 260722는 이미 `data/sj_bags/260722/maps_selfmask/` 에 7종 완비.
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
  --map-dir     ../../data/sj_bags/260722/maps_selfmask/raws3_mapping \
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
  --map-dir     ../../data/sj_bags/260722/maps_selfmask/raws3_mapping \
  --extract-dir ../../data/extracted/raws3 \
  --calib       ../../data/calib_260723/calib.yaml \
  --orient      ../../data/calib_260723/orientation.json \
  --out         ../../data/bev/dataset/raws3 \
  --kf-step 0.4        # 키프레임 이동거리 간격(m)
  --cam-height 0.87    # IPM 지면 평면용 카메라 렌즈 높이[m] 실측(마스트 LiDAR가 바닥을 못 봐 자로 측정)
  # --z-gate 0.3       # obstacle 바닥근접 여유(기본 0.3; 0.15면 통로 더 개방)
  # --alpha 0.45       # review 라벨 오버레이 불투명도
  # --blend nearest    # IPM 다중카메라 합성: nearest(기본, 셀별 최근접 1대) | average(평균)
  # --limit 3          # 스모크: 앞 N개만
```
→ `<출력>/sample_NNNNNN/` 마다:
- **`label.png`** — 순수 class(0/1/2) **인덱스 팔레트**(오버레이 없음) = **재라벨링 원본**(사람이 이걸 보정).
- **`ipm_rgb.png`** — 3어안을 지면 평면에 IPM 투영한 **80×80 BEV RGB 캔버스**(위에서 본 주행면). 라벨 보정 배경.
  다중카메라 합성은 기본 `nearest`(셀별 최근접 카메라 1대 → 겹침 유령상 감소·텍스처 선명), `--blend average`로 평균 전환 가능.
- **`overlay.png`** — `ipm_rgb`에 라벨 반투명 오버레이, **네이티브 80×80·장식 없음** = **CVAT annotation base**.
  여기에 라벨링한 마스크가 곧 80×80 정답이라 resize 왕복이 없다(격자·ego 같은 장식은 셀을 덮으므로 넣지 않음).
- **`review.png`** — 상단 원본 3어안(좌·전·우) + 하단 확대 검수뷰(overlay를 9배 확대 + 미터축·격자·ego). 사람 눈 검수용.
- **`cam_{front,left,right}.jpg`** — 원본 3이미지.
- **`meta.json`** — pose(`world_T_body`)·stamp·BEV 규격·사용 파라미터(z_gate·kf_step·cam_height·blend).
- 그리고 최상위 **`dataset.csv`**(sample↔frame_idx↔stamp). 이미지 결손 키프레임은 건너뛰고 번호는 연속 유지.

### A.5 파라미터 조정
`--z-gate`(obstacle 바닥근접 여유)만 실질적 레버다. 0.6은 통로 위 캐노피 오검(비추), 0.15는 통로 개방, 0.3 절충(기본).
`min_pts`·`min_extent`는 이 환경에선 무효.
**BEV 범위**는 `--xf/--xr/--yh`(기본 3.0/1.0/2.0 = 80×80)로 조절한다. `RES`(0.05 m/cell)는 고정(§4).
쓴 범위는 각 sample 의 `meta.json`(`bev`)에 기록되고, `gather_annotations.py` 는 그 값을 읽어 review 를 그린다.

### A.6 단계3 — 사람 검수·보정 (BEV 위에서 라벨 수정)
`generate.py`가 이미 **IPM 배경(`ipm_rgb.png`) + LiDAR auto-label(`label.png`)** 을 만들어 `review.png`로 겹쳐
보여주므로, **별도의 카메라 마스킹·IPM 투영 단계가 없다.** 사람은 다음만 하면 된다:

1. `review.png`(상단 3어안 + 하단 IPM 캔버스+라벨 오버레이)를 보고 각 sample 을 판단.
2. **`gather_annotations.py`로 dataset의 `overlay.png`(=IPM 배경+라벨 오버레이, 네이티브 80×80·장식 없음)를
   한 폴더로 모아 CVAT 등에 업로드**, 그 위에서 drivable/obstacle 경계를 보정한다:
   ```bash
   cd calibration/bev_autolabel
   python3 gather_annotations.py \
     --dataset ../../data/bev/dataset/raws1 \
     --out ../../data/bev/annotations
   # dataset 하나당 폴더 하나(raws1/) + 하위 2개:
   # → data/bev/annotations/raws1/label/sample_NNNNNN.png  (80×80) — 이 폴더를 그대로 CVAT 업로드
   # → data/bev/annotations/raws1/review/sample_NNNNNN.png — 참고용 확대 검수뷰(원본 3어안+BEV)
   ```
   **네이티브 해상도(기본 80×80)라 CVAT 마스크가 곧 정답** = resize 왕복 없음(격자·ego 장식은 review 로만 확인).
   `review/` 는 sample 폴더를 하나씩 열지 않고 **한 곳에서 훑어보기 위한 참고뷰**(카메라를 크게, 기본
   `--review-scale 18`; 0이면 생략). 업로드용 `label/` 과 하위 폴더로 분리돼 CVAT 업로드에 섞이지 않고,
   dataset 마다 폴더가 하나로 묶여 여러 dataset 을 모아도 경로가 엉키지 않는다.
   보정 결과(세그멘테이션 마스크)가 **최종 정답**이다. `label.png`(80×80 인덱스 0/1/2)는 재라벨링 원본으로 그대로 남는다.

- **주된 보정 패턴**: auto-label 의 drivable(초록)은 궤적 corridor 기반이라 **실제 통로보다 약간 좁다** →
  초록을 좌우 장애물(빨강) 경계까지 넓히는 것이 대부분. 나머지는 대체로 맞음.
- **왜 IPM 배경으로 충분한가**: 통로는 평면이라 IPM 번짐이 없어 **바닥이 정확히 펴진다**(작물 등 수직물만 방사상 번짐).
  마스트 LiDAR가 바닥을 못 보는 사각을 카메라 IPM이 메워, "위에서 본 주행면"을 근사한다(`--cam-height` 실측 필요).
- **한계**: IPM은 평면 가정이라 수직물체 번짐·원거리 부정확, LiDAR auto-label 은 국소 drift 가능 — **둘 다 100% 아님**.
  그래서 이 단계(사람 보정)가 최종 정답을 만든다. auto-label 은 노동을 줄이는 초안일 뿐이다.
- 폐기된 마스킹 경로(`ipm_review.py`·`dataset_flatten.py`, CVAT 카메라 마스크 왕복)는 더 이상 쓰지 않는다(§A.0).

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
| LIO 맵 | `data/sj_bags/260722/maps_selfmask/raws3_mapping/` | `map.pcd`(월드 밀집 클라우드, 259.9만점), `map_clean.pcd`(고립점 제거, 259.8만점), `trajectory.tum`(궤적 42.6m) |
| 캘리브 | `data/calib_260723/calib.yaml` | DS intrinsic 4대 + 카메라간 extrinsic(`T_cam_front`) + **`T_front_lidar`** |
| 카메라 방향 | `data/calib_260723/orientation.json` | 카메라 idx↔front/right/rear/left 매핑 |

- **폴더 규칙(bag별 3쌍)**: `sj_bags/260722/bags/<원본bag>` ↔ `sj_bags/260722/maps_selfmask/<name>_mapping`(LIO 산출)
  ↔ `data/extracted/<name>`(추출 이미지). `<name>`: `raws{N}`=with-sun, `rawos{N}`=without-sun.
  라벨 1건 = `--map-dir maps_selfmask/<name>_mapping` + `--extract-dir extracted/<name>` 쌍을 같은 `<name>`으로 맞춘다.
  구 `sj_bags/260722/maps/`는 self mask·drain 수정 이전 버전으로, **대조용 보관본이니 라벨 입력으로 쓰지 않는다.**
- **맵 건강성 확인 필수**: `trajectory.tum` 총 길이를 실제 온실과 대조(궤적 붕괴 시 라벨 오염).
  길이는 **2 Hz로 다운샘플한 뒤** 재라 — raw(~3000 Hz) 연속 차분 합은 지터로 3배 부풀려진다.
  raws3 재매핑본은 2 Hz 42.6m·시종점 직선 36.9m·범위 ~12m×35m = 건강. 7종 실측치는 `docs/MAPPING.md §6.7`.
- **`map.pcd` vs `map_clean.pcd`**: 후자는 고립 flyer(`d4>0.3m`, 0.05~0.10%)를 뺀 것으로,
  BEV 라벨 입력으로는 **`map_clean.pcd`가 맞다**(떠 있는 단독점이 허위 obstacle 셀이 된다).
  단 현재 `bev_io.load_map()`은 파일명 `map.pcd`를 하드코딩하고 있다 — 라벨 로직을 다시 짤 때
  입력 파일명을 인자로 받도록 바꿀 것.
- **rawos 계열 z 경사**: rawos 4종은 종단 z가 −0.9~−1.9m 기운다. 대부분 맵 전체의 강체 기울기라
  ego-local 크롭에는 불일치분 ≈6cm만 남아 `z-gate`(0.3m) 아래지만, **전역 맵 좌표로 z를 판단하는
  로직을 새로 넣는다면 반드시 고려**할 것. 근거: `docs/MAPPING.md §6.7`.

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
- 위 값은 **기본값**이고 `verify_labels.py`·`generate.py` 의 `--xf/--xr/--yh` 로 바꿀 수 있다
  (예: 5m×5m=100×100 → `--xf 3.5 --xr 1.5 --yh 2.5`). **`RES`(0.05)는 고정** — 셀 단위로 박힌
  상수(obstacle morph 3×3, ray-cast 0.5°)가 함께 스케일되지 않아 판정이 바뀐다.
- 범위는 `RES` 의 정수배여야 한다(아니면 `BevSpec` 이 `ValueError`).
- 기존 산출물(`data/bev/dataset/*`·`annotations/*`)은 전부 80×80이므로 범위를 바꾸면 섞이지 않게 따로 모을 것.
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
| `XF, XR, YH` | 3.0, 1.0, 2.0 m | BEV 전/후/좌우 범위 (**기본값**, `--xf/--xr/--yh` 로 변경) |
| `RES` | 0.05 m | 셀 크기 (**고정**, 기본 범위에서 그리드 80×80) |
| ego 셀 | (60, 40) | x=0,y=0 위치 |
| self 반경/지속성 | **<0.28 m** / >60% pose | 카트 판정(40×40cm 반대각) |
| self 복셀 `VOX` | 0.15 m | self-mask 양자화 |
| self 샘플 pose 수 | 150 | 균등 샘플 |
| 지역 크롭 반경 | 6 m (수평) | 맵→ego 프리필터 |
| `floor` | **국소(~1m 윈도) z 2 퍼센타일** | 바닥 높이 격자. 윈도는 ego 미터좌표 고정 |
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

## 10. 구현 완료 (2026-07-28) & 향후

- CLI 구현 완료: **`calibration/bev_autolabel/`** (실행법 §A). 단계1 `verify_labels.py` + 단계2 `generate.py`(라벨+IPM 통합).
- 출력 규칙 확정: 샘플당 `label.png`(class 인덱스 팔레트) + `ipm_rgb.png`(IPM RGB 캔버스) + `review.png`(검수 오버레이뷰,
  원본 3어안 포함) + `cam_{front,left,right}.jpg` + `meta.json`, 최상위 `dataset.csv`. 키프레임 = 이동거리 0.4m 간격.
- **워크플로우 전환(2026-07-28)**: 카메라 마스킹 경로(`ipm_review`·`dataset_flatten`) 폐기. 사람은 IPM 배경 위 라벨을
  BEV에서 보정만(§A.6). 마스킹 annotation·flatten/gather 불필요.
- 의존 모듈(`calibration/cam_lidar`, `calibration/verify`)은 브랜치에 포함됨.
- **향후**: ①여러 bag/환경 확충(일반화) ②전방 동적물체(§7-E) 처리 ③어안→모델입력 언디스토션(§7-H)
  ④BEV 라벨 보정 툴로 최종 데이터셋 확정.

---

## A-부록: PoC 참조 구현 (footprint + ray-cast, 구판 최종본)

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
