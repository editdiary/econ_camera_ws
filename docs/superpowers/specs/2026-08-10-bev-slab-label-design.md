# BEV 슬래브 라벨 설계 (LiDAR 라벨 로직 전면 교체)

날짜: 2026-08-10 · 브랜치: `feat/bev-slab-label`

## 1. 목적과 범위

`data/sj_bags/260722/maps_selfmask/<name>_mapping/map_clean.pcd` 와 `trajectory.tum` 에서
키프레임별 **BEV occupancy 라벨**과 **BEV visibility 라벨**을 생성한다.

기존 `calibration/bev_autolabel/bev_label.py` 의 LiDAR 라벨 로직(국소 바닥 추정 → 수직성
obstacle → corridor prior → FoV∩raycast → 0/1/2 단일 라벨)을 **대체**한다. 새 파일로 만들고
기존 파일은 손대지 않는다(기존 데이터셋 재현성 보존, 대조 가능).

**범위 밖**: IPM RGB 캔버스 생성(`ipm.py`), 이미지 추출, CVAT 묶음(`gather_annotations.py`),
`generate.py`. 이번 변경은 LiDAR 라벨만 다룬다.

**입력 데이터**: `maps_selfmask/` 만 사용한다. 구 `maps/` 는 쓰지 않는다.

## 2. 핵심 설계 판단과 그 근거

모든 수치는 `map_clean.pcd` 실측이다. 범위는 XF=4·XR=2·YH=3·RES=0.05(=120×120),
슬래브 0.8m, 하위 1% 기준.

### 2.1 3D crop 을 body 프레임에서 먼저 한다

rawos 계열은 world z 가 궤적 전구간에서 선형으로 −0.9~−1.9m 흐른다. 그런데 pose 로 body
프레임에 넣으면 이 드리프트가 상쇄된다:

| bag | pose_z(world) 변화 | body z p1 (궤적 5지점) |
|---|---|---|
| raws3 | 0.00 → +0.15 | +0.05 ~ +0.08 |
| rawos1 | +0.02 → **−1.72** | +0.01 ~ +0.07 |
| rawos3 | −0.04 → −0.89 | +0.02 ~ +0.07 |

body z p1 이 드리프트와 무관하게 고정이다. 따라서 crop 을 body 프레임에서 하고 z 기준도
crop 안에서 다시 구하면 전역 드리프트에 면역이 된다. crop 내 잔여 경사는 슬래브 0.8m 에
비해 무시할 수준이다.

### 2.2 슬래브 하단 z_ref = crop 전역 하위 p% (기본 p=1.0)

LiDAR 가 바닥을 못 보고 자기 수평면 근처부터 돔 형태로 수집하므로, crop 안 최저 z ≈ LiDAR
수평면 ≈ 실제 지상 0.87m 다. 슬래브 `[z_ref, z_ref+0.8]` 은 실세계 약 0.87~1.67m 밴드,
즉 **로봇이 통과해야 하는 높이 구간**이다.

기본 p 는 5 가 아니라 **1** 이다. p5 는 +0.11~0.25 로 슬래브 바닥을 최대 0.25m 들어올려
실제 하위 점을 잘라먹는다. p1 은 +0.01~0.08 로 안정적이고, min(−0.2~−0.7, 이상치)보다
훨씬 강건하다.

슬래브를 flatten 앞에 두므로 **로봇보다 높은 장애물은 자동 배제**된다(열린 문·상인방·천장·
머리 위 배관·높은 잎). 반대로 슬래브 **아래**(실제 0.87m 이하, 바닥의 상자·낮은 벤치)는
LiDAR 가 애초에 못 봐 맵에 없으므로 drivable 로 나온다 — 수집 단계의 한계이며 이 파이프라인
에서 고칠 수 없다. 그 정보는 IPM RGB 에만 있다.

### 2.3 occupancy = 2D 기둥 누적 count ≥ 3

슬래브 점을 (row,col) 로 눌러 셀당 점 개수가 임계 이상이면 obstacle. 임계는 데이터로 정했다.
판정 기준은 **카트가 실제 지나간 궤적 셀이 obstacle 로 찍히는 비율**(작으면 라벨이 실제
주행과 모순되지 않는다):

| 임계 | obstacle 셀 비율 | 궤적셀이 obstacle 인 비율 (raws3 / rawos1 / rawos4) |
|---|---|---|
| N≥1 | 32.6~36.4% | 2.8% / 4.8% / 3.9% |
| **N≥3** | **22.4~27.6%** | **0.2% / 1.2% / 0.6%** |
| N≥10 | 13.4~18.9% | 0.0% / 0.6% / 0.0% |
| N≥30 | 6.8~10.7% | 0.0% / 0.0% / 0.0% |

N=1 은 노이즈가 새고(2.8~4.8%), N≥10 은 obstacle 셀을 크게 줄여(raws3 23.8%→14.9%)
실구조물까지 지운다. **N=3 이 기본값**이다.

### 2.4 corridor prior 를 쓰지 않는다

기존 로직은 궤적 주변을 무조건 drivable 로 강제했다. 2.3 의 측정에서 N=3 일 때 궤적셀의
obstacle 비율이 0.2~1.2% 에 불과하므로 그런 보정이 필요 없다. prior 없이도 라벨이 실제
주행과 일치한다.

### 2.5 self(카트·수집자) 점 제거를 하지 않는다

Point-LIO 의 `pcd_save.self_mask_*` 가 이미 매핑 단계에서 제거했다. 실측 결과 ego 셀은 항상
비어 있어(`ego_occ=0`, 6지점 전부) raycast 원점이 막히지 않고, ego 반경 0.3m 점을 추가로
지워도 visibility 총량이 0.1~5.2%p 밖에 안 변한다. 반면 넓게 지우면 좌우 0.33~0.58m 의
실제 구조물(좁은 통로 벽)을 갉아먹는다. 따라서 추가 제거를 하지 않는다.

### 2.6 visibility = 2D raycast ∧ 카메라 관측가능성

**2D raycast** (3D 아님). ego 셀에서 360° ray 를 쏘고 첫 obstacle 셀까지(포함) visible.
투과율 0 이므로 첫 obstacle 에서 정지한다.

3D 를 쓰지 않는 이유: ray 원점은 LiDAR 원점(body z=0)이고 z_ref 도 +0.01~0.08 이라 **원점이
슬래브 밑면에 붙는다**. 그래서 3D ray 중 elevation≈0 인 것들이 맨아랫층을 타고 장애물 아래로
빠져나가고, "열에 닿은 voxel 이 하나라도 있으면 visible" 이면 거의 전역 1 이 되어 목적을
상실한다. 원점을 위로 올리거나 아래쪽 ray 를 막아도(돔 제한) 문제의 ray 가 바로 그
수평 ray 라서 해결되지 않는다. occupancy 가 2D 기둥이므로 2D raycast 가 정의상으로도 일치한다.

나중에 투과율·3D visibility 가 필요해지면 `slab.pcd` 가 남아 있으니 맵 재로드 없이 덧붙일 수 있다.

**카메라 관측가능성**은 항상 적용한다. 학습 입력이 3어안 이미지이므로 "이미지에서 볼 수 있는
영역"과 라벨 정의가 일치해야 한다. 지금 리그가 사실상 360° 를 덮더라도, 배치를 바꾸거나 BEV
범위를 키우면 조용히 틀린 라벨이 나온다. 그래서 끄지 않고, 대신 **커버리지 %를 매번 로그에
찍어** no-op 인지 실제로 걸러내는지 보이게 한다.

### 2.7 카메라 관측가능성은 지면 평면에서 판정한다 (사각지대의 실체)

셀을 투영할 z 는 `z_ref − ground_offset`(기본 0.87, IPM 의 `cam_height` 와 동일 실측값)이다.
이것이 IPM 이 쓰는 지면 평면과 같으므로 두 라벨이 같은 기하를 공유한다.

평면 선택이 결정적이다. body z=0 은 **카메라 높이 = 수평선 평면**이라 그 위의 점은 이미지
세로 중앙 근처에 찍혀 항상 유효해 보인다. 실제 지면에서 재야 사각지대가 드러난다:

| 투영 평면 | 3어안 FoV | r<0.5m | r 0.5~1.0m | r>1.0m |
|---|---|---|---|---|
| z=0 (수평선, **무효한 측정**) | 100% | 98% | 100% | 100% |
| z=−0.87 (**실제 지면**) | 96.0% | **0%** | 72% | 100% |

카메라 렌즈가 수평 바깥을 보게 장착돼 아래를 충분히 못 내려다본다. 그래서 **반경 0.5m 는
완전 사각, 1.0m 까지 부분 사각**이다. ego 크기보다 훨씬 크다.

### 2.8 카트 자기 가림은 카메라별 이미지 마스크로 처리한다

기하 FoV 로 안 잡히는 두 번째 사각지대가 있다. 카메라가 내려다본 자리를 카트 자기 몸이
가린다 — 상판(연한 갈색), 프레임 파이프, 핸들, 핸들을 잡은 수집자의 팔. 실제 이미지와 기존
`ipm_rgb.png` 양쪽에서 확인된다(IPM 캔버스에는 상판이 지면에 갈색 사각형으로, 수집자 팔이
파란 덩어리로 번져 있다).

지면 반경 ↔ 이미지 높이 대응(FoV 안 셀의 대표 v/h):

| 지면 반경 | v/h p10~p90 |
|---|---|
| 0.5~0.75m | 0.945 ~ 0.992 |
| 0.75~1.0m | 0.874 ~ 0.964 |
| 1.0~1.25m | 0.813 ~ 0.917 |
| 1.5~2.0m | 0.707 ~ 0.798 |
| 2.0~3.0m | 0.635 ~ 0.708 |

상판이 대략 v/h>0.88 부터라, 가려지는 셀이 기하만의 4.0% 에서 **9.7%** 로 늘고 사각 반경이
1.0m → 약 1.5m 로 확장된다.

**BEV 가 아니라 이미지 공간에 마스크를 둔다.** 이유:

1. 카메라 3대의 가림이 서로 다르다. 어떤 셀은 front 에서 상판에 막히고 left 에서는 보인다.
   올바른 판정("적어도 한 대가 보고, 그 카메라가 자기 몸에 안 막혔다")의 per-camera OR 은
   마스크가 각 카메라 이미지에 있어야 계산된다. 핸들 파이프와 팔은 좌우 비대칭이다.
2. BEV 마스크는 지면 평면 가정과 특정 BEV 범위·해상도에 묶여, 범위나 `ground_offset` 을
   바꾸면 다시 그려야 한다. 이미지 마스크는 리그의 속성이라 영구 재사용된다.
3. 상판은 이미지에서 경계가 뚜렷한 한 물체지만, BEV 로 눌리면 세 카메라의 가림이 한 덩어리로
   섞여 원인 구분이 불가능하다.

대안인 "사각 반경 하나로 뭉개기"(`blind_r=1.5`)는 19.6% 셀을 버린다. 마스크의 9.7% 대비 두 배
손실이고, 상판이 없는 방향까지 감은 것으로 취급한다. 근거리 라벨이 가장 값지므로 마스크를 쓴다.

**어안 유효원 바깥 비네팅도 무효 처리한다.** 각 이미지의 12~18%(좌우단 열에서는 60~66%)가
어안 원 바깥 검은 영역인데, 현재 판정은 지면점이 1280×720 사각형 안에만 들어오면 보인다고
센다. 이 원은 프레임 다수의 밝기 합집합으로 **자동 검출**하므로 수작업 대상이 아니다.

#### 마스크 규격 (사람이 만드는 유일한 산출물)

| 항목 | 값 |
|---|---|
| 파일 | `mask_front.png`, `mask_right.png`, `mask_left.png` (cam2=rear 는 미사용) |
| 위치 | `data/calib_260723/self_mask/` (`--self-mask-dir` 로 변경 가능) |
| 크기 | 1280×720 (원본 해상도) |
| 값 | 흰색 255 = 카트 자체(무효), 검정 0 = 사용 가능. 컬러여도 >127 로 판정 |
| 칠할 것 | 상판, 프레임 파이프, 핸들, 손·팔이 상시 오는 구역 |
| 안 칠해도 됨 | 어안 유효원 바깥 검은 영역 (자동 검출) |

마스크가 없어도 파이프라인은 돈다. 그 경우 **근거리 visibility 가 낙관적**이라는 경고를 로그에
찍고 `meta.json` 에 `self_mask: null` 을 기록한다. 마스크가 준비되면 라벨만 재생성한다.

### 2.9 occupancy 에 unknown 클래스를 두지 않는다

"한 번도 관측되지 않은 영역"은 `visibility=0` 이 정확히 그 뜻이다. occupancy 에 또 두면 같은
사실을 말하는 채널이 둘이 되어 어긋날 수 있다. 두 채널의 네 조합이 각각 의미를 갖는다:

| | vis=1 | vis=0 |
|---|---|---|
| occ=obstacle | 보이는 장애물 표면 | 가려진 장애물 |
| occ=drivable | **학습에 쓸 신뢰 영역** | 미관측 → loss 마스킹으로 제외 |

학습에서 visibility 를 loss 마스크로 쓰면 미관측이 자동 배제된다. 다른 unknown 정의(점 0개
vs N미만)가 필요해지면 `slab.pcd` 에서 재산출 가능하다.

### 2.10 시간 윈도우가 필요 없다

전역 맵을 crop 하면 다른 패스의 점이 섞일 수 있으나, 7개 bag 전부 재방문이 없다(키프레임
0.4m 리샘플 후 경로거리 8m 이상 떨어진 두 지점이 1.5m 안에 오는 경우 = **0%**). 궤적
39.2~41.7m 를 한 방향으로 통과한다. 따라서 시간 윈도우 없이 전역 맵을 그대로 crop 한다.

## 3. 구조

| 파일 | 역할 |
|---|---|
| `calibration/bev_autolabel/slab_label.py` | 순수 로직: crop → slab → occupancy → visibility. numpy 만, 파일·하드웨어 불필요 |
| `calibration/bev_autolabel/generate_slab.py` | CLI: 맵+궤적+stamp → 샘플별 산출 |
| `calibration/bev_autolabel/test_slab_label.py` | 순수 로직 pytest |

**재사용(복붙 금지)**: `mapping/pcd_denoise.py` 의 `read_pcd_raw`/`write_pcd_raw`(8필드 보존 —
open3d 경유하면 intensity 소실), `calibration/cam_lidar/cloud_io.py` 의 `load_tum`·`pose_at`,
`chain.py` 의 `se3_inv`·`transform`·`project`, `bev_autolabel/bev_io.py` 의 `load_stamps`,
`bev_label.py` 의 `select_keyframes`·`BevSpec`·`rc_of`·`cell_centers`.

`bev_label.py`·`generate.py` 는 수정하지 않는다.

## 4. 파이프라인 (키프레임 1개)

1. `pose_at(t)` → `T_wb`, `T_bw = se3_inv(T_wb)`
2. world 맵을 pose 중심 bbox 로 사전 필터 → body 프레임 변환
3. **crop**: `−XR ≤ x ≤ XF`, `|y| ≤ YH`, z 무제한 → `crop.pcd`(옵션)
4. **slab**: `z_ref = percentile(crop z, PCT)`, `z_ref ≤ z ≤ z_ref + THICK` → `slab.pcd`
5. **occupancy**: `(row,col)` 히스토그램, `count ≥ MIN_PTS` → obstacle
6. **camera-observable**: 각 셀을 `z_ref − GROUND_OFFSET` 평면에 놓고 front/left/right 로 투영.
   DS 유효 ∧ 이미지 안 ∧ 어안 원 안 ∧ self 마스크 밖 → 카메라별 판정의 OR
7. **visibility**: ego 셀에서 2D 360° raycast(첫 obstacle 셀 포함까지) ∧ camera-observable
8. 저장

좌표 규약은 기존과 동일: `row = (XF − x)/RES`, `col = (YH − y)/RES`. 하류 IPM·
`gather_annotations` 와 호환된다.

## 5. 산출물

```
<out>/
  dataset.csv
  sample_NNNNNN/
    slab.pcd        # body 프레임, 8필드 보존
    crop.pcd        # --save-crop 시에만
    occupancy.png   # 인덱스 팔레트, 0=obstacle 1=drivable
    visibility.png  # 인덱스 팔레트, 0=unseen 1=visible
    review.png      # 확대 검수뷰(4색 + ego 마커 + 격자)
    meta.json
```

`occupancy.png` 는 기존 `label.png` 와 같은 약속(0=obstacle, 1=drivable)을 쓴다. 프로젝트
전역이 한 약속을 유지해 하류 도구의 지뢰를 없앤다.

`review.png` 4색: 초록=visible drivable, 빨강=visible obstacle, 갈색=occluded obstacle,
검정=unseen.

`meta.json`: `frame_idx`, `stamp_ns`, `world_T_body`, `bev`(XF·XR·YH·RES·NX·NY·R_EGO·C_EGO),
`z_ref`, `classes`, `params`(thick·pct·min_pts·ray_step·ground_offset·kf_step),
`self_mask`(경로 또는 null), `stats`(crop 점수·slab 점수·obstacle 셀 %·visible 셀 %·
camera-observable %).

`slab.pcd` 는 샘플당 약 2MB(7~14만점), bag 당 약 200MB. `crop.pcd` 는 30~62만점이라 기본 off.

## 6. 기본값 (전부 CLI 옵션)

```
--xf 4.0  --xr 2.0  --yh 3.0      # RES=0.05 고정 → 120x120
--thick 0.8                        # 슬래브 두께[m]
--pct 1.0                          # z_ref 퍼센타일
--min-pts 3                        # obstacle 임계
--ray-step 0.25                    # raycast 각도 스텝[deg]
--ground-offset 0.87               # 지면 평면(= IPM cam_height)
--kf-step 0.4                      # 키프레임 이동거리 간격[m]
--self-mask-dir <dir>              # 없으면 경고 후 진행
--save-crop                        # crop.pcd 저장
--limit N                          # 테스트용
```

## 7. 검증 계획 (단계별, 각 단계에서 멈추고 결과 확인)

| 단계 | 산출 | 판정 기준 |
|---|---|---|
| 1 | `crop.pcd`·`slab.pcd` + y-z 단면 PNG | 슬래브가 `[z_ref, z_ref+0.8]` 밴드로 정확히 잘림. z_ref 가 bag·위치와 무관하게 +0.0~0.1 범위 |
| 2 | `occupancy.png` | 궤적셀이 obstacle 인 비율 < 2%. obstacle 셀 비율 20~30% |
| 3 | `visibility.png` | 전방 중앙축 reach = XF 도달. 카메라 커버리지 로그가 기하 사각(r<0.5m=0%)을 반영 |
| 4 | bag 7개 일괄 | 요약 통계가 bag 간 일관. 실패 샘플 0 |

## 8. 테스트 (순수 로직, 하드웨어 불필요)

- crop 경계: RES 정수배 경계에서 포함/제외가 정확
- `z_ref` 퍼센타일 계산, 슬래브 밴드 상·하 경계 포함/제외
- occupancy `min_pts` 임계 (N−1 개는 drivable, N 개는 obstacle)
- raycast: 빈 격자 → 전역 visible / 단일 벽 → 벽 뒤 unseen / ego 셀 점유 시의 동작
- self 마스크: 전부 흰색 → camera-observable 전역 0 / 전부 검정 → 마스크 없을 때와 동일
- 어안 원 자동 검출: 원을 피팅하지 않고 프레임 스택의 밝기 퍼센타일로 '거의 항상 어두운'
  화소를 고른다(원 피팅보다 단순하고, 영구적으로 어두운 자기 구조물도 같이 잡힌다).
  합성 프레임(밝은 원 + 한 프레임만 튄 화소)으로 이상치에 안 속는지 확인
- `row`/`col` 규약이 기존 `rc_of` 와 일치

## 9. 남은 한계 (이 설계로 해결되지 않음, 기록용)

- 슬래브 아래(실제 0.87m 이하) 장애물은 맵에 없어 drivable 로 나온다. IPM RGB 에만 보인다.
- rawos 4종의 crop 내 잔여 z 경사. 슬래브 0.8m 대비 작아 비치명적이나 정량화는 안 했다.
- 수집자 팔은 위치가 완전히 고정이 아니므로 마스크가 보수적으로 넓어야 한다. 남는 잔재는
  BEV 에서 사람이 보정한다(기존 방침 유지).
- `ipm_rgb.png` 의 상판·팔 번짐은 그대로 남는다. 같은 마스크로 고칠 수 있으나 이번 범위 밖.
