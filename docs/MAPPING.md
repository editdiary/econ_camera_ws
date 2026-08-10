# 오프라인 매핑 가이드 (Point-LIO)

녹화된 bag을 후처리하여 **ego 궤적(pose) + 3D 맵**을 복원한다. 실시간 수집·녹화와 완전 분리된
오프라인 처리이며, BEV 학습 데이터셋의 전제(ego-motion)를 만든다.

## 1. 구성
- `src/point_lio` — Point-LIO(ROS2) 벤더 패키지. LiDAR-Inertial Odometry.
- `mapping/lio_map_bag.sh` — bag → 산출물 오케스트레이터.
- `mapping/pose_logger.py` — `/aft_mapped_to_init` → `trajectory.tum`.
- `mapping/pcd_preview.py`, `mapping/bev_grid.py` — 헤드리스 PNG 시각화(numpy).
- `mapping/check_lidar_bag.py` — 매핑 전 `/unilidar/cloud` 사전점검(프레임수·rate·빈프레임) + BEV PNG.
- `mapping/check_self_points.py` — 매핑 후 맵에 수집자(사람)가 남았는지 판정(§6.5).
- `mapping/pcd_denoise.py` — 매핑 후 `map.pcd` 고립 노이즈 제거 → `map_clean.pcd`(§6.6).

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
# 예: ./mapping/lio_map_bag.sh data/sj_bags/260722/bags/record-all_with-sun_1 \
#         data/sj_bags/260722/maps_selfmask/raws1_mapping
```
- `--no-preview` : 미리보기 PNG 생략.
- `--max-secs N` : bag 재생을 N초에서 중단(LIO 발산/오염 구간 제외). realtime 재생이라 벽시계≈bag 시간.
- 소요: bag 재생 시간 + 초기화 ~10초.

매핑이 끝나면 **고립 노이즈 제거**(§6.6)를 이어서 돌린다:
```bash
python3 mapping/pcd_denoise.py <출력_폴더>/map.pcd     # → map_clean.pcd
```

## 4. 산출물 (`<출력_폴더>/`)
| 파일 | 내용 |
|---|---|
| `map.pcd` | world 프레임 누적 점군(dense 3D 맵). LIO 원본, 절대 덮어쓰지 않는다 |
| `map_clean.pcd` | 위에서 고립 노이즈만 뺀 것(§6.6). **하류(BEV 라벨)는 이걸 쓴다** |
| `trajectory.tum` | ego pose 시퀀스 `t tx ty tz qx qy qz qw` (BEV의 핵심) |
| `run_info.txt` | bag·git·pose 수·map point 수 |
| `preview/*.png` | top-down/side, BEV height/density 미리보기 |

`map_clean.pcd` 는 `lio_map_bag.sh` 가 자동으로 만들지 않는다(§6.6을 따로 실행). 없으면 `map.pcd`
를 그대로 써도 되지만 고립 flyer 가 BEV 에서 허위 obstacle 셀이 될 수 있다.

### 260722 데이터셋의 현재 위치 (2026-08-10)
| 폴더 | 내용 |
|---|---|
| `data/sj_bags/260722/maps_selfmask/` | **현행.** self mask(§6.5) + drain 수정 적용 재매핑 7종 + `map_clean.pcd` |
| `data/sj_bags/260722/maps/` | 구버전(사람 잔재·뒷부분 잘림 있음). 대조용 보관. `lidaronly_mapping` 은 여기만 있다 |

하류 작업(BEV auto-label 등)은 `maps_selfmask/<name>_mapping/` 을 가리켜야 한다.

## 5. 시각화
- 헤드리스: `python3 mapping/pcd_preview.py <pcd> [out]`, `mapping/bev_grid.py <pcd> [out]`.
- 디스플레이: `pcl_viewer map.pcd`(`sudo apt install pcl-tools`), 또는 RViz2
  (`ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=true` 실시간).

## 6. 판정 / 주의
- **매핑 전 사전점검**: `python3 mapping/check_lidar_bag.py <bag_경로>` 로 `/unilidar/cloud`가
  제대로 찍혔는지(프레임수·rate·빈프레임) 먼저 확인하고 BEV PNG로 실제 구조를 눈으로 본다
  (setup.bash source 필요). 여기서 이상하면 LIO를 돌려도 맵이 안 나온다.
- **정지 vs 이동**: 정지 촬영이면 맵에 동심원 링·과밀이 나타난다(정상). 실제 BEV 데이터용
  맵은 **공간을 이동하며** 녹화해야 궤적이 생긴다. 초기 몇 초는 정지(IMU 초기화).
- **뒷부분 잘림 확인**: `run_info.txt` 의 `traj_span_s` 가 bag 길이와 비슷해야 한다. 재생은
  realtime 인데 노드가 못 따라가면 뒤처진 만큼 bag 뒷부분이 **경고 없이 통째로 누락**된다
  (수정 전 실측: 같은 bag 재실행마다 궤적 129/120/108 s, 맵 끝 x=36.3/34.2/30.2 m). 지금은
  궤적 파일이 안 커질 때까지 기다린 뒤 노드를 내린다(`DRAIN_MAX` 초, 기본 180).
- **궤적 길이는 다운샘플 후 재라.** `trajectory.tum` 은 ~3000 Hz 라 연속 pose 차분을 그냥
  합하면 지터가 누적돼 **3배 부풀려진다**(raws1: raw 116.6 m vs 2 Hz 38.0 m). 실제 이동거리는
  후자다. raw 길이는 pose 개수에 비례하므로 런 간 비교에도 쓸 수 없다.
- **런 간 편차가 있다.** 같은 bag·같은 설정이라도 재생 타이밍에 따라 결과가 달라진다
  (z 종단 드리프트 실측 −0.01~+0.67 m). 한 번의 차이로 설정 효과를 판단하지 말 것.
- `trajectory.tum` 라인 수가 0이면 pose 미복원 → bag의 `/unilidar/imu`·`/unilidar/cloud` 확인.
- `map.pcd` 미생성 → `<out>/point_lio.log` 확인(토픽·per-point time 필드).

## 6.5. self mask — 카트를 끄는 수집자 제거

`map.pcd` 는 **필터가 없는 누적 원장**이다. `laserMapping.cpp:551-563` 이 매 스캔의
`feats_down_world` 를 그대로 `+=` 할 뿐, 동적물체 제거도 free-space carving 도 없다
(정합용 ikd-Tree 맵과는 별개의 물건이다). 입력단 필터는 `blind`(구면 반경) 하나뿐이다.

그래서 **카트를 끌고 따라오는 수집자가 통째로 맵에 적립된다.** 게다가 사람은 카트 뒤
0.8~0.9 m 를 따라오는데 그 자리는 곧 조금 전 카트가 지나온 궤적 위라서, **지나간 경로가
사람 몸으로 덧칠된다.**

### 실측 근거 (260722 bag 7종, 원본 스캔)

| 방향(±10° 콘) | 중앙 거리 |
|---|---|
| 전방 | 3.5~3.9 m (열림) |
| **후방** | **0.90 m (모든 bag 동일)** |
| 좌 / 우 | 0.6~2.0 m (bag 마다 다름) |

후방 물체는 `|y| <= 0.3 m`, `x -0.65~-0.92 m`(p50 -0.80), `z <= 0.85 m` 에 갇혀 있고 그 위는
3 m 이상 열려 있다(= 사람 키). 좌우는 bag 마다 달라지는데 후방만 모든 bag 에서 같다는 것이
**몸에 고정된 물체**라는 근거다. 스캔당 약 230점.

### 마스크는 부채꼴이 아니라 박스여야 한다

260722 는 **좁은 통로**라 좌우 0.33~0.58 m 에 실제 구조물이 있다(정상이며 반드시 남겨야 함).
부채꼴(yaw 150~210°, r<1.3 m)로 자르면 yaw 150° 부근에서 벽까지 `0.5/sin(30°)=1.0 m` 라
**벽 모서리를 같이 갉아먹는다.** 벽은 `|y|>=0.35` 부터 나타나므로 축정렬 박스로 자른다.

### 정합에는 남기고 저장에서만 뺀다 (중요)

처음엔 `preprocess` 단계에서 버렸는데 — 즉 정합에서도 빼봤는데 — **수직 드리프트가 나빠졌다.**
같은 bag·같은 조건(재생 잘림 없음, 결정적)으로 재현:

| | 궤적 길이(2 Hz) | z 종단 |
|---|---|---|
| 마스크 없음 | 40.8 m | **+0.19 m** |
| 전처리에서 제거 | 41.0 m | **+0.67 m** (2회 재현) |
| **저장에서만 제거** | 40.8 m | **+0.19 m** (마스크 없음과 궤적 차이 중앙 0.09 mm) |

원인: 이 라이다는 elevation 0~90° 라 **바닥을 전혀 못 본다.** 수직을 잡아줄 게 온실 지붕뿐이라
원래 수직이 약한데, 몸에 붙어 다니는 사람의 점 궤적(튜브)이 "높이를 유지하라"는 구속으로
작동하고 있었다. 바닥이 평평하므로 그 구속은 우연히 정답이고, 빼면 약점이 드러난다.

그래서 마스크는 **`map.pcd` 누적 단계에만** 건다(`laserMapping.cpp` `publish_frame_world()` 의
`pcd_save` 블록). `feats_down_body[i]` 와 `feats_down_world[i]` 는 같은 점이라(`pointBodyToWorld`
가 인덱스 그대로 변환) body 좌표로 판정하고 world 점을 버린다. 설정은
`config/unilidar_l2.yaml` 의 `pcd_save.self_mask_*`, 기본값 `x -1.5~-0.45 / |y|<0.35 / z<1.0`, 기본 ON.

결과: **맵은 사람이 지워지고, 궤적은 마스크 없을 때와 동일**하다.

### 판정

```bash
python3 mapping/check_self_points.py <mask전_out> <mask후_out> --png
```
각 pose 의 **바디프레임**으로 맵을 되돌려 self 박스 안의 점을 직접 센다. 궤적까지의 거리로
재면 좁은 통로의 벽이 섞여 못 쓴다. 판정 대상은 **코어(`|y| < y_abs/2`)** — 통로 중앙이라
사람 말고는 있을 게 없는 자리다. raws1 전 구간 실측:

| | 코어 | 박스 전체 | map 점수 |
|---|---|---|---|
| mask off | 1334 pts/pose | 4423 | 2,215,541 |
| mask on | **508 pts/pose (-62%)** | 2918 | 2,158,279 |

박스 전체가 덜 줄어드는 건 정상이다. 맵은 전 시간의 누적이라 앞쪽에 있을 때 찍힌 벽이
나중 pose 기준으로는 박스 뒤편에 들어앉는다. 마스크는 '찍히는 순간' 박스 안이던 점만 막는다.

260722 7개 bag 전체 적용 결과는 §6.7.

`--png` 를 주면 `<map_dir>/preview/self_xsec.png` 가 생긴다. 후방 창의 점을 pose 바디프레임
y-z 단면으로 전부 겹친 그림이다(흰 선 = 마스크 박스 경계, 회색 = 센서 높이).
**박스 안만 비고 좌우 두 덩어리(통로 벽)는 그대로**여야 성공이다. 수치보다 이게 빠르다.

주의 세 가지:
- **`blind` 를 키워서 지우려 하면 안 된다.** 구면 반경이라 0.5→1.3 으로 올리면 좌우 통로
  구조물까지 전멸한다.
- 사람을 지워도 **사람이 가렸던 영역이 복원되지는 않는다**(애초에 안 찍혔다). 그 자리는
  카트가 다른 위치에서 봤기를 기대하는 수밖에 없다.
- 박스 치수는 **이 카트·이 수집자 기준**이다. 장비나 끄는 방식이 바뀌면 다시 실측해서
  yaml 을 고쳐야 한다.

## 6.6. 고립 노이즈 제거 (`pcd_denoise.py`)

Point-LIO 재실행 없이 기존 `map.pcd` 를 후처리한다. **원본은 건드리지 않고** 같은 폴더에
`map_clean.pcd` 를 만든다.

```bash
python3 mapping/pcd_denoise.py data/sj_bags/260722/maps_selfmask/*_mapping/map.pcd
python3 mapping/pcd_denoise.py <...>/map.pcd --dry-run     # 쓰지 않고 통계만
```

지우는 것은 **고립점만**이다. DBSCAN(eps=0.3 m)으로 보면 맵은 세 계층으로 갈린다 —
본체(90~99%) / 500점+ 대형 분리군집 / 고립점. **대형 분리군집은 실구조물이다**(raws3 의
24,158점 군집 = 13×42 m 천장). "최대 군집만 남기기" 류를 하면 천장이 통째로 날아가므로 금지.

척도는 k번째 최근접 이웃 거리 `d_k`. 8개 맵 전수 측정에서 `d4` 는 중앙 3.1 cm / p99.9
25~31 cm / 최대 1~20 m 의 깨끗한 이봉 분포라 기본 임계 `--dist 0.3` 이 데이터로 정당화된다.

**바닥 보호**: 마스트 라이다가 바닥을 거의 못 봐 바닥 점이 원래 희소하고, 그대로 두면 밀도
필터가 바닥부터 지운다(실측: 하위 5% 낮은 점의 제거율이 평균의 2.3배). `--protect-below`(기본
`p1`) 아래는 임계를 `--protect-factor`(기본 3.0)배로 **완화**한다 — 면제가 아니라 완화라서
낮은 위치의 진짜 flyer 는 여전히 걸린다.

주의:
- 임계값은 **점 밀도에 의존**한다. voxel 다운샘플한 클라우드에 그대로 쓰면 안 된다.
- PCD 바이너리를 직접 읽고 써서 `intensity`/normal/curvature 등 8개 필드를 전부 보존한다
  (open3d 로 읽고 쓰면 x/y/z 만 남아 intensity 가 소실된다).
- 낮은 z 꼬리가 곧 노이즈는 아니다. rawos4 의 `z<-1.0` 3,630점은 1,015점 군집을 이루는 응집
  구조라 그대로 남는다(= `z min` 이 안 변하는 게 정상). 그게 실제 지형인지 LIO 수직 드리프트
  인지는 고립도로 판별할 수 없는 별개 문제다.

260722 7종 실측 제거율 **0.046~0.103%**(rawos3 0.046% ~ raws1 0.103%). 즉 맵 모양은 그대로고
극단 flyer 만 빠진다.

## 6.7. 260722 재매핑 검증 결과 (2026-08-10)

self mask(§6.5) + drain 수정(§6) 을 적용해 7개 bag 을 전부 재매핑한 것이
`data/sj_bags/260722/maps_selfmask/`. 검증 요약:

| map | bag 길이 | `traj_span_s` | 궤적(2 Hz) | 시종점 직선 | z 종단 | 코어 pts/pose 구→신 |
|---|---|---|---|---|---|---|
| raws1  | 138.8 | 138.5 | 40.9 | 38.7 | +0.20 | 1354 → 522 (−61%) |
| raws2  | 150.0 | 149.0 | 42.2 | 40.4 | +0.43 | 943 → 213 (−77%) |
| raws3  | 169.2 | 168.8 | 42.6 | 36.9 | +0.14 | 1118 → 257 (−77%) |
| rawos1 | 178.9 | 178.6 | 41.8 | 39.8 | −1.94 | 3492 → 353 (−90%) |
| rawos2 | 194.1 | 193.7 | 44.0 | 40.7 | −0.90 | 1523 → 595 (−61%) |
| rawos3 | 165.1 | 164.7 | 41.2 | 38.7 | −0.89 | 4971 → 174 (−96%) |
| rawos4 | 161.8 | 161.3 | 41.5 | 39.1 | −1.11 | 773 → 106 (−86%) |

읽는 법:
- **잘림 없음 7/7** — `traj_span_s` 가 bag 길이와 최대 1.0 s 차. drain 수정이 전 bag 에서 동작한다.
- **궤적이 전부 40.9~44.0 m 로 수렴**한다(같은 통로를 왕복한 것과 일치). 구 `maps/` 의 rawos3 는
  길이 58.1 m 인데 시종점 직선이 6.6 m 였다(= top-down 이 방사형 별모양, LIO 가 전진 못 함).
  rawos1 도 43.5 m / 직선 31.1 m 로 눈에 띄게 휘어 있었다. **둘 다 정상 복구**됐다.
- **잔재는 사람이 아니라 벽이다.** 구 맵의 코어 x-프로파일에는 x −0.85~−0.65 m 에 국소 혹이
  있었는데(사람) 신 맵에서는 사라지고 단조 감쇠만 남는다. 벽 피크 FWHM 0.24~0.47 m 로
  이중벽/유령상도 없다 = 마스크가 구조물을 갉아먹지 않았다.

**rawos 계열의 z 경사는 알려진 잔여 문제다.** rawos 4종은 종단 z 가 −0.9~−1.9 m 로 선형에 가깝게
기운다. 궤적 기울기와 천장 기울기를 비교하면 raws1·raws2·rawos1·rawos2 는 0.3% 이내로 일치 —
즉 **맵 전체가 통째로 기운 강체 회전**이라 회전 한 번으로 정정 가능하다. raws3·rawos3·rawos4 만
0.8~1.5% 의 실제 변형이 남는다. BEV 라벨은 ego-local 크롭이고 ego pose 도 같이 기우므로 국소로
남는 건 이 **불일치분뿐**이다: 1.5% × 4 m ≈ **6 cm** 로 `z-gate`(기본 0.3 m) 한참 아래다.
전역 맵을 그대로 볼 때만 눈에 띈다.

가장 약한 케이스는 **rawos2**(코어 595, 단면 중앙에 옅은 V자가 남음 — 수집자가 약간 바깥으로
걸은 듯). 이 bag 만 문제되면 `self_y_abs` 를 0.40 으로 올려 재매핑한다.

부수 관찰: 천장 높이가 raws ~5.15 m / rawos ~3.0~3.6 m 두 그룹으로 갈린다. 온실 내 다른 구역일
가능성이 높다 — 학습 데이터를 섞을 때 알고 있어야 한다.

## 7. 문제해결
| 증상 | 조치 |
|---|---|
| 빌드 시 pcl 못 찾음 | `sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions` |
| `Failed to find match for field 'time'` | bag 점군에 per-point time 필드 없음(L2 정상 출력이면 발생 안 함) |
| 맵이 흐트러짐/드리프트 | 초기 정지 구간 확보, 급격한 이동 자제 |
