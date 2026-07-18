# 데이터 수집 시스템 구성 & bag 녹화 주의사항 (Camera + LiDAR → BEV)

- 작성일: 2026-07-18
- 상태: 설계 노트 (카메라 수집은 구현 완료, LiDAR 통합은 하드웨어 도착 후)
- 목적: bag으로 topic을 저장할 때 주의할 점과, 현재까지 확정한 데이터 수집 시스템 구성을 남긴다.
- 관련 문서: `2026-07-16-econ-camera-ros2-capture-design.md` (카메라 수집 설계)

---

## 1. 최종 목표

사족보행 로봇에 탑재한 **e-con AR0234 4-camera** + **Unitree 4D LiDAR L2(IMU 내장)** 로
주행 중 데이터를 **단일 ROS2 bag(mcap)** 으로 수집하고, 이후 **BEV(Bird's-Eye-View) 학습
데이터**로 가공한다.

- 플랫폼: **사족보행 로봇(이동체)** — 보행 진동으로 자세가 빠르게 흔들림 → 시각 동기·모션보정 민감.
- 카메라: 4대 하드웨어 동기(`frame_sync`), `1280x720@30`, HW JPEG.
- LiDAR: Unitree L2 (4D LiDAR + IMU 내장). **도착 예정, 본 구성에 통합 대상**.
- 산출물: 카메라 4뷰 + LiDAR 점군(+IMU)을 시각 정렬해 BEV로 투영 가능한 데이터셋.

---

## 2. 현재 구현된 카메라 수집 구성 (완료)

### 발행 토픽 — 이미지 4개가 전부
`capture` 노드가 발행하는 토픽은 이것뿐이다(코드상 `create_publisher` 단일 지점).

| 토픽 | 타입 | 내용 | QoS |
|---|---|---|---|
| `/camera0/image_raw/compressed` | `sensor_msgs/CompressedImage` | HW JPEG | Reliable, KeepLast(10) |
| `/camera1/image_raw/compressed` | 〃 | 〃 | 〃 |
| `/camera2/image_raw/compressed` | 〃 | 〃 | 〃 |
| `/camera3/image_raw/compressed` | 〃 | 〃 | 〃 |

- rclpy 기본 토픽 `/rosout`, `/parameter_events`는 자동 광고되지만 **record 대상이 아니라 bag에 안 들어간다.**
- `monitor`(웹) 노드는 **구독 전용(발행 0)**, 녹화와 완전 격리. 안 켜면 비용 0.

### bag 저장 방식
`record.launch.py`가 **명시된 토픽만** 기록한다(`-a` 전체기록 아님):
```
ros2 bag record -s mcap  /camera0/image_raw/compressed ... /camera3/image_raw/compressed
```
→ bag(mcap) 안에는 **카메라 4대 JPEG 스트림만** 들어간다. 확장은 이 토픽 리스트에 추가하는 방식.

### 타임스탬프
각 프레임 `header.stamp` = 파이프라인 공유클럭 PTS를 **첫 프레임 ROS 시각(시스템 시간)에 앵커링**한
값. 카메라 4대 간 stamp 직접 비교 가능(실측 sub-ms 동기). LiDAR와의 정합도 이 stamp 기준.

### 워밍업
`capture` 노드는 시작 시 `warmup_s`(기본 4.0s) 동안 프레임을 폐기하며 4대 정상 동작을 확인한 뒤,
같은 주기의 4대가 모인 첫 사이클부터 발행을 시작한다 → 시작 프레임 수 불일치 제거.

### 추출
`bag_extract` — bag에서 카메라 4대를 동기 세트로 묶어 `frame_NNNNNN/cam{0..3}.jpg` + `sets.csv`로
추출. 기본 동기 허용오차 1ms(frame_sync 실측 sub-ms 반영).

---

## 3. LiDAR 통합 시 추가 구성 (도착 후)

Unitree L2는 **4D LiDAR + IMU 내장** 구성이라, 드라이버에서 점군과 IMU를 함께 받는다.

**bag에 추가로 담을 토픽:**
```
/lidar/points   (sensor_msgs/PointCloud2, 점별 타임스탬프 포함)
/lidar/imu      (sensor_msgs/Imu, 고레이트)
```
통합 방식: 별도 LiDAR 드라이버 노드 + 통합 런치 하나가 `capture` + LiDAR + **단일 bag record**(카메라
토픽 + LiDAR 토픽 합침)를 함께 기동. record가 명시 토픽 리스트를 받으므로 리스트 확장만으로 끝난다.

---

## 4. 핵심 원칙 — 정적(캘리브) vs 동적(ego-motion)

수집 시 무엇을 bag에 남기고 무엇을 사후에 처리할지는 **데이터가 정적인가 동적인가**로 갈린다.

| 데이터 | 성격 | 언제 확보하나 | bag에 넣나 |
|---|---|---|---|
| intrinsics (K, D) | 정적(고정 리그면 불변) | **아무 때나** — 촬영 전/후 무관 | 불필요 → YAML |
| 카메라 간 extrinsic | 정적 | 아무 때나 | 불필요 → YAML |
| cam ↔ LiDAR extrinsic | 정적 | 아무 때나(리그 고정 후 1회) | 불필요 → YAML |
| IMU ↔ LiDAR extrinsic | 정적 | Unitree 제공값 사용 | 불필요 → YAML |
| **ego-pose / 궤적** | **동적(주행마다 다름)** | **주행 그 순간만** | pointcloud+IMU로 사후 복원 |
| point cloud, IMU | 동적(원시 스트림) | **주행 그 순간만** | **반드시 bag에 기록** |

**비대칭의 핵심:**
- **캘리브레이션은 정적 상수** → 하드웨어를 고정하면 값이 안 바뀌므로, 촬영을 다 끝낸 뒤 같은
  환경에서 계산해도 무손실. **topic으로 bag에 넣을 필요 없다.** `calib.yaml` 사이드카로 충분.
- **동적 데이터(점군·IMU)는 그 순간에 기록 안 하면 영영 복원 불가.** 애매하면 무조건 남긴다.

### ego-pose는 저장이 아니라 *복원*한다
주행 궤적(ego-pose)은 센서가 직접 뱉지 않는다. **pointcloud + IMU** 를 **LIO(LiDAR-Inertial
Odometry, 예: FAST-LIO / Point-LIO)** 에 오프라인으로 돌려 궤적과 deskew된 점군을 만든다. Unitree
L2(LiDAR+IMU)는 이 조합을 겨냥한 구성이라, **점군+IMU만 남기면 ego-motion은 사후 완결 복원 가능**.
별도 odometry 소스(다리·휠) 불필요.

---

## 5. bag 녹화 시 주의사항 (수집 당시 반드시 챙길 것)

### (1) ⭐ 카메라 ↔ LiDAR 시각 동기 도메인 — 가장 중요
카메라 리그와 L2는 **별개 장치라 하드웨어 동기가 없다.** 크로스모달 정합은 소프트웨어 타임스탬프
매칭에 의존하고, 사족보행 진동으로 자세가 빨리 변해 시각 오차가 BEV 융합 품질에 바로 드러난다.
- **확인**: Unitree 드라이버 `header.stamp`가 카메라와 **같은 시계(ROS 시스템 시간)** 인가?
  센서 자체 클럭/수신시각이면 카메라(PTS→시스템시간 앵커)와 도메인이 어긋나 매칭이 틀어진다.
- 필요 시 드라이버의 timestamp 모드를 host/ROS 시간으로 맞춘다.

### (2) 점군 점별 타임스탬프
`/lidar/points`(PointCloud2)에 **점별 타임스탬프 필드**가 있어야 주행 중 deskew(모션보정)가 된다.
드라이버 출력에 포함되는지 확인.

### (3) IMU 레이트
deskew/LIO는 보통 **≥200Hz** IMU를 선호. L2 IMU 발행 레이트 확인.

### (4) QoS 이종 혼재
카메라는 Reliable, LiDAR PointCloud2는 흔히 BestEffort(SensorData QoS). rosbag2가 토픽별 QoS를
자동 감지·기록하므로 섞여도 문제없다.

### (5) 녹화 대상은 명시 토픽만
`/rosout`, `/parameter_events` 등은 BEV에 무관 → 지금처럼 **필요한 토픽만 명시 기록**(용량·정리 이점).

### (6) 워밍업 후 기록
카메라 워밍업(4s) 동안은 발행이 없어 bag 시작이 깔끔하다. LiDAR도 안정화 시간을 두고 시작 프레임의
품질을 확인.

---

## 6. 사이드카 캘리브레이션 (`calib.yaml`, 정적 — 촬영 후 계산 OK)

```
카메라 4대 intrinsics (K, D)          # 선행 프로젝트 Multi-Cam_module_test 도구 재사용 가능
카메라 간 extrinsics                   # 〃
cam ↔ LiDAR extrinsic  ← 새 작업       # BEV 정확도를 좌우하는 크로스모달 캘리브(타깃/targetless)
IMU ↔ LiDAR extrinsic                  # Unitree 제공값 사용
```
- **새로 해야 하는 것은 cam↔LiDAR extrinsic 하나.** 나머지는 재사용/제공값.
- 리그를 완전히 고정한 뒤 1회 계산하면 되고, dataset 옆에 YAML로 둔다.

---

## 7. 최종 데이터 흐름 (수집 → BEV)

```
[수집]  통합 런치 → 단일 bag(mcap)
        /camera0..3/image_raw/compressed  (하드웨어 동기, sub-ms)
        /lidar/points                      (점별 stamp 포함)
        /lidar/imu                         (고레이트)
[가공]  1) 카메라 4대끼리 동기 세트로 묶기        (bag_extract 방식, 1ms 허용)
        2) 타임스탬프로 최근접 LiDAR 프레임 매칭   (크로스모달 정합)
        3) (temporal 필요 시) 점군+IMU로 LIO → pose/deskew 복원
        4) calib.yaml의 cam↔LiDAR extrinsic으로 BEV 평면 투영
```

---

## 8. L2 도착 시 확인 체크리스트

- [ ] 드라이버 `header.stamp` 시계 도메인 = 카메라와 동일(ROS 시스템 시간)인가
- [ ] `/lidar/points`에 점별 타임스탬프 필드 존재
- [ ] IMU 발행 레이트(≥200Hz 권장)
- [ ] IMU↔LiDAR extrinsic(Unitree 제공값) 위치 확인
- [ ] 통합 런치에서 카메라+LiDAR 토픽을 단일 bag으로 기록
- [ ] cam↔LiDAR extrinsic 캘리브 절차 확정(리그 고정 후 1회)

---

## 결론

- **지금 bag = 카메라 4대 JPEG 스트림만.** 구조상 LiDAR 확장은 토픽 리스트 추가 + 통합 런치 수준.
- **캘리브레이션은 YAML 사이드카로 충분**(정적 상수, 사후 계산 무손실). topic 불필요.
- **이동체라서 동적 데이터(점군·IMU)는 반드시 주행 중 bag에 기록** — ego-pose는 이걸로 사후 복원.
- **남는 숙제 두 개**: ① 카메라↔LiDAR 시각 동기 도메인 일치, ② cam↔LiDAR extrinsic 캘리브.
