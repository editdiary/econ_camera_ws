# 서버 후처리 환경 (Docker)

수집이 끝난 데이터를 **서버 컴퓨터에서 가공**하기 위한 환경. 촬영·수집은 대상이 아니다
(그건 Jetson에서 호스트 네이티브 ROS2로 한다 — [USAGE](USAGE.md)).

## 1. 왜 Docker 하나로 끝나는가

- **후처리 코드는 colcon 빌드가 필요 없다.** `bag_extract.py`·`check_lidar_bag.py`·
  `accumulate_cloud.py`는 `rosbag2_py`/`rclpy`를 **함수 안에서 지연 import**하는 평범한
  파이썬 스크립트라, ROS 런타임만 있으면 `python3 <경로>`로 그대로 돌아간다.
  colcon이 필요한 건 `point_lio`(매핑)와 `unitree_lidar_ros2`(실시간 수집)뿐이고 둘 다 범위 밖.
- 그래서 **ROS 런타임 + 파이썬 후처리 스택을 한 이미지에** 넣는다. base가
  `ros:humble-ros-base-jammy`라 Ubuntu 22.04 / Python 3.10이 딸려오는데, 이는 원래 Jetson
  환경과 같은 조합이라 검증 부담이 가장 적다.
- 공용 서버라 호스트를 오염시키지 않는다. 호스트에 필요한 건 Docker뿐.

## 2. 준비 (최초 1회)

```bash
cd ~/Desktop/econ_camera_ws     # 서버 경로: /data/home/dhlee/Desktop/econ_camera_ws
./docker/build.sh               # sudo docker build → econ-proc:humble
```

빌드 컨텍스트는 `docker/`만 쓴다(저장소 루트를 컨텍스트로 잡으면 `data/` 66GB를 통째로
데몬에 전송하게 된다). 이미지 이름을 바꾸려면 `ECON_IMAGE=... ./docker/build.sh`.

> **sudo**: 이 서버에서 `dhlee`는 docker 그룹에 없고 `sudo docker`로만 실행한다.
> `build.sh`/`run.sh`가 알아서 `sudo`를 붙이므로 비밀번호만 입력하면 된다.
> 연속 작업 전에 `sudo -v` 한 번 해두면 이후 프롬프트가 줄어든다.

## 3. 실행

```bash
./docker/run.sh                                     # 대화형 셸
./docker/run.sh python3 mapping/pcd_preview.py data/sj_bags/260722/maps_selfmask/raws1_mapping/map.pcd
./docker/run.sh bash -c 'cd calibration/bev_autolabel && python3 generate_slab.py ...'
```

`run.sh`가 하는 일은 셋뿐이다:

| 옵션 | 이유 |
|---|---|
| `--user $(id -u):$(id -g)` | **필수.** `sudo docker`라 이게 없으면 `data/` 산출물이 전부 root 소유로 떨어져 호스트에서 손대지 못한다 |
| `-e HOME=/tmp` | 컨테이너에 그 uid의 계정이 없어 HOME이 비는데, open3d/pip 등이 캐시를 쓰려다 실패한다 |
| `-v <ws>:/ws -w /ws` | 저장소 루트 하나만 마운트. `data/`가 저장소 안에 있어 bag·extracted·dataset이 전부 이 하나로 커버된다 |

ros 계열 base 이미지의 ENTRYPOINT(`/ros_entrypoint.sh`)가 `/opt/ros/humble/setup.bash`를
소스한 뒤 명령을 exec 하므로, 컨테이너 안에서 따로 source 할 필요가 없다.

**경로는 항상 저장소 루트 기준 상대경로**로 쓴다. 호스트의 `~/Desktop/econ_camera_ws`가
컨테이너의 `/ws`이므로, 호스트 절대경로(`/data/home/...`)를 넘기면 컨테이너 안에서 찾지 못한다.

## 4. 이미지 구성과 버전 핀

`docker/Dockerfile` 참조. 핵심만:

| 패키지 | 버전 | 비고 |
|---|---|---|
| `ros-humble-rosbag2-storage-mcap` | apt | **mcap bag 읽기.** ros-base 이미지에 없어 따로 설치 |
| `numpy` | `1.26.4` | **2.x 금지** (아래) |
| `scipy` | `1.13.1` | `chain.py`의 Rotation/Slerp |
| `opencv-python-headless` | `4.10.0.84` | 서버가 헤드리스라 GUI 심볼 없는 빌드 |
| `open3d` | `0.18.0` | `map.pcd` 읽기 |
| `PyYAML` / `Pillow` / `mcap` / `pytest` | 고정 | calib.yaml, 팔레트 PNG, bag 타임스탬프 점검, 테스트 |

**numpy를 1.x로 고정하는 이유** (Jetson에서의 이유와는 다르다):

1. Humble의 `rclpy`/`rosbag2_py`가 numpy 1.x에 대해 빌드돼 있다. 2.x를 얹으면 bag 읽기가 깨진다.
2. `open3d 0.18`은 numpy<2 전용이다.

pip의 numpy 1.26은 `/usr/local/lib/python3.10/dist-packages`에 깔려 시스템 numpy(1.21)보다
우선하고, numpy 1.x 내부에서는 ABI가 호환되므로 ROS 확장 모듈과 함께 안전하게 동작한다.
`libgl1`/`libgomp1`/`libusb-1.0-0`은 open3d 런타임 의존이라 헤드리스로 파일 IO만 해도 필요하다.

## 5. 이 환경에서 되는 것 / 안 되는 것

**된다**

```bash
# BEV auto-label(현행 = 슬래브 라벨) — 생성 → 검수 시트 → CVAT용 수집
./docker/run.sh bash -c 'cd calibration/bev_autolabel && python3 generate_slab.py \
    --map-dir ../../data/sj_bags/260722/maps_selfmask/raws1_mapping \
    --extract-dir ../../data/extracted/raws1 \
    --calib ../../data/calib_260723/calib.yaml \
    --orient ../../data/calib_260723/orientation.json \
    --self-mask-dir ../../data/calib_260723/self_mask \
    --out ../../data/bev/slab/raws1'
./docker/run.sh python3 calibration/bev_autolabel/slab_sheet.py data/bev/slab/raws1
./docker/run.sh python3 calibration/bev_autolabel/gather_slab.py \
    --dataset data/bev/slab/raws1 --out data/bev/annotations

# 구판 경로(verify_labels/generate/gather_annotations)도 그대로 돈다 — 의존성이 같다.
# docker/smoke_test.sh 가 검증하는 건 아직 이 구판 경로다(BEV_AUTOLABEL 부록 A).

# calib.yaml 시각 검증
./docker/run.sh python3 calibration/verify/verify_undistort.py \
    --images data/calib_260723/extracted --frames "0,800"

# 맵 미리보기 / BEV 격자 (numpy+zlib 자작 PNG라 의존성 0)
./docker/run.sh python3 mapping/pcd_preview.py data/sj_bags/260722/maps_selfmask/raws1_mapping/map.pcd
./docker/run.sh python3 mapping/bev_grid.py    data/sj_bags/260722/maps_selfmask/raws1_mapping/map.pcd

# map.pcd 고립 노이즈 제거 → map_clean.pcd (numpy+scipy cKDTree만 씀)
./docker/run.sh python3 mapping/pcd_denoise.py data/sj_bags/260722/maps_selfmask/raws1_mapping/map.pcd

# bag 관련 (mcap 플러그인 포함)
./docker/run.sh python3 src/econ_camera_ros/econ_camera_ros/bag_extract.py <bag> -o data/extracted/<name>
./docker/run.sh python3 mapping/check_lidar_bag.py <bag> --out <out_dir>
./docker/run.sh python3 tools/check_recording.py <bag>

# 순수 로직 테스트
./docker/run.sh bash -c 'cd src/econ_camera_ros && python3 -m pytest test/ -q'
./docker/run.sh bash -c 'cd calibration/bev_autolabel && python3 -m pytest . -q'
```

> `web_monitor_node.py`가 쓰는 `econ_cam.stats`(선행 프로젝트, 촬영측 의존)는 서버에 없다.
> 그래서 그 import를 **`_status()` 안으로 옮겨 지연 import**로 바꿨다 — 저장소의 다른
> ROS 의존(`bag_extract`, `cloud_io`)과 같은 방식이다. 덕분에 `index_html` 같은 순수 로직은
> econ_cam 없이도 테스트되고, 촬영 시 동작은 그대로다.
>
> (처음엔 `pytest.importorskip`으로 해당 파일만 건너뛰려 했으나, ROS의 `launch_testing`
> pytest 플러그인이 수집 단계에서 모듈을 미리 import 하는 탓에 **`test/` 디렉터리 전체가
> 통째로 skip**되어 나머지 테스트까지 안 돌았다. 지연 import가 올바른 해법이다.)

## 5.1 환경 검증

`./docker/smoke_test.sh`가 실데이터로 위 경로를 한 번씩 돌려 `[PASS]`/`[FAIL]`을 찍는다.
Dockerfile을 고쳤거나 환경이 의심스러울 때 이걸 먼저 돌린다.

```bash
./docker/build.sh      2>&1 | tee /tmp/build.log
./docker/smoke_test.sh 2>&1 | tee /tmp/smoke.log
```

산출물은 `data/_archive/docker_smoke/`에 떨어진다(검증용이라 지워도 무방).
마지막 항목이 **산출물 소유권**을 확인한다 — `dhlee`가 아니라 `root`로 찍히면
`run.sh`를 우회해 실행한 것이다(§6).

**안 된다 (의도적 제외)**

| 항목 | 이유 / 대안 |
|---|---|
| `cam_lidar/pick_correspondences.py`, `mapping/pcd_view.py` | 인터랙티브 GUI 창 필요. 서버는 헤드리스이고 opencv도 `-headless` 빌드다. 대응점 클릭이 다시 필요하면 디스플레이 있는 장비에서 하거나 이미지에 X11/VNC를 따로 구성해야 한다 |
| Point-LIO 재매핑 (`mapping/lio_map_bag.sh`) | colcon 빌드 + PCL/Eigen 필요. 현재 `data/sj_bags/260722/maps_selfmask/`에 7개 bag의 `map.pcd`·`map_clean.pcd`+`trajectory.tum`이 이미 있어 불필요(매핑 후처리인 `pcd_denoise.py`는 이 이미지에서 돈다). 필요해지면 **이 Dockerfile에 apt(PCL) + `colcon build` 레이어를 덧붙이면 된다**(이미지 교체 아님) |
| 카메라 촬영·LiDAR 수집 | 하드웨어가 있는 Jetson 전용 |

## 6. 문제해결

**`permission denied ... /var/run/docker.sock`** — `sudo` 없이 docker를 부른 경우.
`build.sh`/`run.sh`를 쓰면 자동으로 붙는다.

**산출물이 root 소유로 생김** — `run.sh`를 거치지 않고 직접 `sudo docker run` 한 경우.
`--user $(id -u):$(id -g)`를 빠뜨리면 그렇게 된다. 이미 생긴 파일은
`sudo chown -R $(id -u):$(id -g) <경로>`로 되돌린다.

**`ValueError: numpy.dtype size changed`** — 컨테이너 안에서 numpy를 2.x로 올린 경우.
Dockerfile의 핀을 지키고, 컨테이너 안에서 `pip install -U` 하지 않는다.

**`No module named 'rosbag2_py'`** — `run.sh`를 거치지 않아 ENTRYPOINT의 ROS 소싱이 빠진 경우
(예: `docker run --entrypoint bash`). `run.sh`를 쓰거나 직접
`source /opt/ros/humble/setup.bash` 한다.

**파일을 못 찾음** — 호스트 절대경로를 넘겼을 가능성. §3의 상대경로 규칙을 따른다.
