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
