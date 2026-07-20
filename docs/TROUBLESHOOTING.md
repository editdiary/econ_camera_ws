# 문제해결 (Troubleshooting)

실기 운영 중 겪은 문제와 원인·해결법을 사례별로 정리한다. 도메인별 세부 트러블슈팅은
[`LIDAR.md`](LIDAR.md) §7, [`USAGE.md`](USAGE.md), [`CALIBRATION.md`](CALIBRATION.md)에도 있다.

---

## T1. `record_all` 실행 시 `[WARNING] Unilidar is not initialized!` 반복

### 증상
```
[unitree_lidar_ros2_node-2] [WARNING] Unilidar is not initialized!
[capture-1] [INFO] ... 워밍업 4.0s 완료 — 발행 시작. ...
[unitree_lidar_ros2_node-2] [WARNING] Unilidar is not initialized!
[unitree_lidar_ros2_node-2] [WARNING] Unilidar is not initialized!
...
```
카메라(`capture-1`)는 정상 발행하는데 LiDAR 노드만 위 경고를 계속 뱉는다.
`/unilidar/cloud`·`/unilidar/imu`가 하나도 발행되지 않는다.

### 원인
`Unilidar is not initialized!`는 SDK가 **LiDAR 초기화(핸드셰이크)를 끝내지 못했다**는 뜻이며,
이 상태의 노드는 들어온 데이터를 전부 버린다. **하드웨어·네트워크 문제가 아니라, 이미 다른
LiDAR 런치가 실행 중이라 UDP 포트(6201)와 LiDAR를 선점하고 있어서** 두 번째로 뜬 노드가
소켓을 잡지 못하는 것이 전형적 원인이다.

- LiDAR는 **한 번에 하나의 프로세스만** 점유할 수 있다.
- 예: 예전에 `ros2 launch unitree_lidar_ros2 launch.py`(단독 LiDAR 런치)를 띄운 채 종료하지
  않았고, 그 상태에서 `record_all.launch.py`를 실행 → record_all이 띄운 LiDAR 노드가 충돌.
- 이때 첫 번째(선점한) 노드는 정상 발행 중이고, 두 번째 노드만 경고를 낸다.

### 진단
```bash
# 1) LiDAR 노드가 몇 개 떠 있는지 — 2개 이상이면 충돌
pgrep -af unitree_lidar_ros2_node

# 2) 살아있는 launch 세션 확인 (record_all 외에 lidar 단독 launch가 있는지)
pgrep -af "ros2 launch"

# 3) UDP 포트 점유 프로세스 (6201 = 호스트 수신)
ss -uanp | grep -E ":6201|:6101"

# (참고) 네트워크·하드웨어는 정상인지 — 문제 아니면 아래는 모두 OK
ip -4 addr show | grep 192.168.1     # 호스트에 192.168.1.2/24
ping -c 3 192.168.1.62               # LiDAR 응답
```
- `pgrep`에 노드가 2개, 또는 `ros2 launch`가 여러 개면 **충돌 확정**.
- 6201을 점유한 PID의 부모(`ps -o ppid= -p <PID>`)를 보면 어느 런치가 잡았는지 알 수 있다.

### 해결
```bash
# 잔류 런치를 그 터미널에서 Ctrl-C. 안 되면 프로세스 정리:
pkill -f "unitree_lidar_ros2.*launch"
pkill -f unitree_lidar_ros2_node

# 정리 확인 — 아무것도 안 나오고 포트가 비어야 함
pgrep -af unitree_lidar_ros2_node          # (없음)
ss -uanp | grep -E ":6201|:6101"           # (점유 없음)

# 그다음 record_all 단독 실행
ros2 launch econ_camera_ros record_all.launch.py
```

### 예방
- LiDAR 런치는 **한 번에 하나만**. `record_all` 실행 전 위 `pgrep`으로 잔류 노드가 없는지 확인.
- `Ctrl-C` 후에도 노드가 고아(orphan)로 남을 수 있으니, 다음 실행 전 한 번 더 확인하는 습관.
