# nev_remote

NEV 차량과 원격 서버 간 UDP 통신을 담당하는 ROS2 패키지.

## 패키지 구성

```
nev_remote/
├── nev_remote/               # 메인 ROS2 패키지
│   ├── scripts/
│   │   └── net_bridge.py     # 네트워크 브리지 노드
│   ├── config/
│   │   └── net_bridge_params.yaml
│   ├── launch/
│   │   └── net_bridge.launch.py
│   └── test/
│       └── test_server.py    # 개발용 테스트 서버
└── nev_remote_msgs/          # 커스텀 메시지 정의 패키지
    └── msg/
        ├── CmdMode.msg
        ├── EStopStatus.msg
        ├── MuxStatus.msg
        ├── NetworkStatus.msg
        └── SystemResources.msg
```

## 의존성

- `rclpy`, `geometry_msgs`, `std_msgs`
- `hunter_msgs` (hunter_ros2 패키지)
- `system_monitor_msgs` (system_monitor 패키지)

## 빌드 및 실행

```bash
cd ~/dev/ros2_ws
colcon build --packages-select nev_remote_msgs nev_remote
source install/setup.bash

ros2 launch nev_remote net_bridge.launch.py
```

## 파라미터 (`config/net_bridge_params.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `server_ip` | `127.0.0.1` | 원격 서버 IP |
| `server_port` | `5000` | 원격 서버 UDP 포트 |
| `listen_port` | `5000` | 로컬 수신 포트 |
| `heartbeat_timeout` | `2.0` s | 하트비트 타임아웃 |
| `control_timeout` | `1.0` s | 원격 제어 타임아웃 (REMOTE 모드 한정) |
| `vehicle_rate` | `20.0` Hz | 차량 상태 패킷 전송 주기 |
| `resource_rate` | `1.0` Hz | 시스템 리소스 패킷 전송 주기 |

## 토픽

### 구독

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/vehicle/mux_status` | `nev_remote_msgs/MuxStatus` | Mux 노드 상태 |
| `/vehicle/estop_status` | `nev_remote_msgs/EStopStatus` | 통합 E-Stop 상태 |
| `/cmd_vel` | `geometry_msgs/Twist` | 자율주행 속도 명령 |
| `/remote/teleop_cmd` | `geometry_msgs/Twist` | 원격 조종 속도 에코 (자신이 발행한 토픽) |
| `/final_cmd` | `geometry_msgs/Twist` | 최종 선택된 속도 명령 |
| `/hunter_status` | `hunter_msgs/HunterStatus` | 차량 상태 |
| `/system_monitor/cpu` | `system_monitor_msgs/CpuMetrics` | CPU 메트릭 |
| `/system_monitor/memory` | `system_monitor_msgs/MemoryMetrics` | 메모리 메트릭 |
| `/system_monitor/disk` | `system_monitor_msgs/DiskMetrics` | 디스크 I/O 메트릭 |
| `/system_monitor/network` | `system_monitor_msgs/NetworkMetrics` | 네트워크 인터페이스 메트릭 |
| `/system_monitor/gpu` | `system_monitor_msgs/GpuMetrics` | GPU 메트릭 |

### 발행

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/remote/teleop_cmd` | `geometry_msgs/Twist` | 서버로부터 수신한 원격 조종 명령 |
| `/remote/estop_status` | `nev_remote_msgs/EStopStatus` | 브리지 E-Stop 상태 |
| `/remote/cmd_mode` | `nev_remote_msgs/CmdMode` | 제어 모드 변경 요청 |
| `/remote/network_status` | `nev_remote_msgs/NetworkStatus` | 네트워크 연결 상태 (모니터링용) |

## UDP 프로토콜

모든 패킷은 little-endian 바이너리 포맷. 앞 2바이트가 패킷 헤더.

### 수신 (서버 → 차량)

| 헤더 | 포맷 | 내용 |
|---|---|---|
| `HB` | `<2s d H` | 하트비트 (timestamp, seq) |
| `TC` | `<2s f f H` | 원격 조종 (linear_x, angular_z, seq) |
| `ES` | `<2s B H` | E-Stop 명령 (0=해제, 1=활성, seq) |
| `CM` | `<2s b H` | 제어 모드 변경 (mode, seq) |

### 송신 (차량 → 서버)

**차량 상태 (`vehicle_rate` Hz)**

| 헤더 | 포맷 | 내용 |
|---|---|---|
| `MS` | `<2s b b b b b b B H` | Mux 상태 (mode, cmd_source, remote_status, nav/teleop/final_active) |
| `TV` | `<2s f f f f f f H` | nav/teleop/final Twist 값 (linear_x, angular_z 각 3쌍) |
| `NS` | `<2s B b f f H` | 네트워크 연결 상태 (connected, status_code, rtt_ms, bandwidth) |
| `HS` | `<2s d d B B H d H` | Hunter 차량 상태 (속도, 조향각, 상태, 제어모드, 에러코드, 배터리) |
| `EP` | `<2s B b b H` | E-Stop 상태 (is_estop, bridge_flag, mux_flag) |
| `RE` | `<2s B H` | 원격 제어 허용 여부 (remote_status) |

**시스템 리소스 (`resource_rate` Hz)**

| 헤더 | 포맷 | 내용 |
|---|---|---|
| `CR` | `<2s f f f f f f q H` | CPU (사용률, 주파수, 온도, load avg 1/5/15m, ctx_switches) |
| `MR` | `<2s q q q q f H` | 메모리 (total/available/used/free bytes, percent) |
| `DI` | `<2s q q q q q q q H` | 디스크 I/O 요약 (read/write count, bytes, time, busy_time) |
| `DP` | `<2s B 32s q q q f B H` | 파티션별 사용량 (인덱스, mountpoint, total/used/free bytes, percent, accessible) |
| `NM` | `<2s i i i H` | 네트워크 인터페이스 수 요약 (total, active, down) |
| `NF` | `<2s B 16s B i i d d q q q q q q q q H` | 인터페이스별 상세 (인덱스, name, is_up, mtu, speed, rx/tx rate, bytes, packets, errors, drops) |
| `GR` | `<2s i f f f f f H` | GPU별 상태 (index, 사용률, memory_used/total_mb, 온도, 전력) |

## Bridge Flag

`net_bridge` 내부 상태 플래그. 값이 0이 아니면 E-Stop 발행.

| 값 | 의미 |
|---|---|
| `0` | 정상 |
| `1` | 서버 E-Stop 명령 수신 |
| `2` | 소켓 오류 |
| `3` | 하트비트 타임아웃 |
| `4` | REMOTE 모드에서 제어 타임아웃 |

## 제어 모드 (`CmdMode.msg`)

| 값 | 의미 |
|---|---|
| `-1` | Idle |
| `0` | 컨트롤러 활성, 제어 없음 (CTRL_ON) |
| `1` | cmd_vel 주행 (자율주행, NAV) |
| `2` | 원격 주행 (REMOTE) |

## 테스트 서버

로컬 개발 및 디버깅용 UDP 서버.

```bash
python3 install/nev_remote/lib/nev_remote/test_server.py \
  --vehicle-ip 127.0.0.1 --vehicle-port 5000 --listen-port 5001
```

| 키 | 동작 |
|---|---|
| `w` / `s` | 전진 / 후진 속도 ±0.1 (최대 ±1.5) |
| `a` / `d` | 좌 / 우 회전 ±0.1 (최대 ±2.0) |
| `space` | 정지 (linear/angular 0) |
| `e` | E-Stop 토글 |
| `0` / `1` / `2` | 모드 설정 (CTRL_ON / NAV / REMOTE) |
| `i` | Idle 모드 (-1) |
| `q` | 종료 |
