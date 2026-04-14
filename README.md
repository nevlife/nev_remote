# NEV Teleop Bot

차량 탑재 텔레오퍼레이션 패키지. ROS 2 센서 데이터를 Zenoh로 서버에 전달하고, 서버 명령을 수신하여 차량을 제어합니다.

## 패키지

- **nev_teleop_bot** - ROS 2 ↔ Zenoh 브릿지 + H.264/H.265 영상 인코더
- **nev_teleop_bot_msgs** - 커스텀 ROS 2 메시지 (EStopStatus, CmdMode, MuxStatus)

## 노드

| 노드 | 언어 | 역할 |
|------|------|------|
| `net_bridge.py` | Python | ROS 2 토픽 ↔ Zenoh 브릿지 (텔레메트리, 명령, E-Stop, 핑퐁) |
| `video_bridge_h264` | C++ | ROS 2 Image → H.264 인코딩 → Zenoh |
| `video_bridge_h265` | C++ | ROS 2 Image → H.265 인코딩 → Zenoh |

## 실행

```bash
colcon build --packages-select nev_teleop_bot_msgs nev_teleop_bot

ros2 launch nev_teleop_bot net_bridge.launch.py
ros2 launch nev_teleop_bot video_bridge_h264.launch.py   # 또는 video_bridge_h265
```

## 주요 파라미터

### net_bridge

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `telemetry_locator` | `tcp/127.0.0.1:7447` | Zenoh 서버 엔드포인트 |
| `vehicle_id` | `0` | 차량 식별자 |
| `heartbeat_timeout` | `2.0` | server_ping 미수신 E-Stop 타임아웃 (초) |
| `control_timeout` | `1.0` | remote 모드 제어 미수신 E-Stop 타임아웃 (초) |
| `wheelbase` | `0.650` | Ackermann 조향 변환 휠베이스 (m) |

### video_bridge

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `hw_accel` | `true` | NVIDIA NVENC (GPU) / x264/x265 (CPU) |
| `bitrate` | `1000` | 목표 비트레이트 (kbit/s) |
| `gop-size` | `60` | I-프레임 간격 |
| `max_fps` | `30.0` | 최대 프레임레이트 |

## Zenoh 토픽

### 발행 (Bot → Server) `nev/robot/{id}/...`

| 토픽 | 주기 | 내용 |
|------|------|------|
| `mux` | 20Hz | 모드, 소스 선택, remote/nav/teleop/final 활성 상태 |
| `twist` | 20Hz | nav/teleop/final 속도 (linear_x, angular_z) |
| `estop` | 20Hz | is_estop, bridge_flag(0~4), mux_flag |
| `cpu` | 1Hz | cpu_usage, cpu_temp, cpu_load |
| `mem` | 1Hz | ram_total, ram_used (MB) |
| `gpu` | 1Hz | GPU 배열 (usage, mem, temp, power) |
| `disk` | 1Hz | 파티션 배열 (mountpoint, total, used, percent) |
| `net` | 1Hz | 인터페이스 배열 (name, is_up, speed, in/out bps) |
| `camera` | 10-30fps | Binary: vehicle_ts(8B) + encode_ms(4B) + NAL |
| `video_stats` | 1Hz | codec, bw_mbps, fps, drop, enc_avg_ms 등 |
| `vehicle/{name}` | vehicle_rate 또는 resource_rate | YAML 설정 동적 토픽 (아래 참고) |
| `server_pong` | 즉시 | server_ping 에코 `{ts}` |
| `bot_pong` | 즉시 | bot_ping 에코 `{ts}` |

### 구독 (Server → Bot) `nev/gcs/{id}/...`

| 토픽 | 내용 |
|------|------|
| `server_ping` | `{ts}` — heartbeat 겸 RTT |
| `bot_ping` | `{ts}` — cli↔bot RTT (서버 경유) |
| `teleop` | `{linear_x, steer_angle}` → 봇에서 Ackermann 변환 |
| `estop` | `{active}` (RELIABLE) |
| `cmd_mode` | `{mode}` (RELIABLE) |

## 차량별 가변 토픽 추가

`config/teleop_topics.yaml`에 차량 고유 ROS 토픽을 추가하면 자동으로 Zenoh 브릿지됩니다.
`nev/robot/{vehicle_id}/vehicle/{name}` 으로 발행됩니다.

```yaml
teleop_topics:
  - topic_name: /hunter_status              # 구독할 ROS 토픽
    msg_type: hunter_msgs/msg/HunterStatus  # ROS 메시지 타입
    name: hunter_status                     # (선택) Zenoh subtopic 이름, 미지정 시 클래스명 snake_case
    rate: vehicle                           # (선택) "vehicle" (기본, vehicle_rate Hz) | "resource" (resource_rate Hz)
    ros_qos:                                # (선택) ROS 구독 QoS
      reliability: reliable                 # "reliable" (기본) | "best_effort"
    zenoh_qos:                              # (선택) Zenoh 발행 QoS
      reliability: best_effort              # "reliable" | "best_effort" (기본)
      congestion: drop                      # "drop" (기본) | "block"
      priority: data                        # real_time | interactive_high | data (기본) | ...
```

### 발행 주기

토픽별 개별 Hz 지정은 불가. `rate` 필드로 두 그룹 중 택1:

| rate 값 | 주기 | 파라미터 |
|---------|------|---------|
| `"vehicle"` (기본) | 20Hz | `vehicle_rate` |
| `"resource"` | 1Hz | `resource_rate` |

### 예시: 다른 차량에 적용

```yaml
teleop_topics:
  # Scout Mini
  - topic_name: /scout_status
    msg_type: scout_msgs/msg/ScoutStatus

  # 커스텀 센서
  - topic_name: /battery_state
    msg_type: sensor_msgs/msg/BatteryState
    name: battery
    rate: resource   # 1Hz로 충분
```

`package.xml`에 해당 메시지 패키지 의존성 추가 필요 (예: `<depend>scout_msgs</depend>`).

## 판단/계산 영역

- **E-Stop 판단**: server_ping 타임아웃(2초), 제어 타임아웃(1초), 서버 E-Stop 명령
- **Ackermann 변환**: steer_angle → angular_z (`lx * tan(steer) / wheelbase`)
- **영상 인코딩**: GStreamer 파이프라인, 프레임 rate limiting, 인코딩 통계
- **텔레메트리 수집**: ROS 토픽 → JSON 직렬화

## 의존성

- ROS 2 Humble
- [zenoh-c](https://github.com/eclipse-zenoh/zenoh-c) 1.8.0
- GStreamer 1.20 (`x264enc`, `x265enc`, `nvh264enc`, `nvh265enc`)
