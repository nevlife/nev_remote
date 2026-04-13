# NEV Teleop Bot

차량 탑재 텔레오퍼레이션 패키지. ROS 2 센서 데이터를 Zenoh로 GCS에 전달하고, GCS 명령을 수신합니다.

## 패키지

- **nev_teleop_bot** - ROS 2 ↔ Zenoh 브릿지 + H.264/H.265 카메라 인코더
- **nev_teleop_bot_msgs** - 커스텀 ROS 2 메시지 정의

## 노드

| 노드 | 역할 |
|------|------|
| `net_bridge.py` | ROS 2 토픽 ↔ Zenoh 브릿지 (텔레메트리, 명령, E-stop, 하트비트) |
| `video_bridge_h264` | ROS 2 이미지 → H.264 인코딩 → Zenoh |
| `video_bridge_h265` | ROS 2 이미지 → H.265 인코딩 → Zenoh |

## 실행

```bash
# 빌드
colcon build --packages-select nev_teleop_bot_msgs nev_teleop_bot

# net_bridge
ros2 launch nev_teleop_bot net_bridge.launch.py

# 비디오 브릿지 (택 1)
ros2 launch nev_teleop_bot video_bridge_h264.launch.py
ros2 launch nev_teleop_bot video_bridge_h265.launch.py
```

## 인코더 설정

`config/video_params_h264.yaml` (또는 `video_params_h265.yaml`):

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `hw_accel` | `false` | `true`: NVIDIA NVENC (GPU), `false`: x264enc/x265enc (CPU) |
| `bitrate` | `1000` | 목표 비트레이트 (kbit/s) |
| `gop-size` | `60` | I-프레임 간격 |
| `max_fps` | `30.0` | 최대 프레임레이트 |

HW 가속은 GStreamer 1.20 + 드라이버 호환성에 따라 동작하지 않을 수 있음. 미동작 시 `hw_accel: false`로 설정.

## Zenoh 토픽

### 발행 (차량 → GCS)

| 토픽 | 주기 | 내용 |
|------|------|------|
| `nev/robot/mux` | 20 Hz | 모드/소스 상태 |
| `nev/robot/twist` | 20 Hz | 속도/각속도 |
| `nev/robot/hunter` | 20 Hz | 로봇 상태 (배터리, 에러) |
| `nev/robot/estop` | 20 Hz | E-stop 상태 |
| `nev/robot/network` | 20 Hz | 네트워크 상태 |
| `nev/robot/camera` | 10-30 fps | H.264/H.265 NAL (10B 헤더 + payload) |
| `nev/robot/video_stats` | 1 Hz | 인코딩 통계 (JSON) |
| `nev/robot/cpu,mem,gpu,disk,net` | 1 Hz | 시스템 리소스 |

### 구독 (GCS → 차량)

| 토픽 | 내용 |
|------|------|
| `nev/gcs/heartbeat` | GCS 하트비트 |
| `nev/gcs/teleop` | 속도/각속도 명령 |
| `nev/gcs/estop` | E-stop (RELIABLE) |
| `nev/gcs/cmd_mode` | 모드 변경 (RELIABLE) |

## 의존성

- ROS 2 Humble
- [zenoh-c](https://github.com/eclipse-zenoh/zenoh-c) 1.8.0
- GStreamer 1.20 (`x264enc`, `x265enc`, `nvh264enc`, `nvh265enc`)
