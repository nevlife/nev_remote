# NEV Teleop Bot

NEV 차량 탑재 텔레오퍼레이션 패키지. ROS 2 센서 데이터를 Zenoh로 GCS에 전달하고, GCS 명령을 수신합니다.

## 패키지

- **nev_teleop_bot** - ROS 2 ↔ Zenoh 브릿지 + H.265 카메라 인코더
- **nev_remote_msgs** - 커스텀 ROS 2 메시지 정의

## 노드

| 노드 | 역할 |
|------|------|
| `net_bridge.py` | ROS 2 토픽 ↔ Zenoh 브릿지 (텔레메트리, 명령, E-stop, 하트비트) |
| `zenoh_video_encoder.py` | 단일 카메라 H.265 인코딩 → Zenoh |
| `dual_camera_encoder.py` | 2대 카메라 (RealSense + C922) 병합 → H.265 → Zenoh |
| `quad_camera_encoder.py` | 4대 카메라 2x2 그리드 → H.265 → Zenoh |

## 실행

```bash
# 빌드
colcon build --packages-select nev_remote_msgs nev_teleop_bot

# net_bridge
ros2 launch nev_teleop_bot net_bridge.launch.py

# 카메라 인코더 (택 1)
ros2 launch nev_teleop_bot zenoh_video_encoder.launch.py
ros2 launch nev_teleop_bot dual_camera.launch.py
ros2 launch nev_teleop_bot quad_camera_encoder.launch.py
```

## Zenoh 토픽

### 발행 (차량 → GCS)

| 토픽 | 주기 | 내용 |
|------|------|------|
| `nev/robot/mux` | 20 Hz | 모드/소스 상태 |
| `nev/robot/twist` | 20 Hz | 속도/각속도 |
| `nev/robot/hunter` | 20 Hz | 로봇 상태 (배터리, 에러) |
| `nev/robot/estop` | 20 Hz | E-stop 상태 |
| `nev/robot/network` | 20 Hz | 네트워크 상태 |
| `nev/robot/camera` | 10-15 fps | H.265 NAL (10B 헤더 + payload) |
| `nev/robot/cpu,mem,gpu,disk,net` | 1 Hz | 시스템 리소스 |

### 구독 (GCS → 차량)

| 토픽 | 내용 |
|------|------|
| `nev/gcs/heartbeat` | GCS 하트비트 |
| `nev/gcs/teleop` | 속도/각속도 명령 |
| `nev/gcs/estop` | E-stop (RELIABLE) |
| `nev/gcs/cmd_mode` | 모드 변경 (RELIABLE) |

## 의존성

- ROS 2 (rclpy, geometry_msgs, sensor_msgs, std_msgs)
- [eclipse-zenoh](https://zenoh.io/)
- GStreamer + NVIDIA nvh265enc
- OpenCV, NumPy
