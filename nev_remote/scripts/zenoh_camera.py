#!/usr/bin/env python3
"""
ROS2 카메라 토픽 → GStreamer(NVENC) H.264 압축 → Zenoh 발행

- ROS 이미지를 수신하여 GStreamer 파이프라인으로 전달
- NVIDIA 하드웨어 인코더(nvh264enc)를 사용하여 H.264 비트스트림 생성
- 생성된 H.264 스트림을 Zenoh 토픽(`nev/vehicle/camera`)으로 발행
"""
import json
import threading
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import cv2
import numpy as np
import zenoh
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ZenohCamera(Node):
    def __init__(self):
        super().__init__('zenoh_camera')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('image_topic',    '/camera/image_raw')
        self.declare_parameter('zenoh_locator',  'tcp/127.0.0.1:7447')
        self.declare_parameter('width',          1280)
        self.declare_parameter('height',         720)
        self.declare_parameter('fps',            30)
        self.declare_parameter('bitrate_kbps',   2000)

        topic   = self.get_parameter('image_topic').value
        locator = self.get_parameter('zenoh_locator').value
        width   = self.get_parameter('width').value
        height  = self.get_parameter('height').value
        fps     = self.get_parameter('fps').value
        bitrate = self.get_parameter('bitrate_kbps').value * 1000

        # ── Zenoh ─────────────────────────────────────────────────────────────
        conf = zenoh.Config()
        if locator:
            conf.insert_json5('connect/endpoints', json.dumps([locator]))
        self._zsession = zenoh.open(conf)
        self._zpub = self._zsession.declare_publisher(
            'nev/vehicle/camera',
            congestion_control=zenoh.CongestionControl.DROP,
        )

        # ── GStreamer Pipeline ────────────────────────────────────────────────
        Gst.init(None)
        self.pipeline = self._create_pipeline(width, height, fps, bitrate)
        self.appsink = self.pipeline.get_by_name('appsink')
        self.appsink.set_property('emit-signals', True)
        self.appsink.connect('new-sample', self._on_new_sample, None)

        # ── ROS2 Subscription ─────────────────────────────────────────────────
        self._sub = self.create_subscription(Image, topic, self._on_ros_image, 1)

        # ── Start Pipeline ────────────────────────────────────────────────────
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(
            f'Zenoh H.264 Camera 시작 | {width}x{height} @ {fps}fps | '
            f'bitrate={bitrate/1000}kbps | topic={topic} | locator={locator}'
        )

    def _create_pipeline(self, w: int, h: int, fps: int, bitrate: int):
        # ROS 이미지를 받아 H.264로 하드웨어 인코딩하는 GStreamer 파이프라인
        pipeline_str = (
            f'appsrc name=appsrc format=time is-live=true do-timestamp=true ! '
            f'videoconvert ! '
            f'nvvideoconvert ! '
            f'video/x-raw(memory:NVMM),width={w},height={h},format=NV12,framerate={fps}/1 ! '
            f'nvv4l2h264enc bitrate={bitrate} preset-level=4 control-rate=1 ! '
            f'h264parse config-interval=-1 ! '
            f'video/x-h264,stream-format=byte-stream,alignment=au ! '
            f'appsink name=appsink drop=true max-buffers=2 sync=false'
        )
        self.get_logger().info(f"GStreamer 파이프라인: {pipeline_str}")
        return Gst.parse_launch(pipeline_str)

    def _on_ros_image(self, msg: Image):
        # ROS 이미지를 GStreamer `appsrc`에 주입
        try:
            frame = self._to_bgr(msg)
            if frame is None:
                return

            data = frame.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            
            appsrc = self.pipeline.get_by_name('appsrc')
            appsrc.push_buffer(buf)

        except Exception as e:
            self.get_logger().warning(f'Frame 주입 실패: {e}', throttle_duration_sec=5)

    def _on_new_sample(self, sink, data):
        # 인코딩된 H.264 샘플을 Zenoh로 발행
        sample = sink.emit('pull-sample')
        if isinstance(sample, Gst.Sample):
            buf = sample.get_buffer()
            result, map_info = buf.map(Gst.MapFlags.READ)
            if result:
                try:
                    self._zpub.put(map_info.data)
                except Exception as e:
                    self.get_logger().warning(f'Zenoh 발행 실패: {e}', throttle_duration_sec=3)
                finally:
                    buf.unmap(map_info)
        return Gst.FlowReturn.OK

    def _to_bgr(self, msg: Image) -> np.ndarray | None:
        # 다양한 ROS 이미지 포맷을 BGR8로 변환 (GStreamer 주입용)
        enc = msg.encoding
        h, w = msg.height, msg.width
        try:
            if enc == 'bgr8':
                return np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
            if enc == 'rgb8':
                arr = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
                return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            # 다른 포맷에 대한 변환 로직 추가 가능 (rgba8, mono8 등)
            # ...
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패 ({enc}): {e}', throttle_duration_sec=5)
            return None
        
        self.get_logger().warn(f'지원하지 않는 인코딩: {enc}', throttle_duration_sec=10)
        return None

    def destroy_node(self):
        self.get_logger().info('GStreamer 파이프라인 종료 중...')
        self.pipeline.set_state(Gst.State.NULL)
        self._zpub.undeclare()
        self._zsession.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ZenohCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
