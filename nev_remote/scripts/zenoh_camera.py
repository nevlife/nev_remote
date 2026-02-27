#!/usr/bin/env python3
"""
ROS2 카메라 토픽 → GStreamer(NVENC) H.264 압축 → Zenoh 발행
"""
import json
import time
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import cv2
import numpy as np
import zenoh
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class ZenohCamera(Node):
    def __init__(self):
        super().__init__('zenoh_camera')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('image_topic',   '/camera/camera/color/image_raw')
        self.declare_parameter('zenoh_locator', 'tcp/127.0.0.1:7447')
        self.declare_parameter('width',         640)
        self.declare_parameter('height',        480)
        self.declare_parameter('fps',           15)
        self.declare_parameter('bitrate_kbps',  500)

        topic   = self.get_parameter('image_topic').value
        locator = self.get_parameter('zenoh_locator').value
        width   = self.get_parameter('width').value
        height  = self.get_parameter('height').value
        fps     = self.get_parameter('fps').value
        bitrate = self.get_parameter('bitrate_kbps').value * 1000

        self._enc_width  = width
        self._enc_height = height
        self._frame_interval = 1.0 / fps        # 프레임 간격 (초)
        self._last_push_time = 0.0              # 마지막으로 appsrc에 넣은 시각

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
        self.appsrc  = self.pipeline.get_by_name('appsrc')
        self.appsink = self.pipeline.get_by_name('appsink')

        # appsrc 큐 제한: 프레임 2개 분량만 허용, 가득 차면 오래된 것부터 버림
        self.appsrc.set_property('max-bytes',   width * height * 3 * 2)
        self.appsrc.set_property('block',       False)
        self.appsrc.set_property('leaky-type',  2)   # 2 = downstream (오래된 버퍼 드롭)

        self.appsink.set_property('emit-signals', True)
        self.appsink.connect('new-sample', self._on_new_sample, None)

        # ── ROS2 Subscription (BEST_EFFORT + depth=1) ─────────────────────────
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._sub = self.create_subscription(Image, topic, self._on_ros_image, qos)

        # ── Start Pipeline ────────────────────────────────────────────────────
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(
            f'Zenoh H.264 Camera 시작 | {width}x{height} @ {fps}fps | '
            f'bitrate={bitrate/1000}kbps | topic={topic} | locator={locator}'
        )

    def _create_pipeline(self, w: int, h: int, fps: int, bitrate: int):
        bitrate_kbps = bitrate // 1000
        pipeline_str = (
            f'appsrc name=appsrc format=time is-live=true do-timestamp=true '
            f'caps="video/x-raw,format=BGR,width={w},height={h},framerate={fps}/1" ! '
            f'videoconvert ! '
            f'video/x-raw,format=NV12 ! '
            f'nvh265enc bitrate={bitrate_kbps} zerolatency=true rc-mode=cbr-ld-hq gop-size={fps} ! '
            f'h265parse config-interval=-1 ! '
            f'video/x-h265,stream-format=byte-stream,alignment=au ! '
            f'appsink name=appsink drop=true max-buffers=2 sync=false'
        )
        self.get_logger().info(f"GStreamer 파이프라인: {pipeline_str}")
        try:
            return Gst.parse_launch(pipeline_str)
        except gi.repository.GLib.GError as e:
            self.get_logger().error(f"파이프라인 생성 실패: {e}")
            raise

    def _on_ros_image(self, msg: Image):
        # fps 기준으로 처리 간격보다 이른 프레임은 즉시 버림
        now = time.monotonic()
        if now - self._last_push_time < self._frame_interval:
            return
        self._last_push_time = now

        try:
            frame = self._to_bgr(msg)
            if frame is None:
                return
            if frame.shape[1] != self._enc_width or frame.shape[0] != self._enc_height:
                frame = cv2.resize(frame, (self._enc_width, self._enc_height),
                                   interpolation=cv2.INTER_LINEAR)
            buf = Gst.Buffer.new_wrapped(frame.tobytes())
            retval = self.appsrc.emit('push-buffer', buf)
            if retval not in (Gst.FlowReturn.OK, Gst.FlowReturn.FLUSHING):
                self.get_logger().warning(f'버퍼 주입 상태 이상: {retval}',
                                          throttle_duration_sec=3)
        except Exception as e:
            self.get_logger().warning(f'Frame 주입 실패: {e}', throttle_duration_sec=5)

    def _on_new_sample(self, sink, data):
        sample = sink.emit('pull-sample')
        if isinstance(sample, Gst.Sample):
            buf = sample.get_buffer()
            result, map_info = buf.map(Gst.MapFlags.READ)
            if result:
                try:
                    self._zpub.put(map_info.data)
                except Exception as e:
                    self.get_logger().warning(f'Zenoh 발행 실패: {e}',
                                              throttle_duration_sec=3)
                finally:
                    buf.unmap(map_info)
        return Gst.FlowReturn.OK

    def _to_bgr(self, msg: Image) -> np.ndarray | None:
        enc = msg.encoding
        h, w = msg.height, msg.width
        try:
            if enc == 'bgr8':
                return np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
            if enc == 'rgb8':
                arr = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
                return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            if enc == 'rgba8':
                arr = np.frombuffer(msg.data, np.uint8).reshape(h, w, 4)
                return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
            if enc == 'mono8':
                arr = np.frombuffer(msg.data, np.uint8).reshape(h, w)
                return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패 ({enc}): {e}',
                                    throttle_duration_sec=5)
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
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()