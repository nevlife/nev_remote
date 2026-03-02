#!/usr/bin/env python3
"""
ROS2 카메라 토픽 → GStreamer(NVENC) H.265 초저지연 압축 → Zenoh 발행
(설정 파일 기반 파이프라인 동적 제어 및 RTX 3060 완벽 최적화 버전)
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

        # ── 1. 파라미터 선언 및 불러오기 (config.yaml 매핑) ───────────────────
        self.declare_parameter('image_topic',          '/camera/camera/color/image_raw')
        self.declare_parameter('zenoh_locator',        'tcp/127.0.0.1:7447')
        self.declare_parameter('width',                640)
        self.declare_parameter('height',               480)
        self.declare_parameter('fps',                  15.0)
        self.declare_parameter('bitrate_kbps',         500)
        self.declare_parameter('videoconvert_threads', 4)
        self.declare_parameter('encoder_preset',       'low-latency-hq')
        self.declare_parameter('encoder_rc_mode',      'cbr-ld-hq')
        self.declare_parameter('encoder_zerolatency',  True)
        self.declare_parameter('encoder_spatial_aq',   True)  # 추가된 공간 양자화 옵션
        self.declare_parameter('gop_size',             15)
        self.declare_parameter('config_interval',      -1)

        self._topic        = self.get_parameter('image_topic').value
        self._locator      = self.get_parameter('zenoh_locator').value
        self._target_w     = self.get_parameter('width').value
        self._target_h     = self.get_parameter('height').value
        self._fps          = int(self.get_parameter('fps').value)
        self._bitrate      = self.get_parameter('bitrate_kbps').value
        self._v_threads    = self.get_parameter('videoconvert_threads').value
        self._enc_preset   = self.get_parameter('encoder_preset').value
        self._enc_rc_mode  = self.get_parameter('encoder_rc_mode').value
        self._gop_size     = self.get_parameter('gop_size').value
        self._config_int   = self.get_parameter('config_interval').value
        
        # GStreamer 문법 처리를 위해 파이썬의 True/False를 'true'/'false' 문자열로 변환
        self._enc_zerolat  = str(self.get_parameter('encoder_zerolatency').value).lower()
        self._enc_spatial  = str(self.get_parameter('encoder_spatial_aq').value).lower()

        self._frame_interval = 1.0 / self._fps
        self._last_push_time = 0.0

        # ── 2. Zenoh 초기화 ───────────────────────────────────────────────────
        conf = zenoh.Config()
        if self._locator:
            conf.insert_json5('connect/endpoints', json.dumps([self._locator]))
        self._zsession = zenoh.open(conf)
        self._zpub = self._zsession.declare_publisher(
            'nev/vehicle/camera',
            congestion_control=zenoh.CongestionControl.DROP,
        )

        # ── 3. GStreamer 및 ROS2 구독 초기화 ──────────────────────────────────
        Gst.init(None)
        self.pipeline = None  # 첫 이미지가 들어올 때 해상도에 맞춰 동적 생성됨

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._sub = self.create_subscription(Image, self._topic, self._on_ros_image, qos)

        self.get_logger().info(f'Zenoh 카메라 노드 대기 중... (토픽: {self._topic})')

    def _init_pipeline(self, orig_w: int, orig_h: int):
        """첫 번째 프레임의 원본 해상도(orig_w, orig_h)를 바탕으로 파이프라인 동적 구축"""
        
        # 파라미터 값들이 모두 적용된 GStreamer 파이프라인 문자열
        pipeline_str = (
            f'appsrc name=appsrc format=time is-live=true do-timestamp=true '
            f'caps="video/x-raw,format=BGR,width={orig_w},height={orig_h},framerate={self._fps}/1" ! '
            f'videoscale ! '
            f'video/x-raw,width={self._target_w},height={self._target_h} ! '
            f'videoconvert n-threads={self._v_threads} ! '
            f'video/x-raw,format=NV12 ! '
            f'nvh265enc preset={self._enc_preset} bitrate={self._bitrate} '
            f'zerolatency={self._enc_zerolat} rc-mode={self._enc_rc_mode} '
            f'spatial-aq={self._enc_spatial} gop-size={self._gop_size} ! '
            f'h265parse config-interval={self._config_int} ! '
            f'video/x-h265,stream-format=byte-stream,alignment=au ! '
            f'appsink name=appsink drop=true max-buffers=2 sync=false'
        )
        
        self.get_logger().info(f"🚀 파이프라인 구축 완료:\n{pipeline_str}")
        
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except Exception as e:
            self.get_logger().error(f"파이프라인 생성 실패: {e}")
            raise

        self.appsrc  = self.pipeline.get_by_name('appsrc')
        self.appsink = self.pipeline.get_by_name('appsink')

        # appsrc 큐 제한 (원본 해상도 기준 프레임 2개분)
        self.appsrc.set_property('max-bytes', orig_w * orig_h * 3 * 2)
        self.appsrc.set_property('block', False)
        self.appsrc.set_property('leaky-type', 2)

        self.appsink.set_property('emit-signals', True)
        self.appsink.connect('new-sample', self._on_new_sample, None)

        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(f'🎥 전송 시작 | 타겟: {self._target_w}x{self._target_h} @ {self._fps}fps | {self._bitrate}kbps')

    def _on_ros_image(self, msg: Image):
        now = time.monotonic()
        if now - self._last_push_time < self._frame_interval:
            return
        self._last_push_time = now

        if self.pipeline is None:
            self._init_pipeline(msg.width, msg.height)

        try:
            frame = self._to_bgr(msg)
            if frame is None:
                return

            buf = Gst.Buffer.new_wrapped(frame.tobytes())
            retval = self.appsrc.emit('push-buffer', buf)
            
            if retval not in (Gst.FlowReturn.OK, Gst.FlowReturn.FLUSHING):
                self.get_logger().warning(f'버퍼 주입 상태 이상: {retval}', throttle_duration_sec=3)
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
                    self.get_logger().warning(f'Zenoh 발행 실패: {e}', throttle_duration_sec=3)
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
            self.get_logger().error(f'이미지 변환 실패 ({enc}): {e}', throttle_duration_sec=5)
            return None
        return None

    def destroy_node(self):
        self.get_logger().info('GStreamer 파이프라인 종료 중...')
        if self.pipeline:
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