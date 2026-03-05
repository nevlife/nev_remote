#!/usr/bin/env python3
"""
DualCameraEncoder
  - RealSense : ROS2 topic  (/camera/camera/color/image_raw)
  - C922      : GStreamer   (v4l2src /dev/video6)
  - 출력       : [ RealSense 640x480 | C922 640x480 ] → 1280x480 → nvh265enc → Zenoh
"""
import json
import struct
import time
import threading
import sys
import collections

import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import zenoh
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image


# ── 각 카메라 출력 크기 (half tile)
HALF_W = 640
HALF_H = 480
# ── 합성 출력 크기
OUT_W  = HALF_W * 2   # 1280
OUT_H  = HALF_H       # 480


def _resize_nn(frame: np.ndarray, h: int, w: int) -> np.ndarray:
    """OpenCV 없이 순수 numpy nearest-neighbor resize"""
    fh, fw = frame.shape[:2]
    row = (np.arange(h) * fh / h).astype(np.int32)
    col = (np.arange(w) * fw / w).astype(np.int32)
    return frame[row][:, col]


class DualCameraEncoder(Node):
    def __init__(self):
        super().__init__('dual_camera_encoder')
        Gst.init(None)

        self.declare_parameter('realsense_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('c922_device',     '/dev/video6')
        self.declare_parameter('c922_fps',        15)
        self.declare_parameter('target_fps',      15)
        self.declare_parameter('bitrate',         800)
        self.declare_parameter('zenoh_locator',   'tcp/203.250.33.77:80')
        self.declare_parameter('zenoh_key',       'nev/vehicle/camera')
        self.declare_parameter('video_stats_key', 'nev/vehicle/video_stats')

        rs_topic   = self.get_parameter('realsense_topic').value
        c922_dev   = self.get_parameter('c922_device').value
        c922_fps   = self.get_parameter('c922_fps').value
        target_fps = self.get_parameter('target_fps').value
        bitrate    = self.get_parameter('bitrate').value
        locator    = self.get_parameter('zenoh_locator').value
        zkey       = self.get_parameter('zenoh_key').value
        stats_key  = self.get_parameter('video_stats_key').value

        self._frame_interval = 1.0 / target_fps
        self._target_fps     = target_fps

        # ── Zenoh ──────────────────────────────────────────────────────────
        conf = zenoh.Config()
        conf.insert_json5('connect/endpoints', json.dumps([locator]))
        self._zsession  = zenoh.open(conf)
        self._zpub      = self._zsession.declare_publisher(
            zkey, congestion_control=zenoh.CongestionControl.DROP)
        self._stats_pub = self._zsession.declare_publisher(stats_key)
        self.get_logger().info(f'Zenoh connected: {locator}  key={zkey}')

        # ── 프레임 저장소 ───────────────────────────────────────────────────
        self._rs_frame   = None   # numpy uint8 RGB
        self._c922_frame = None   # numpy uint8 RGB
        self._lock       = threading.Lock()

        # ── C922 캡처 파이프라인 ────────────────────────────────────────────
        # v4l2src → MJPG 디코드 → RGB → appsink
        cap_str = (
            f'v4l2src device={c922_dev} ! '
            f'image/jpeg,width={HALF_W},height={HALF_H},framerate={c922_fps}/1 ! '
            f'jpegdec ! '
            f'videoconvert ! '
            f'video/x-raw,format=RGB ! '
            f'appsink name=c922_sink drop=true max-buffers=1 sync=false'
        )
        self.get_logger().info(f'C922 pipeline: {cap_str}')
        self._cap_pl  = Gst.parse_launch(cap_str)
        c922_sink     = self._cap_pl.get_by_name('c922_sink')
        c922_sink.set_property('emit-signals', True)
        c922_sink.connect('new-sample', self._on_c922_sample, None)
        self._cap_pl.set_state(Gst.State.PLAYING)
        self.get_logger().info(f'C922 capture started: {c922_dev}')

        # ── 인코딩 파이프라인 ───────────────────────────────────────────────
        # appsrc(RGB 1280x480) → videoconvert(NV12) → nvh265enc → h265parse → appsink
        enc_str = (
            f'appsrc name=enc_src format=time is-live=true do-timestamp=true '
            f'caps="video/x-raw,format=RGB,width={OUT_W},height={OUT_H},'
            f'framerate={target_fps}/1" ! '
            f'videoconvert n-threads=4 qos=true ! '
            f'video/x-raw,format=NV12 ! '
            f'nvh265enc '
            f'preset=low-latency-hq rc-mode=cbr-ld-hq '
            f'bitrate={bitrate} max-bitrate=0 const-quality=0 '
            f'gop-size=15 aud=true qos=false zerolatency=true '
            f'rc-lookahead=0 bframes=0 i-adapt=false b-adapt=false ! '
            f'h265parse config-interval=-1 ! '
            f'video/x-h265,stream-format=byte-stream,alignment=au ! '
            f'appsink name=enc_sink drop=true max-buffers=2 sync=false'
        )
        self.get_logger().info(f'Encode pipeline: {enc_str}')
        self._enc_pl  = Gst.parse_launch(enc_str)
        self._appsrc  = self._enc_pl.get_by_name('enc_src')
        enc_sink      = self._enc_pl.get_by_name('enc_sink')

        self._appsrc.set_property('max-bytes', OUT_W * OUT_H * 3 * 2)
        self._appsrc.set_property('block', False)
        self._appsrc.set_property('leaky-type', 2)

        enc_sink.set_property('emit-signals', True)
        enc_sink.connect('new-sample', self._on_encoded_sample, None)
        self._enc_pl.set_state(Gst.State.PLAYING)
        self.get_logger().info(f'Encode pipeline started: {OUT_W}x{OUT_H} @ {target_fps}fps  {bitrate}kbps')

        # ── 통계 ───────────────────────────────────────────────────────────
        self._ts_queue     = collections.deque(maxlen=30)
        self._tx_bytes     = 0
        self._enc_ms_sum   = 0.0
        self._enc_ms_count = 0
        self._stats_ts     = time.time()

        # ── RealSense ROS2 구독 ─────────────────────────────────────────────
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._rs_sub = self.create_subscription(
            Image, rs_topic, self._on_realsense_image, qos)

        # ── 합성 타이머 ─────────────────────────────────────────────────────
        self.create_timer(self._frame_interval, self._combine_and_push)

        self.get_logger().info(
            f'DualCameraEncoder ready  '
            f'layout=[RealSense {HALF_W}x{HALF_H} | C922 {HALF_W}x{HALF_H}]')

    # ── C922 원시 프레임 콜백 ────────────────────────────────────────────────
    def _on_c922_sample(self, sink, _data):
        sample = sink.emit('pull-sample')
        if not isinstance(sample, Gst.Sample):
            return Gst.FlowReturn.OK
        buf    = sample.get_buffer()
        ok, mi = buf.map(Gst.MapFlags.READ)
        if ok:
            try:
                frame = np.frombuffer(mi.data, np.uint8).reshape(HALF_H, HALF_W, 3).copy()
                with self._lock:
                    self._c922_frame = frame
            except Exception as e:
                self.get_logger().warn(f'C922 frame error: {e}', throttle_duration_sec=5)
            finally:
                buf.unmap(mi)
        return Gst.FlowReturn.OK

    # ── RealSense ROS2 콜백 ──────────────────────────────────────────────────
    def _on_realsense_image(self, msg: Image):
        enc = msg.encoding
        try:
            raw = np.frombuffer(msg.data, np.uint8)
            if enc == 'rgb8':
                frame = raw.reshape(msg.height, msg.width, 3)
            elif enc == 'bgr8':
                frame = raw.reshape(msg.height, msg.width, 3)[:, :, ::-1]
            elif enc == 'rgba8':
                frame = raw.reshape(msg.height, msg.width, 4)[:, :, :3]
            elif enc == 'bgra8':
                frame = raw.reshape(msg.height, msg.width, 4)[:, :, 2::-1]
            else:
                self.get_logger().warn(
                    f'Unsupported encoding: {enc}', throttle_duration_sec=5)
                return
            with self._lock:
                self._rs_frame = frame.copy()
        except Exception as e:
            self.get_logger().warn(
                f'RealSense frame parse error: {e}', throttle_duration_sec=5)

    # ── 합성 → appsrc push ──────────────────────────────────────────────────
    def _combine_and_push(self):
        with self._lock:
            rs   = self._rs_frame
            c922 = self._c922_frame

        # 아직 프레임이 없으면 검은 화면으로 대체
        if c922 is None:
            c922 = np.zeros((HALF_H, HALF_W, 3), np.uint8)
        if rs is None:
            rs_tile = np.zeros((HALF_H, HALF_W, 3), np.uint8)
        else:
            rs_tile = _resize_nn(rs, HALF_H, HALF_W)

        # 좌: RealSense  우: C922
        combined = np.ascontiguousarray(np.hstack([rs_tile, c922]))

        try:
            buf    = Gst.Buffer.new_wrapped(combined.tobytes())
            self._ts_queue.append(time.perf_counter())
            retval = self._appsrc.emit('push-buffer', buf)
            if retval not in (Gst.FlowReturn.OK, Gst.FlowReturn.FLUSHING):
                self.get_logger().warn(
                    f'push-buffer abnormal: {retval}', throttle_duration_sec=3)
        except Exception as e:
            self.get_logger().warn(
                f'push-buffer error: {e}', throttle_duration_sec=5)

    # ── 인코딩된 프레임 → Zenoh ─────────────────────────────────────────────
    def _on_encoded_sample(self, sink, _data):
        sample = sink.emit('pull-sample')
        if not isinstance(sample, Gst.Sample):
            return Gst.FlowReturn.OK

        latency_ms = 0.0
        if self._ts_queue:
            latency_ms = (time.perf_counter() - self._ts_queue.popleft()) * 1000.0

        buf    = sample.get_buffer()
        ok, mi = buf.map(Gst.MapFlags.READ)
        if ok:
            try:
                nal_bytes = bytes(mi.data)
                header    = struct.pack('dH', time.time(), int(latency_ms))
                self._zpub.put(header + nal_bytes)
                self._tx_bytes     += len(nal_bytes)
                self._enc_ms_sum   += latency_ms
                self._enc_ms_count += 1
            except Exception as e:
                self.get_logger().warn(
                    f'Zenoh put failed: {e}', throttle_duration_sec=3)
            finally:
                buf.unmap(mi)

        now = time.time()
        dt  = now - self._stats_ts
        if dt >= 1.0:
            bw_mbps   = round(self._tx_bytes * 8 / (dt * 1e6), 3)
            encode_ms = round(self._enc_ms_sum / self._enc_ms_count, 2) \
                        if self._enc_ms_count > 0 else 0.0
            try:
                self._stats_pub.put(
                    json.dumps({'bw_mbps': bw_mbps, 'encode_ms': encode_ms}))
            except Exception:
                pass
            self.get_logger().info(
                f'BW={bw_mbps:.3f}Mbps  encode={encode_ms:.1f}ms',
                throttle_duration_sec=1)
            self._tx_bytes = self._enc_ms_sum = self._enc_ms_count = 0
            self._stats_ts = now

        return Gst.FlowReturn.OK

    # ── 종료 ─────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info('Shutting down pipelines...')
        self._cap_pl.set_state(Gst.State.NULL)
        self._enc_pl.set_state(Gst.State.NULL)
        self._zpub.undeclare()
        self._stats_pub.undeclare()
        self._zsession.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DualCameraEncoder()
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
