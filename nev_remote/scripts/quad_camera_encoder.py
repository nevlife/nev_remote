#!/usr/bin/env python3
"""
QuadCameraEncoder – 4카메라 쿼드뷰 H.265 Zenoh 스트리머
  Q1(top-right): RealSense  ROS2 topic       (16:9 → letterbox)
  Q2(top-left) : C922       v4l2src 640x480  (직접 캡처)
  Q3(bot-left) : Left cam   v4l2src 640x480  (직접 캡처)
  Q4(bot-right): Right cam  v4l2src 640x480  (직접 캡처)

  레이아웃: [ C922(Q2) | RealSense(Q1) ]
            [ Left(Q3) | Right(Q4)     ]
  출력: 1280x960 → nvh265enc → Zenoh
  레이턴시: appsrc do-timestamp=true + buf.pts 기반
"""
import collections
import json
import struct
import time
import threading

import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import zenoh
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image


TILE_W = 640
TILE_H = 480
OUT_W  = TILE_W * 2   # 1280
OUT_H  = TILE_H * 2   # 960


def _resize_nn(frame: np.ndarray, h: int, w: int) -> np.ndarray:
    fh, fw = frame.shape[:2]
    if fh == h and fw == w:
        return frame
    row = (np.arange(h) * fh / h).astype(np.int32)
    col = (np.arange(w) * fw / w).astype(np.int32)
    return frame[row][:, col]


def _letterbox(frame: np.ndarray, h: int, w: int) -> np.ndarray:
    fh, fw = frame.shape[:2]
    scale  = min(w / fw, h / fh)
    nw, nh = int(fw * scale), int(fh * scale)
    resized = _resize_nn(frame, nh, nw)
    out = np.zeros((h, w, 3), np.uint8)
    out[(h - nh) // 2:(h - nh) // 2 + nh,
        (w - nw) // 2:(w - nw) // 2 + nw] = resized
    return out


class QuadCameraEncoder(Node):

    def __init__(self):
        super().__init__('quad_camera_encoder')
        Gst.init(None)

        # ── 파라미터 ──────────────────────────────────────────────────────────
        self.declare_parameter('realsense_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('c922_device',     '/dev/video8')
        self.declare_parameter('left_device',     '/dev/video6')
        self.declare_parameter('right_device',    '/dev/video11')
        self.declare_parameter('capture_fps',     15)
        self.declare_parameter('target_fps',      15)
        self.declare_parameter('zenoh_locator',   'tcp/203.250.33.77:80')
        self.declare_parameter('zenoh_key',       'nev/vehicle/camera')
        self.declare_parameter('video_stats_key', 'nev/vehicle/video_stats')
        self.declare_parameter('bitrate',         500)
        self.declare_parameter('max-bitrate',     0)
        self.declare_parameter('preset',          'low-latency-hq')
        self.declare_parameter('rc-mode',         'cbr-ld-hq')
        self.declare_parameter('gop-size',        15)
        self.declare_parameter('aud',             True)
        self.declare_parameter('zerolatency',     True)
        self.declare_parameter('rc-lookahead',    0)
        self.declare_parameter('bframes',         0)
        self.declare_parameter('i-adapt',         False)
        self.declare_parameter('b-adapt',         False)
        self.declare_parameter('n-threads',       4)
        self.declare_parameter('vc-qos',          True)

        rs_topic   = self.get_parameter('realsense_topic').value
        c922_dev   = self.get_parameter('c922_device').value
        left_dev   = self.get_parameter('left_device').value
        right_dev  = self.get_parameter('right_device').value
        cap_fps    = self.get_parameter('capture_fps').value
        target_fps = self.get_parameter('target_fps').value
        locator    = self.get_parameter('zenoh_locator').value
        zkey       = self.get_parameter('zenoh_key').value
        stats_key  = self.get_parameter('video_stats_key').value

        self._frame_interval = 1.0 / target_fps
        self._rs_last_t      = 0.0

        # ── Zenoh ──────────────────────────────────────────────────────────
        conf = zenoh.Config()
        conf.insert_json5('connect/endpoints', json.dumps([locator]))
        self._zsession  = zenoh.open(conf)
        self._zpub      = self._zsession.declare_publisher(
            zkey, congestion_control=zenoh.CongestionControl.DROP)
        self._stats_pub = self._zsession.declare_publisher(stats_key)
        self.get_logger().info(f'Zenoh connected: {locator}  key={zkey}')

        # ── 프레임 저장소 ────────────────────────────────────────────────────
        self._rs_frame    = None
        self._c922_frame  = None
        self._left_frame  = None
        self._right_frame = None
        self._lock        = threading.Lock()
        self._black_tile  = np.zeros((TILE_H, TILE_W, 3), np.uint8)

        # ── USB 카메라 캡처 파이프라인 3개 ──────────────────────────────────
        for name, dev, w, h, flip in [
            ('c922',  c922_dev,  640, 480, False),
            ('left',  left_dev,  640, 480, True),
            ('right', right_dev, 640, 480, True),
        ]:
            pl = self._build_capture_pipeline(name, dev, w, h, cap_fps, flip)
            sink = pl.get_by_name(f'{name}_sink')
            sink.set_property('emit-signals', True)
            sink.connect('new-sample', self._make_frame_cb(name, h, w), None)
            pl.set_state(Gst.State.PLAYING)
            setattr(self, f'_cap_pl_{name}', pl)
            self.get_logger().info(f'{name} capture started: {dev} {w}x{h}@{cap_fps}')

        # ── 인코딩 파이프라인 ────────────────────────────────────────────────
        bitrate      = self.get_parameter('bitrate').value
        max_bitrate  = self.get_parameter('max-bitrate').value
        preset       = self.get_parameter('preset').value
        rc_mode      = self.get_parameter('rc-mode').value
        gop_size     = self.get_parameter('gop-size').value
        aud          = str(self.get_parameter('aud').value).lower()
        zerolatency  = str(self.get_parameter('zerolatency').value).lower()
        rc_lookahead = self.get_parameter('rc-lookahead').value
        bframes      = self.get_parameter('bframes').value
        i_adapt      = str(self.get_parameter('i-adapt').value).lower()
        b_adapt      = str(self.get_parameter('b-adapt').value).lower()
        n_threads    = self.get_parameter('n-threads').value
        vc_qos       = str(self.get_parameter('vc-qos').value).lower()

        enc_str = (
            f'appsrc name=enc_src format=time is-live=true do-timestamp=true '
            f'caps="video/x-raw,format=RGB,width={OUT_W},height={OUT_H},'
            f'framerate={target_fps}/1" ! '
            f'videoconvert n-threads={n_threads} qos={vc_qos} ! '
            f'video/x-raw,format=NV12 ! '
            f'nvh265enc '
            f'preset={preset} rc-mode={rc_mode} '
            f'bitrate={bitrate} max-bitrate={max_bitrate} const-quality=0 '
            f'gop-size={gop_size} aud={aud} qos=false zerolatency={zerolatency} '
            f'rc-lookahead={rc_lookahead} bframes={bframes} '
            f'i-adapt={i_adapt} b-adapt={b_adapt} ! '
            f'h265parse config-interval=-1 ! '
            f'video/x-h265,stream-format=byte-stream,alignment=au ! '
            f'appsink name=enc_sink drop=true max-buffers=2 sync=false'
        )
        self.get_logger().info(f'Encode pipeline: {enc_str}')
        self._enc_pl = Gst.parse_launch(enc_str)
        self._appsrc = self._enc_pl.get_by_name('enc_src')
        enc_sink     = self._enc_pl.get_by_name('enc_sink')

        self._appsrc.set_property('max-bytes', OUT_W * OUT_H * 3 * 2)
        self._appsrc.set_property('block', False)
        self._appsrc.set_property('leaky-type', 2)

        enc_sink.set_property('emit-signals', True)
        enc_sink.connect('new-sample', self._on_encoded_sample, None)
        self._enc_pl.set_state(Gst.State.PLAYING)
        self.get_logger().info(
            f'Encode pipeline started: {OUT_W}x{OUT_H} @ {target_fps}fps  {bitrate}kbps')

        # ── 통계 ─────────────────────────────────────────────────────────────
        self._ts_queue     = collections.deque(maxlen=30)
        self._tx_bytes     = 0
        self._enc_ms_sum   = 0.0
        self._enc_ms_count = 0
        self._stats_ts     = time.time()

        # ── RealSense 구독 ───────────────────────────────────────────────────
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._rs_sub = self.create_subscription(
            Image, rs_topic, self._on_realsense_image, qos)

        # ── 합성 타이머 ──────────────────────────────────────────────────────
        self.create_timer(self._frame_interval, self._combine_and_push)

        self.get_logger().info(
            f'QuadCameraEncoder ready  layout=[C922|RS / Left|Right]  out={OUT_W}x{OUT_H}')

    # ── 유틸 ─────────────────────────────────────────────────────────────────

    def _build_capture_pipeline(self, name, dev, w, h, fps, flip=False):
        flip_el = 'videoflip method=rotate-180 ! ' if flip else ''
        pipe_str = (
            f'v4l2src device={dev} ! '
            f'image/jpeg,width={w},height={h},framerate={fps}/1 ! '
            f'jpegdec ! {flip_el}videoconvert ! video/x-raw,format=RGB ! '
            f'appsink name={name}_sink drop=true max-buffers=1 sync=false'
        )
        return Gst.parse_launch(pipe_str)

    def _make_frame_cb(self, cam, fh, fw):
        attr = f'_{cam}_frame'

        def _cb(sink, _):
            sample = sink.emit('pull-sample')
            if not isinstance(sample, Gst.Sample):
                return Gst.FlowReturn.OK
            buf    = sample.get_buffer()
            ok, mi = buf.map(Gst.MapFlags.READ)
            if ok:
                try:
                    frame = np.frombuffer(mi.data, np.uint8).reshape(fh, fw, 3).copy()
                    with self._lock:
                        setattr(self, attr, frame)
                except Exception as e:
                    self.get_logger().warn(
                        f'{cam} frame error: {e}', throttle_duration_sec=5)
                finally:
                    buf.unmap(mi)
            return Gst.FlowReturn.OK

        return _cb

    # ── RealSense 콜백 ────────────────────────────────────────────────────────
    def _on_realsense_image(self, msg: Image):
        now = time.monotonic()
        if now - self._rs_last_t < self._frame_interval:
            return
        self._rs_last_t = now

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
                f'RealSense frame error: {e}', throttle_duration_sec=5)

    # ── 합성 → appsrc push ────────────────────────────────────────────────────
    def _combine_and_push(self):
        with self._lock:
            rs    = self._rs_frame
            c922  = self._c922_frame
            left  = self._left_frame
            right = self._right_frame

        blk = self._black_tile

        # RealSense만 letterbox (16:9 → 640x480), 나머지는 640x480 직접 캡처
        q1 = _letterbox(rs, TILE_H, TILE_W) if rs    is not None else blk  # top-right
        q2 = c922  if c922  is not None else blk                            # top-left
        q3 = left  if left  is not None else blk                            # bot-left
        q4 = right if right is not None else blk                            # bot-right

        combined = np.ascontiguousarray(
            np.vstack([
                np.hstack([q2, q1]),
                np.hstack([q3, q4]),
            ])
        )

        try:
            buf    = Gst.Buffer.new_wrapped(combined.tobytes())
            self._ts_queue.append(time.perf_counter())
            retval = self._appsrc.emit('push-buffer', buf)
            if retval not in (Gst.FlowReturn.OK, Gst.FlowReturn.FLUSHING):
                self.get_logger().warn(
                    f'push-buffer abnormal: {retval}', throttle_duration_sec=3)
        except Exception as e:
            self.get_logger().warn(f'push-buffer error: {e}', throttle_duration_sec=5)

    # ── 인코딩 완료 → Zenoh ───────────────────────────────────────────────────
    def _on_encoded_sample(self, sink, _):
        sample = sink.emit('pull-sample')
        if not isinstance(sample, Gst.Sample):
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()

        encode_latency_ms = 0.0
        if self._ts_queue:
            encode_latency_ms = (time.perf_counter() - self._ts_queue.popleft()) * 1000.0

        ok, mi = buf.map(Gst.MapFlags.READ)
        if ok:
            try:
                nal_bytes = bytes(mi.data)
                header    = struct.pack('dH', time.time(), int(encode_latency_ms))
                self._zpub.put(header + nal_bytes)
                self._tx_bytes     += len(nal_bytes)
                self._enc_ms_sum   += encode_latency_ms
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
            self.get_logger().info(f'BW={bw_mbps:.3f}Mbps  encode={encode_ms:.5f}ms')
            self._tx_bytes = self._enc_ms_sum = self._enc_ms_count = 0
            self._stats_ts = now

        return Gst.FlowReturn.OK

    # ── 종료 ──────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info('Shutting down pipelines...')
        for attr in ('_cap_pl_c922', '_cap_pl_left', '_cap_pl_right', '_enc_pl'):
            pl = getattr(self, attr, None)
            if pl:
                pl.set_state(Gst.State.NULL)
        self._zpub.undeclare()
        self._stats_pub.undeclare()
        self._zsession.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = QuadCameraEncoder()
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
