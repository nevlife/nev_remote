#!/usr/bin/env python3
import json
import struct
import time
import threading
import collections
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import numpy as np
import zenoh
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image

class ZenohVideoEncoder(Node):
    def __init__(self):
        super().__init__('zenoh_camera')
        self.declare_parameter('image_topic',     '/camera/camera/color/image_raw')
        self.declare_parameter('zenoh_locator',   'tcp/127.0.0.1:7447')
        self.declare_parameter('width',           640)
        self.declare_parameter('height',          480)
        self.declare_parameter('max_fps',         15.0)
        self.declare_parameter('config_interval', -1)
        self.declare_parameter('n-threads',       4)
        self.declare_parameter('vc-qos',          True)
        self.declare_parameter('preset',          'low-latency-hq')
        self.declare_parameter('rc-mode',         'cbr-ld-hq')
        self.declare_parameter('bitrate',         500)
        self.declare_parameter('max-bitrate',     0)
        self.declare_parameter('const-quality',   0.0)
        self.declare_parameter('gop-size',        15)
        self.declare_parameter('aud',             True)
        self.declare_parameter('gst_qos',          False)
        self.declare_parameter('zerolatency',     True)
        self.declare_parameter('rc-lookahead',    0)
        self.declare_parameter('bframes',         0)
        self.declare_parameter('i-adapt',         False)
        self.declare_parameter('b-adapt',         False)

        self._topic        = self.get_parameter('image_topic').value
        self._locator      = self.get_parameter('zenoh_locator').value
        self._target_w     = self.get_parameter('width').value
        self._target_h     = self.get_parameter('height').value
        self._fps          = int(self.get_parameter('max_fps').value)
        self._config_int   = self.get_parameter('config_interval').value

        self._frame_interval = 1.0 / self._fps
        self._last_push_time = 0.0

        # Start time frame
        self._timestamp_queue = collections.deque(maxlen=30)

        conf = zenoh.Config()
        if self._locator:
            conf.insert_json5('connect/endpoints', json.dumps([self._locator]))
        try:
            self._zsession = zenoh.open(conf)
        except Exception as e:
            self.get_logger().fatal(f'Zenoh connect failed: {e}')
            raise SystemExit(1)
        self._zpub       = self._zsession.declare_publisher('nev/robot/camera', congestion_control=zenoh.CongestionControl.DROP)
        self._vstats_pub = self._zsession.declare_publisher('nev/robot/video_stats')

        self._stats_lock     = threading.Lock()
        self._tx_bytes       = 0
        self._encode_ms_sum  = 0.0
        self._encode_ms_count = 0
        self._stats_ts       = time.time()

        Gst.init(None)
        self.pipeline = None

        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._sub = self.create_subscription(Image, self._topic, self._on_ros_image, qos_profile)

        self.get_logger().info(f"Zenoh node ready... (topic: {self._topic})")

    def _get_str_bool(self, param_name: str) -> str:
        return str(self.get_parameter(param_name).value).lower()

    def _init_pipeline(self, orig_w: int, orig_h: int, gst_format: str):
        
        v_threads = self.get_parameter('n-threads').value
        vc_qos = self._get_str_bool('vc-qos')
        
        enc_options = (
            f'preset={self.get_parameter("preset").value} '
            f'rc-mode={self.get_parameter("rc-mode").value} '
            f'bitrate={self.get_parameter("bitrate").value} '
            f'max-bitrate={self.get_parameter("max-bitrate").value} '
            f'const-quality={self.get_parameter("const-quality").value} '
            f'gop-size={self.get_parameter("gop-size").value} '
            f'aud={self._get_str_bool("aud")} '
            f'qos={self._get_str_bool("gst_qos")} '
            f'zerolatency={self._get_str_bool("zerolatency")} '
            f'rc-lookahead={self.get_parameter("rc-lookahead").value} '
            f'bframes={self.get_parameter("bframes").value} '
            f'i-adapt={self._get_str_bool("i-adapt")} '
            f'b-adapt={self._get_str_bool("b-adapt")}'
        )

        scale_element = ""
        if orig_w != self._target_w or orig_h != self._target_h:
            scale_element = f"videoscale ! video/x-raw,width={self._target_w},height={self._target_h} ! "

        pipeline_str = (
            f'appsrc name=appsrc format=time is-live=true do-timestamp=true '
            f'caps="video/x-raw,format={gst_format},width={orig_w},height={orig_h},framerate={self._fps}/1" ! '
            f'{scale_element}'
            f'videoconvert n-threads={v_threads} qos={vc_qos} ! '
            f'video/x-raw,format=NV12 ! '
            f'nvh265enc {enc_options} ! '
            f'h265parse config-interval={self._config_int} ! '
            f'video/x-h265,stream-format=byte-stream,alignment=au ! '
            f'appsink name=appsink drop=true max-buffers=2 sync=false'
        )
        
        self.get_logger().info(f"pipeline: {pipeline_str}")

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except Exception as e:
            self.get_logger().error(f"Failed to create pipeline: {e}")
            raise

        self.appsrc  = self.pipeline.get_by_name('appsrc')
        self.appsink = self.pipeline.get_by_name('appsink')

        self.appsrc.set_property('max-bytes', orig_w * orig_h * 3 * 2)
        self.appsrc.set_property('block', False)
        self.appsrc.set_property('leaky-type', 2)

        self.appsink.set_property('emit-signals', True)
        self.appsink.connect('new-sample', self._on_new_sample, None)

        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(f'Started streaming : {self._target_w}x{self._target_h} @ {self._fps}fps')

    def _on_ros_image(self, msg: Image):
        now = time.monotonic()
        if now - self._last_push_time < self._frame_interval:
            return
        self._last_push_time = now

        frame, gst_format = self._extract_raw_frame(msg)
        if frame is None:
            return

        if self.pipeline is None:
            self._init_pipeline(msg.width, msg.height, gst_format)

        try:
            buf = Gst.Buffer.new_wrapped(frame.tobytes())

            # Before pushing the buffer, store the current time in the queue
            self._timestamp_queue.append(time.perf_counter())
            
            retval = self.appsrc.emit('push-buffer', buf)
            
            if retval not in (Gst.FlowReturn.OK, Gst.FlowReturn.FLUSHING):
                self.get_logger().warning(f'Buffer injection status abnormal: {retval}', throttle_duration_sec=3)
        except Exception as e:
            self.get_logger().warning(f'Frame injection failed: {e}', throttle_duration_sec=5)

    def _on_new_sample(self, sink, data):
        sample = sink.emit('pull-sample')
        if isinstance(sample, Gst.Sample):

            # When the encoded frame comes out of the pipeline
            latency_ms = 0.0
            try:
                start_time = self._timestamp_queue.popleft()
                latency_ms = (time.perf_counter() - start_time) * 1000.0
            except IndexError:
                pass

            buf = sample.get_buffer()
            result, map_info = buf.map(Gst.MapFlags.READ)
            if result:
                try:
                    nal_bytes = bytes(map_info.data)
                    header = struct.pack('dH', time.time(), min(int(latency_ms), 65535))
                    self._zpub.put(header + nal_bytes)
                    with self._stats_lock:
                        self._tx_bytes        += len(nal_bytes)
                        self._encode_ms_sum   += latency_ms
                        self._encode_ms_count += 1
                except Exception as e:
                    self.get_logger().warning(f'Zenoh publication failed: {e}', throttle_duration_sec=3)
                finally:
                    buf.unmap(map_info)

            now = time.time()
            with self._stats_lock:
                dt  = now - self._stats_ts
                if dt >= 1.0:
                    bw_mbps   = round(self._tx_bytes * 8 / (dt * 1e6), 3)
                    encode_ms = round(self._encode_ms_sum / self._encode_ms_count, 2) \
                                if self._encode_ms_count > 0 else 0.0
                    self._tx_bytes        = 0
                    self._encode_ms_sum   = 0.0
                    self._encode_ms_count = 0
                    self._stats_ts        = now
                else:
                    bw_mbps = None
            if bw_mbps is not None:
                try:
                    self._vstats_pub.put(json.dumps({'bw_mbps': bw_mbps, 'encode_ms': encode_ms}))
                except Exception as e:
                    self.get_logger().warning(f'video_stats publish failed: {e}', throttle_duration_sec=3)

        return Gst.FlowReturn.OK

    def _extract_raw_frame(self, msg: Image):
        enc = msg.encoding
        try:
            if enc == 'bgr8':
                return np.frombuffer(msg.data, np.uint8), "BGR"
            elif enc == 'rgb8':
                return np.frombuffer(msg.data, np.uint8), "RGB"
            elif enc == 'rgba8':
                return np.frombuffer(msg.data, np.uint8), "RGBA"
            elif enc == 'bgra8':
                return np.frombuffer(msg.data, np.uint8), "BGRA"
            elif enc == 'mono8':
                return np.frombuffer(msg.data, np.uint8), "GRAY8"
            else:
                self.get_logger().error(f'Unsupported encoding: {enc}', throttle_duration_sec=5)
                return None, None
        except Exception as e:
            self.get_logger().error(f'Image parsing failed ({enc}): {e}', throttle_duration_sec=5)
            return None, None

    def destroy_node(self):
        self.get_logger().info('GStreamer pipeline shutting down...')
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        self._zpub.undeclare()
        self._vstats_pub.undeclare()
        self._zsession.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = ZenohVideoEncoder()
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