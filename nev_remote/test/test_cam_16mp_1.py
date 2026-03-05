#!/usr/bin/env python3
"""
Test: 16MP USB Camera 1 (/dev/video10, usb-0000:00:14.0-6.3)
zenoh_video_encoder.py 와 동일한 파이프라인/Zenoh 포맷 사용
v4l2src 로 직접 캡처 (ROS2 불필요)
"""
import json
import struct
import time
import sys

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import zenoh

# ── 설정 ─────────────────────────────────────────────────────────────────────
DEVICE         = '/dev/video10'
WIDTH          = 1280
HEIGHT         = 720
FPS            = 15
BITRATE        = 500
CONFIG_INT     = -1

ZENOH_KEY      = 'nev/vehicle/camera'
STATS_KEY      = 'nev/vehicle/video_stats'
ZENOH_LOCATOR  = 'tcp/203.250.33.77:80'
TEST_DURATION  = 10
# ─────────────────────────────────────────────────────────────────────────────

class CameraTest:
    def __init__(self):
        Gst.init(None)

        print(f'\n[16mp_1] 16MP Camera 1 테스트  device={DEVICE}  {WIDTH}x{HEIGHT}@{FPS}fps')

        # ── Zenoh ──
        conf = zenoh.Config()
        conf.insert_json5('connect/endpoints', json.dumps([ZENOH_LOCATOR]))
        try:
            self._zsession  = zenoh.open(conf)
            self._zpub      = self._zsession.declare_publisher(
                ZENOH_KEY, congestion_control=zenoh.CongestionControl.DROP)
            self._stats_pub = self._zsession.declare_publisher(STATS_KEY)
            print(f'  Zenoh 연결 OK  ({ZENOH_LOCATOR})')
        except Exception as e:
            print(f'  Zenoh 연결 실패 ({e})  →  파이프라인만 테스트')
            self._zsession = self._zpub = self._stats_pub = None

        # ── GStreamer pipeline ──
        pipeline_str = (
            f'v4l2src device={DEVICE} ! '
            f'image/jpeg,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! '
            f'jpegdec ! '
            f'videoconvert n-threads=4 qos=true ! '
            f'video/x-raw,format=NV12 ! '
            f'nvh265enc '
            f'preset=low-latency-hq rc-mode=cbr-ld-hq '
            f'bitrate={BITRATE} max-bitrate=0 const-quality=0 '
            f'gop-size=15 aud=true qos=false zerolatency=true '
            f'rc-lookahead=0 bframes=0 i-adapt=false b-adapt=false ! '
            f'h265parse config-interval=-1 ! '
            f'video/x-h265,stream-format=byte-stream,alignment=au ! '
            f'appsink name=appsink drop=true max-buffers=2 sync=false'
        )
        print(f'\n  pipeline:\n  {pipeline_str}\n')

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except Exception as e:
            print(f'[오류] 파이프라인 생성 실패: {e}')
            sys.exit(1)

        self.appsink = self.pipeline.get_by_name('appsink')
        self.appsink.set_property('emit-signals', True)
        self.appsink.connect('new-sample', self._on_new_sample, None)

        self._tx_bytes    = 0
        self._pub_ms_sum  = 0.0
        self._pub_count   = 0
        self._stats_ts    = time.time()
        self._frame_count = 0
        self._start_time  = None

    def _on_new_sample(self, sink, _data):
        sample = sink.emit('pull-sample')
        if not isinstance(sample, Gst.Sample):
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        result, map_info = buf.map(Gst.MapFlags.READ)
        if result:
            try:
                nal_bytes = bytes(map_info.data)
                t0 = time.perf_counter()
                if self._zpub:
                    header = struct.pack('dH', time.time(), 0)
                    self._zpub.put(header + nal_bytes)
                pub_ms = (time.perf_counter() - t0) * 1000.0

                self._tx_bytes    += len(nal_bytes)
                self._pub_ms_sum  += pub_ms
                self._pub_count   += 1
                self._frame_count += 1
            except Exception as e:
                print(f'  [경고] Zenoh 전송 실패: {e}')
            finally:
                buf.unmap(map_info)

        now = time.time()
        dt  = now - self._stats_ts
        if dt >= 1.0:
            bw_mbps = round(self._tx_bytes * 8 / (dt * 1e6), 3)
            pub_ms  = round(self._pub_ms_sum / self._pub_count, 2) if self._pub_count > 0 else 0.0
            print(f'  [{now - self._start_time:4.1f}s] '
                  f'frames={self._frame_count}  '
                  f'BW={bw_mbps:.3f}Mbps  '
                  f'pub={pub_ms:.1f}ms')
            if self._stats_pub:
                try:
                    self._stats_pub.put(json.dumps({'bw_mbps': bw_mbps, 'pub_ms': pub_ms}))
                except Exception:
                    pass
            self._tx_bytes = self._pub_ms_sum = self._pub_count = 0
            self._stats_ts = now

        return Gst.FlowReturn.OK

    def run(self):
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print('[오류] 파이프라인 시작 실패 — 카메라 또는 NVENC 확인')
            sys.exit(1)

        self._start_time = time.time()
        print(f'  스트리밍 중... ({TEST_DURATION}초)\n')

        while time.time() - self._start_time < TEST_DURATION:
            time.sleep(0.1)

        elapsed = time.time() - self._start_time
        result  = 'PASS' if self._frame_count > 0 else 'FAIL'
        print(f'\n{"="*50}')
        print(f'  결과      : [{result}]')
        print(f'  총 프레임 : {self._frame_count}')
        print(f'  평균 FPS  : {self._frame_count / elapsed:.2f}')
        print(f'{"="*50}\n')

        self.pipeline.set_state(Gst.State.NULL)
        if self._zpub:      self._zpub.undeclare()
        if self._stats_pub: self._stats_pub.undeclare()
        if self._zsession:  self._zsession.close()


if __name__ == '__main__':
    CameraTest().run()
