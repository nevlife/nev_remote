#!/usr/bin/env python3
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.node import Node
import rclpy
from gi.repository import Gst
import array
import json
import os
import time
import threading
import numpy as np
import cv2
import gi
gi.require_version('Gst', '1.0')

WIDTH, HEIGHT, FPS = 1280, 720, 15
FOCAL_SCALE = 1.0

CAM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

CAMERAS = [
    ('/dev/cam_front_left',  '/camera/front_left/image_raw',  'cam_front_left'),
    ('/dev/cam_side_left',   '/camera/side_left/image_raw',   'cam_side_left'),
    ('/dev/cam_rear',        '/camera/rear/image_raw',        'cam_rear'),
    ('/dev/cam_front_right', '/camera/front_right/image_raw', 'cam_front_right'),
    ('/dev/cam_side_right',  '/camera/side_right/image_raw',  'cam_side_right'),
]

CALIB_JSON = os.path.expanduser(
    "~/Downloads/ELP-USB16MP01-BL180-2048x1536/ELP-USB16MP01-BL180-2048x1536_calibration.json"
)


def build_undistort_maps(calib_w, calib_h, out_w, out_h):
    with open(CALIB_JSON) as f:
        calib = json.load(f)["ELP-USB16MP01-BL180-2048x1536"]["Intrinsic"]

    K = np.array(calib["K"]).reshape(3, 3)
    D = np.array(calib["D"][1:]).reshape(-1, 1)

    sx = out_w / calib_w
    sy = out_h / calib_h
    K_scaled = K.copy()
    K_scaled[0, 0] *= sx
    K_scaled[0, 2] *= sx
    K_scaled[1, 1] *= sy
    K_scaled[1, 2] *= sy

    K_new = K_scaled.copy()
    K_new[0, 0] *= FOCAL_SCALE
    K_new[1, 1] *= FOCAL_SCALE

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K_scaled, D, np.eye(3), K_new, (out_w, out_h), cv2.CV_16SC2
    )
    return map1, map2


class CamStream:
    def __init__(self, node: Node, device: str, topic: str, frame_id: str,
                 map1: np.ndarray, map2: np.ndarray):
        self._node = node
        self._frame_id = frame_id
        self._running = True
        self._pub = node.create_publisher(Image, topic, CAM_QOS)
        self._map1 = map1
        self._map2 = map2

        pipeline_str = (
            f'v4l2src device={device} ! '
            f'image/jpeg,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! '
            f'jpegdec ! videoconvert ! '
            f'video/x-raw,format=BGR ! '
            f'appsink name=sink emit-signals=false drop=true max-buffers=1 sync=false'
        )
        self._pipeline = Gst.parse_launch(pipeline_str)
        self._sink = self._pipeline.get_by_name('sink')
        self._pipeline.set_state(Gst.State.PLAYING)

        bus = self._pipeline.get_bus()
        msg = bus.timed_pop_filtered(
            5 * Gst.SECOND,
            Gst.MessageType.STATE_CHANGED | Gst.MessageType.ERROR,
        )
        if msg and msg.type == Gst.MessageType.ERROR:
            err, _ = msg.parse_error()
            node.get_logger().fatal(f'[{frame_id}] Failed to open {device}: {err.message}')
            raise SystemExit(1)

        node.get_logger().info(
            f'[{frame_id}] Opened {device} — {WIDTH}x{HEIGHT}@{FPS}fps (undistort ON)')
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        count = 0
        log_time = time.monotonic()

        while self._running:
            t0 = time.perf_counter()
            sample = self._sink.emit('pull-sample')
            t1 = time.perf_counter()

            if sample is None:
                continue

            buf = sample.get_buffer()
            ok, map_info = buf.map(Gst.MapFlags.READ)
            if not ok:
                continue

            frame = np.ndarray(
                (HEIGHT, WIDTH, 3), dtype=np.uint8, buffer=map_info.data
            )

            t2 = time.perf_counter()
            undistorted = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)
            t3 = time.perf_counter()

            msg = Image()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            msg.height = HEIGHT
            msg.width = WIDTH
            msg.encoding = 'bgr8'
            msg.step = WIDTH * 3
            msg.data = array.array('B', undistorted.tobytes())
            t4 = time.perf_counter()

            self._pub.publish(msg)
            t5 = time.perf_counter()

            buf.unmap(map_info)
            count += 1

            now = time.monotonic()
            if now - log_time >= 5.0:
                self._node.get_logger().info(
                    f'[{self._frame_id}] pull={_ms(t0,t1)} remap={_ms(t2,t3)} '
                    f'msg={_ms(t3,t4)} pub={_ms(t4,t5)} total={_ms(t0,t5)} '
                    f'fps={count/(now-log_time):.1f}'
                )
                count = 0
                log_time = now

    def stop(self):
        self._running = False
        self._pipeline.set_state(Gst.State.NULL)


class MultiCamPub(Node):
    def __init__(self):
        super().__init__('test_multi_cam_pub')
        Gst.init(None)

        self.get_logger().info('Building fisheye undistort maps...')
        map1, map2 = build_undistort_maps(2048, 1536, WIDTH, HEIGHT)
        self.get_logger().info('Undistort maps ready')

        self._streams = []
        for device, topic, frame_id in CAMERAS:
            stream = CamStream(self, device, topic, frame_id, map1, map2)
            self._streams.append(stream)

        self.get_logger().info(f'All {len(self._streams)} cameras started')

    def destroy_node(self):
        for s in self._streams:
            s.stop()
        super().destroy_node()


def _ms(a, b):
    return f'{(b - a) * 1000:.1f}ms'


def main():
    rclpy.init()
    node = MultiCamPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
