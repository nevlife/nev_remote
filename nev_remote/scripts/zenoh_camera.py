#!/usr/bin/env python3
"""
ROS2 카메라 토픽 → JPEG 압축 → Zenoh 발행

webrtc_camera.py와 달리 asyncio 스레드 불필요.
zenoh pub.put()은 rclpy 콜백에서 직접 호출 가능.

Parameters
----------
image_topic   : str   = /camera/image_raw
zenoh_locator : str   = tcp/127.0.0.1:7447
jpeg_quality  : int   = 80
max_fps       : float = 30.0
"""
import json
import time

import cv2
import numpy as np
import zenoh
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ZenohCamera(Node):
    def __init__(self):
        super().__init__('zenoh_camera')

        self.declare_parameter('image_topic',   '/camera/image_raw')
        self.declare_parameter('zenoh_locator', 'tcp/127.0.0.1:7447')
        self.declare_parameter('jpeg_quality',  80)
        self.declare_parameter('max_fps',       30.0)

        topic   = self.get_parameter('image_topic').value
        locator = self.get_parameter('zenoh_locator').value
        self._quality  = self.get_parameter('jpeg_quality').value
        self._interval = 1.0 / max(1.0, self.get_parameter('max_fps').value)
        self._last_ts  = 0.0

        # ── Zenoh ─────────────────────────────────────────────────────────────
        conf = zenoh.Config()
        if locator:
            conf.insert_json5('connect/endpoints', json.dumps([locator]))
        self._zsession = zenoh.open(conf)
        # 영상은 DROP 우선: 네트워크 막혀도 최신 프레임 우선
        self._zpub = self._zsession.declare_publisher(
            'nev/vehicle/camera',
            congestion_control=zenoh.CongestionControl.DROP,
        )

        # ── ROS2 ──────────────────────────────────────────────────────────────
        self._sub = self.create_subscription(Image, topic, self._cb, 1)

        self.get_logger().info(
            f'zenoh_camera 시작 | topic={topic} | locator={locator} | '
            f'quality={self._quality} | max_fps={1.0/self._interval:.0f}'
        )

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _cb(self, msg: Image):
        now = time.monotonic()
        if now - self._last_ts < self._interval:
            return

        jpeg = self._encode(msg)
        if jpeg is None:
            return

        self._last_ts = now
        try:
            self._zpub.put(jpeg)
        except Exception as e:
            self.get_logger().warning(f'zenoh put 실패: {e}', throttle_duration_sec=3)

    def _encode(self, msg: Image) -> bytes | None:
        bgr = self._to_bgr(msg)
        if bgr is None:
            return None
        ok, buf = cv2.imencode(
            '.jpg', bgr,
            [cv2.IMWRITE_JPEG_QUALITY, self._quality],
        )
        return buf.tobytes() if ok else None

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
            if enc == 'bgra8':
                arr = np.frombuffer(msg.data, np.uint8).reshape(h, w, 4)
                return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
            if enc == 'mono8':
                arr = np.frombuffer(msg.data, np.uint8).reshape(h, w)
                return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
            if enc == 'mono16':
                arr = np.frombuffer(msg.data, np.uint16).reshape(h, w)
                return cv2.cvtColor((arr >> 8).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        except Exception as e:
            self.get_logger().error(f'인코딩 변환 실패 ({enc}): {e}')
            return None

        self.get_logger().warn(
            f'지원하지 않는 인코딩: {enc}', throttle_duration_sec=10
        )
        return None

    # ── 종료 ──────────────────────────────────────────────────────────────────

    def destroy_node(self):
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
