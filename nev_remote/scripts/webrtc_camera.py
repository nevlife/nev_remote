#!/usr/bin/env python3
"""
ROS2 node: subscribes to a camera image topic, compresses each frame to JPEG,
and streams it to the GCS via WebSocket.

Parameters
----------
image_topic  : str   = /camera/image_raw   ROS2 image topic to subscribe
gcs_url      : str   = ws://127.0.0.1:8080/ws/vehicle  GCS WebSocket URL
jpeg_quality : int   = 80                  JPEG quality (1-100)
max_fps      : float = 30.0                Maximum send rate (frames/sec)
"""
import asyncio
import threading
import time

import cv2
import numpy as np
import rclpy
import websockets
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraStreamer(Node):
    def __init__(self):
        super().__init__("camera_streamer")

        self.declare_parameter("image_topic",  "/camera/image_raw")
        self.declare_parameter("gcs_url",      "ws://127.0.0.1:8080/ws/vehicle")
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("max_fps",      30.0)

        self._topic    = self.get_parameter("image_topic").value
        self._gcs_url  = self.get_parameter("gcs_url").value
        self._quality  = self.get_parameter("jpeg_quality").value
        self._interval = 1.0 / max(1.0, self.get_parameter("max_fps").value)

        self._last_ts:    float = 0.0
        self._loop:       asyncio.AbstractEventLoop | None = None
        self._queue:      asyncio.Queue | None = None
        self._loop_ready = threading.Event()

        self._thread = threading.Thread(target=self._run_asyncio, daemon=True)
        self._thread.start()
        self._loop_ready.wait(timeout=5.0)

        self._sub = self.create_subscription(Image, self._topic, self._cb, 1)
        self.get_logger().info(
            f"camera_streamer ready | topic={self._topic} | gcs={self._gcs_url}"
        )

    # ── asyncio thread ────────────────────────────────────────────────────────

    def _run_asyncio(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._queue = asyncio.Queue(maxsize=2)
        self._loop_ready.set()
        self._loop.run_until_complete(self._stream_forever())

    async def _stream_forever(self) -> None:
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to GCS: {self._gcs_url}")
                async with websockets.connect(
                    self._gcs_url,
                    ping_interval=10,
                    ping_timeout=5,
                    close_timeout=2,
                ) as ws:
                    self.get_logger().info("WebSocket connected to GCS")
                    while True:
                        frame = await self._queue.get()
                        await ws.send(frame)
            except Exception as exc:
                self.get_logger().warn(f"WebSocket error: {exc} — retrying in 2 s")
                await asyncio.sleep(2)

    # ── ROS callback (main thread) ────────────────────────────────────────────

    def _cb(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_ts < self._interval:
            return
        if not self._loop_ready.is_set():
            return

        jpeg = self._encode(msg)
        if jpeg is None:
            return

        self._last_ts = now

        # Non-blocking put; drop frame if queue is full (latency > throughput)
        try:
            self._loop.call_soon_threadsafe(self._queue.put_nowait, jpeg)
        except asyncio.QueueFull:
            pass
        except RuntimeError:
            pass

    def _encode(self, msg: Image) -> bytes | None:
        try:
            bgr = self._to_bgr(msg)
            if bgr is None:
                return None
            ok, buf = cv2.imencode(
                ".jpg", bgr,
                [cv2.IMWRITE_JPEG_QUALITY, self._quality]
            )
            return buf.tobytes() if ok else None
        except Exception as exc:
            self.get_logger().error(f"Encode error: {exc}")
            return None

    def _to_bgr(self, msg: Image) -> np.ndarray | None:
        enc = msg.encoding
        h, w = msg.height, msg.width

        if enc == "rgb8":
            arr = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if enc == "bgr8":
            return np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        if enc == "rgba8":
            arr = np.frombuffer(msg.data, np.uint8).reshape(h, w, 4)
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        if enc == "bgra8":
            arr = np.frombuffer(msg.data, np.uint8).reshape(h, w, 4)
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        if enc == "mono8":
            arr = np.frombuffer(msg.data, np.uint8).reshape(h, w)
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        if enc == "mono16":
            arr = np.frombuffer(msg.data, np.uint16).reshape(h, w)
            arr8 = (arr >> 8).astype(np.uint8)
            return cv2.cvtColor(arr8, cv2.COLOR_GRAY2BGR)

        self.get_logger().warn(f"Unsupported encoding: {enc}", throttle_duration_sec=10)
        return None


def main():
    rclpy.init()
    node = CameraStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
