#!/usr/bin/env python3
import array
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

WIDTH, HEIGHT = 1280, 720
FPS = 15

CAM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# 왼→오 순서로 이어붙일 토픽
SRC_TOPICS = [
    '/camera/front_left/image_raw',
    '/camera/side_left/image_raw',
    '/camera/rear/image_raw',
    '/camera/front_right/image_raw',
    '/camera/side_right/image_raw',
]

DST_TOPIC = '/camera/camera/color/image_raw'


class StitchCamPub(Node):
    def __init__(self):
        super().__init__('test_stitch_cam_pub')

        self._num = len(SRC_TOPICS)
        self._lock = threading.Lock()
        self._frames = [None] * self._num
        self._running = True

        for i, topic in enumerate(SRC_TOPICS):
            self.create_subscription(Image, topic, lambda msg, idx=i: self._on_image(idx, msg), CAM_QOS)
            self.get_logger().info(f'Subscribed: {topic}')

        self._pub = self.create_publisher(Image, DST_TOPIC, CAM_QOS)

        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f'Publishing stitched image to {DST_TOPIC}')

    def _on_image(self, idx, msg: Image):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        frame = np.ascontiguousarray(frame[::-1, ::-1])
        with self._lock:
            self._frames[idx] = frame

    def _publish_loop(self):
        count = 0
        log_time = time.monotonic()
        stitch_w = WIDTH * self._num

        while self._running:
            t0 = time.perf_counter()

            with self._lock:
                frames = list(self._frames)

            if any(f is None for f in frames):
                time.sleep(0.01)
                continue

            t1 = time.perf_counter()
            stitched = np.hstack(frames)
            t2 = time.perf_counter()

            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_stitched'
            msg.height = HEIGHT
            msg.width = stitch_w
            msg.encoding = 'bgr8'
            msg.step = stitch_w * 3
            msg.data = array.array('B', stitched.tobytes())
            t3 = time.perf_counter()

            self._pub.publish(msg)
            t4 = time.perf_counter()

            count += 1
            now = time.monotonic()
            if now - log_time >= 5.0:
                self.get_logger().info(
                    f'[stitch] grab={_ms(t0,t1)} hstack={_ms(t1,t2)} '
                    f'msg={_ms(t2,t3)} pub={_ms(t3,t4)} total={_ms(t0,t4)} '
                    f'fps={count/(now-log_time):.1f}  size={stitch_w}x{HEIGHT}'
                )
                count = 0
                log_time = now

            elapsed = time.perf_counter() - t0
            sleep_time = (1.0 / FPS) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def _ms(a, b):
    return f'{(b - a) * 1000:.1f}ms'


def main():
    rclpy.init()
    node = StitchCamPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
