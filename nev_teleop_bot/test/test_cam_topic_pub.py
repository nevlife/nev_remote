#!/usr/bin/env python3
import array
import time
import threading
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

WIDTH, HEIGHT, FPS = 1280, 720, 30


class CamTopicPub(Node):
    def __init__(self):
        super().__init__('test_cam_topic_pub')
        Gst.init(None)
        self._pub = self.create_publisher(Image, '/camera/camera/color/image_raw', qos_profile_sensor_data)
        self._running = True

        pipeline_str = (
            f'v4l2src device=/dev/video0 ! '
            f'image/jpeg,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! '
            f'jpegdec ! videoconvert ! '
            f'video/x-raw,format=BGR ! appsink name=sink emit-signals=false drop=true max-buffers=1 sync=false'
        )
        self._pipeline = Gst.parse_launch(pipeline_str)
        self._sink = self._pipeline.get_by_name('sink')
        self._pipeline.set_state(Gst.State.PLAYING)

        bus = self._pipeline.get_bus()
        msg = bus.timed_pop_filtered(5 * Gst.SECOND, Gst.MessageType.STATE_CHANGED | Gst.MessageType.ERROR)
        if msg and msg.type == Gst.MessageType.ERROR:
            err, _ = msg.parse_error()
            self.get_logger().fatal(f'Failed to open camera: {err.message}')
            raise SystemExit(1)

        self.get_logger().info(f'Camera opened: {WIDTH}x{HEIGHT}@{FPS}fps (GStreamer)')
        threading.Thread(target=self._loop, daemon=True).start()

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

            t2 = time.perf_counter()
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            msg.height = HEIGHT
            msg.width = WIDTH
            msg.encoding = 'bgr8'
            msg.step = WIDTH * 3
            msg.data = array.array('B', map_info.data)
            t3 = time.perf_counter()

            self._pub.publish(msg)
            t4 = time.perf_counter()

            buf.unmap(map_info)
            count += 1

            now = time.monotonic()
            if now - log_time >= 2.0:
                self.get_logger().info(
                    f'[{count}] pull={_ms(t0,t1)} map+msg={_ms(t1,t3)} '
                    f'(copy={_ms(t2,t3)}) pub={_ms(t3,t4)} total={_ms(t0,t4)} '
                    f'fps={count/(now-log_time):.1f}'
                )
                count = 0
                log_time = now

    def destroy_node(self):
        self._running = False
        self._pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def _ms(a, b):
    return f'{(b - a) * 1000:.1f}ms'


def main():
    rclpy.init()
    node = CamTopicPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
