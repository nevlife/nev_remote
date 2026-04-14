#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402
from nev_teleop_bot_msgs.msg import EStopStatus, CmdMode, MuxStatus  # noqa: E402
from system_monitor_msgs.msg import (  # noqa: E402
    CpuMetrics,
    MemoryMetrics,
    DiskMetrics,
    NetworkMetrics,
    GpuMetrics,
)

from net_bridge import (  # noqa: E402
    ZenohTransport,
    InboundHandler,
    HealthMonitor,
    TelemetrySerializer,
)


class NetBridge(Node):
    def __init__(self):
        super().__init__("net_bridge")

        self.declare_parameter("telemetry_locator", "tcp/127.0.0.1:7447")
        self.declare_parameter("vehicle_id", "0")
        self.declare_parameter("heartbeat_timeout", 2.0)
        self.declare_parameter("control_timeout", 1.0)
        self.declare_parameter("vehicle_rate", 20.0)
        self.declare_parameter("resource_rate", 1.0)
        self.declare_parameter("wheelbase", 0.650)
        self.declare_parameter("speed_topic", "/vehicle/speed")
        self.declare_parameter("angle_topic", "/vehicle/steer_angle")

        locator = self.get_parameter("telemetry_locator").value
        vid = self.get_parameter("vehicle_id").value
        hb_timeout = self.get_parameter("heartbeat_timeout").value
        ctrl_timeout = self.get_parameter("control_timeout").value
        v_rate = self.get_parameter("vehicle_rate").value
        r_rate = self.get_parameter("resource_rate").value
        wheelbase = self.get_parameter("wheelbase").value
        speed_topic = self.get_parameter("speed_topic").value
        angle_topic = self.get_parameter("angle_topic").value

        self._vid = vid
        self.server_estop = False
        self.current_mode = -1

        logger = self.get_logger()
        self.transport = ZenohTransport(locator, logger)
        self.inbound = InboundHandler(vid, self.transport, logger, wheelbase)
        self.health = HealthMonitor(hb_timeout, ctrl_timeout)

        for suffix in (
            "mux",
            "twist",
            "estop",
            "cpu",
            "mem",
            "gpu",
            "disk",
            "net",
            "server_pong",
            "bot_pong",
        ):
            self.transport.declare_publisher(f"nev/robot/{vid}/{suffix}")

        self.transport.declare_subscriber(
            f"nev/gcs/{vid}/server_ping",
            self.inbound.on_server_ping,
        )
        self.transport.declare_subscriber(
            f"nev/gcs/{vid}/bot_ping",
            self.inbound.on_bot_ping,
        )
        self.transport.declare_subscriber(
            f"nev/gcs/{vid}/teleop", self.inbound.on_teleop
        )
        self.transport.declare_subscriber(
            f"nev/gcs/{vid}/estop", self.inbound.on_estop
        )
        self.transport.declare_subscriber(
            f"nev/gcs/{vid}/cmd_mode", self.inbound.on_cmd_mode
        )

        self.mux_status = MuxStatus()
        self.estop_status = EStopStatus()
        self.last_nav = Twist()
        self.last_teleop = Twist()
        self.last_final = Twist()
        self.current_speed: float = 0.0
        self.current_steer_angle: float = 0.0
        self.cpu_metrics: CpuMetrics | None = None
        self.mem_metrics: MemoryMetrics | None = None
        self.disk_metrics: DiskMetrics | None = None
        self.net_metrics: NetworkMetrics | None = None
        self.gpu_metrics: GpuMetrics | None = None

        self.create_subscription(
            MuxStatus, "/vehicle/mux_status",
            lambda m: setattr(self, "mux_status", m), 10,
        )
        self.create_subscription(
            EStopStatus, "/vehicle/estop_status",
            lambda m: setattr(self, "estop_status", m), 10,
        )
        self.create_subscription(
            Twist, "/cmd_vel",
            lambda m: setattr(self, "last_nav", m), 10,
        )
        self.create_subscription(
            Twist, "/remote/teleop_cmd",
            lambda m: setattr(self, "last_teleop", m), 10,
        )
        self.create_subscription(
            Twist, "/final_cmd",
            lambda m: setattr(self, "last_final", m), 10,
        )
        self.create_subscription(
            Float64, speed_topic,
            lambda m: setattr(self, "current_speed", m.data), 10,
        )
        self.create_subscription(
            Float64, angle_topic,
            lambda m: setattr(self, "current_steer_angle", m.data), 10,
        )
        self.create_subscription(
            CpuMetrics, "/system_monitor/cpu",
            lambda m: setattr(self, "cpu_metrics", m), 10,
        )
        self.create_subscription(
            MemoryMetrics, "/system_monitor/memory",
            lambda m: setattr(self, "mem_metrics", m), 10,
        )
        self.create_subscription(
            DiskMetrics, "/system_monitor/disk",
            lambda m: setattr(self, "disk_metrics", m), 10,
        )
        self.create_subscription(
            NetworkMetrics, "/system_monitor/network",
            lambda m: setattr(self, "net_metrics", m), 10,
        )
        self.create_subscription(
            GpuMetrics, "/system_monitor/gpu",
            lambda m: setattr(self, "gpu_metrics", m), 10,
        )

        self._teleop_pub = self.create_publisher(Twist, "/remote/teleop_cmd", 10)
        self._estop_pub = self.create_publisher(EStopStatus, "/remote/estop_status", 10)
        self._mode_pub = self.create_publisher(CmdMode, "/remote/cmd_mode", 10)
        self.create_timer(0.05, self._process_commands)
        self.create_timer(1.0 / v_rate, self._send_vehicle)
        self.create_timer(1.0 / r_rate, self._send_resources)
        self.create_timer(0.2, self._check_heartbeat)

        logger.info(
            f"net_bridge started (vehicle_id={vid}) "
            f'-> {locator or "auto-discovery"}'
        )
        logger.info(f"speed_topic={speed_topic}, angle_topic={angle_topic}")

    def _process_commands(self):
        cmds = self.inbound.drain_pending()

        if cmds.teleop is not None:
            msg = Twist()
            msg.linear.x, msg.angular.z = cmds.teleop
            self._teleop_pub.publish(msg)

        if cmds.estop is not None:
            self.server_estop = cmds.estop
            self._publish_estop()

        if cmds.mode is not None:
            self.current_mode = cmds.mode
            self._mode_pub.publish(CmdMode(mode=cmds.mode))

    def _check_heartbeat(self):
        state = self.health.evaluate(
            self.inbound.last_hb_time,
            self.inbound.last_ctrl_time,
            self.current_mode,
            self.server_estop,
        )

        if state.flag_changed:
            self._publish_estop(state.bridge_flag)

    def _publish_estop(self, bridge_flag: int | None = None):
        if bridge_flag is None:
            bridge_flag = self.health._prev_flag
        self._estop_pub.publish(
            EStopStatus(
                is_estop=(bridge_flag != 0),
                bridge_flag=int(bridge_flag),
                mux_flag=0,
            )
        )

    def _send_vehicle(self):
        payloads = TelemetrySerializer.serialize_vehicle(
            self._vid,
            self.mux_status,
            self.last_nav,
            self.last_teleop,
            self.last_final,
            self.current_speed,
            self.current_steer_angle,
            self.estop_status,
        )
        for key, data in payloads.items():
            self.transport.put(key, data)

    def _send_resources(self):
        payloads = TelemetrySerializer.serialize_resources(
            self._vid,
            self.cpu_metrics,
            self.mem_metrics,
            self.gpu_metrics,
            self.disk_metrics,
            self.net_metrics,
        )
        for key, data in payloads.items():
            self.transport.put(key, data)

    def destroy_node(self):
        self.transport.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NetBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
