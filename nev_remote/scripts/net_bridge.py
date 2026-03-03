#!/usr/bin/env python3
import json
import threading
import time

import zenoh
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from hunter_msgs.msg import HunterStatus
from nev_remote_msgs.msg import EStopStatus, CmdMode, NetworkStatus, MuxStatus
from system_monitor_msgs.msg import (
    CpuMetrics, MemoryMetrics, DiskMetrics, NetworkMetrics, GpuMetrics,
)


class NetBridge(Node):
    def __init__(self):
        super().__init__('net_bridge')

        self.declare_parameter('zenoh_locator',    'tcp/127.0.0.1:7447')
        self.declare_parameter('heartbeat_timeout', 2.0)
        self.declare_parameter('control_timeout',   1.0)
        self.declare_parameter('vehicle_rate',      20.0)
        self.declare_parameter('resource_rate',     1.0)

        locator       = self.get_parameter('zenoh_locator').value
        self.hb_timeout   = self.get_parameter('heartbeat_timeout').value
        self.ctrl_timeout = self.get_parameter('control_timeout').value
        v_rate        = self.get_parameter('vehicle_rate').value
        r_rate        = self.get_parameter('resource_rate').value

        # ── Zenoh session ─────────────────────────────────────────────────────
        conf = zenoh.Config()
        if locator:
            conf.insert_json5('connect/endpoints', json.dumps([locator]))
        self._zsession = zenoh.open(conf)

        # Publishers (vehicle → GCS)
        self._zpubs = {
            k: self._zsession.declare_publisher(k) for k in [
                'nev/vehicle/mux',
                'nev/vehicle/twist',
                'nev/vehicle/network',
                'nev/vehicle/hunter',
                'nev/vehicle/estop',
                'nev/vehicle/cpu',
                'nev/vehicle/mem',
                'nev/vehicle/gpu',
                'nev/vehicle/disk',
                'nev/vehicle/net',
                'nev/vehicle/hb_ack',
            ]
        }

        # Subscribers (GCS → vehicle) — run in zenoh background thread
        self._zsubs = [
            self._zsession.declare_subscriber('nev/gcs/heartbeat', self._on_heartbeat),
            self._zsession.declare_subscriber('nev/gcs/teleop',    self._on_teleop),
            self._zsession.declare_subscriber('nev/gcs/estop',     self._on_estop),
            self._zsession.declare_subscriber('nev/gcs/cmd_mode',  self._on_cmd_mode),
        ]

        # ── State ─────────────────────────────────────────────────────────────
        self._lock         = threading.Lock()
        self.last_hb_time  = 0.0
        self.last_ctrl_time = 0.0
        self.bridge_flag   = 0
        self.server_estop  = False
        self.current_mode  = -1

        # Pending commands (written by zenoh thread, read by rclpy timer)
        self._pending_teleop: tuple | None = None   # (lx, az)
        self._pending_estop:  bool | None  = None
        self._pending_mode:   int  | None  = None

        # ── Cached ROS messages ───────────────────────────────────────────────
        self.mux_status    = MuxStatus()
        self.estop_status  = EStopStatus()
        self.last_nav      = Twist()
        self.last_teleop   = Twist()
        self.last_final    = Twist()
        self.hunter_status: HunterStatus | None = None
        self.cpu_metrics:   CpuMetrics   | None = None
        self.mem_metrics:   MemoryMetrics | None = None
        self.disk_metrics:  DiskMetrics  | None = None
        self.net_metrics:   NetworkMetrics | None = None
        self.gpu_metrics:   GpuMetrics   | None = None

        # ── ROS subscriptions ─────────────────────────────────────────────────
        self.create_subscription(MuxStatus,    '/vehicle/mux_status',   self.mux_status_callback, 10)
        self.create_subscription(EStopStatus,  '/vehicle/estop_status', self.estop_status_callback, 10)
        self.create_subscription(Twist,        '/cmd_vel',              self.cmd_vel_callback, 10)
        self.create_subscription(Twist,        '/remote/teleop_cmd',    self.teleop_cmd_callback, 10)
        self.create_subscription(Twist,        '/final_cmd',            self.final_cmd_callback, 10)
        self.create_subscription(HunterStatus, '/hunter_status',        self.hunter_status_callback, 10)
        self.create_subscription(CpuMetrics,   '/system_monitor/cpu',   self.cpu_callback, 10)
        self.create_subscription(MemoryMetrics,'/system_monitor/memory',self.mem_callback, 10)
        self.create_subscription(DiskMetrics,  '/system_monitor/disk',  self.disk_callback, 10)
        self.create_subscription(NetworkMetrics,'/system_monitor/network',self.net_callback, 10)
        self.create_subscription(GpuMetrics,   '/system_monitor/gpu',   self.gpu_callback, 10)

        # ── ROS publishers ────────────────────────────────────────────────────
        self.teleop_cmd_pub    = self.create_publisher(Twist,         '/remote/teleop_cmd', 10)
        self.estop_status_pub  = self.create_publisher(EStopStatus,   '/remote/estop_status', 10)
        self.cmd_mode_pub      = self.create_publisher(CmdMode,       '/remote/cmd_mode', 10)
        self.network_status_pub = self.create_publisher(NetworkStatus, '/remote/network_status', 10)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(0.05,          self._process_commands)   # 20 Hz
        self.create_timer(1.0 / v_rate,  self.send_vehicle)
        self.create_timer(1.0 / r_rate,  self.send_resources)
        self.create_timer(0.2,           self.check_heartbeat)

        self.get_logger().info(f'net_bridge (zenoh) started → {locator or "auto-discovery"}')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def mux_status_callback(self, msg):    self.mux_status = msg
    def estop_status_callback(self, msg):  self.estop_status = msg
    def cmd_vel_callback(self, msg):       self.last_nav = msg
    def teleop_cmd_callback(self, msg):    self.last_teleop = msg
    def final_cmd_callback(self, msg):     self.last_final = msg
    def hunter_status_callback(self, msg): self.hunter_status = msg
    def cpu_callback(self, msg):           self.cpu_metrics = msg
    def mem_callback(self, msg):           self.mem_metrics = msg
    def disk_callback(self, msg):          self.disk_metrics = msg
    def net_callback(self, msg):           self.net_metrics = msg
    def gpu_callback(self, msg):           self.gpu_metrics = msg

    # ── Zenoh receive callbacks (zenoh background thread) ──────────────────────

    def _on_heartbeat(self, sample):
        data = json.loads(bytes(sample.payload))
        ts = data.get('ts', 0.0)
        self.last_hb_time = time.monotonic()
        # ts를 그대로 반송 — 서버에서 왕복 시간 계산
        self._zput('nev/vehicle/hb_ack', {'ts': ts})

    def _on_teleop(self, sample):
        data = json.loads(bytes(sample.payload))
        with self._lock:
            self._pending_teleop = (
                float(data.get('linear_x', 0.0)),
                float(data.get('angular_z', 0.0)),
            )

    def _on_estop(self, sample):
        data = json.loads(bytes(sample.payload))
        with self._lock:
            self._pending_estop = bool(data.get('active', False))

    def _on_cmd_mode(self, sample):
        data = json.loads(bytes(sample.payload))
        with self._lock:
            self._pending_mode = int(data.get('mode', -1))

    # ── Command processing (rclpy timer, main thread) ─────────────────────────

    def _process_commands(self):
        with self._lock:
            teleop = self._pending_teleop
            estop  = self._pending_estop
            mode   = self._pending_mode
            self._pending_teleop = None
            self._pending_estop  = None
            self._pending_mode   = None

        if teleop is not None:
            msg = Twist()
            msg.linear.x, msg.angular.z = teleop
            self.teleop_cmd_pub.publish(msg)
            self.last_ctrl_time = time.monotonic()

        if estop is not None:
            self.server_estop = estop
            self.publish_estop()

        if mode is not None:
            self.current_mode = mode
            self.cmd_mode_pub.publish(CmdMode(mode=mode))

    # ── Safety watchdog ───────────────────────────────────────────────────────

    def check_heartbeat(self):
        now      = time.monotonic()
        old_flag = self.bridge_flag

        if self.server_estop:
            self.bridge_flag = 1
        elif self.last_hb_time > 0 and (now - self.last_hb_time) > self.hb_timeout:
            self.bridge_flag = 3
        elif (self.current_mode == 2
              and self.last_ctrl_time > 0
              and (now - self.last_ctrl_time) > self.ctrl_timeout):
            self.bridge_flag = 4
        else:
            self.bridge_flag = 0

        if self.bridge_flag != old_flag:
            self.publish_estop()

        connected   = self.bridge_flag == 0 and self.last_hb_time > 0
        status_code = {0: 0, 3: 1}.get(self.bridge_flag, 2)

        net_msg = NetworkStatus(
            connected=connected,
            status_code=status_code,
        )
        self.network_status_pub.publish(net_msg)

    def publish_estop(self):
        self.estop_status_pub.publish(EStopStatus(
            is_estop=(self.bridge_flag != 0),
            bridge_flag=int(self.bridge_flag),
            mux_flag=0,
        ))

    # ── Zenoh publish helpers ─────────────────────────────────────────────────

    def _zput(self, topic: str, data):
        try:
            self._zpubs[topic].put(json.dumps(data))
        except Exception as e:
            self.get_logger().warning(f'zenoh put [{topic}]: {e}')

    # ── Vehicle state → GCS ───────────────────────────────────────────────────

    def send_vehicle(self):
        ms = self.mux_status
        self._zput('nev/vehicle/mux', {
            'ts':             time.time(),
            'requested_mode': int(ms.mode),
            'active_source':  int(ms.cmd_source),
            'remote_enabled': bool(ms.remote_status),
            'nav_active':     bool(ms.nav_active),
            'teleop_active':  bool(ms.teleop_active),
            'final_active':   bool(ms.final_active),
        })

        self._zput('nev/vehicle/twist', {
            'ts':        time.time(),
            'nav_lx':    float(self.last_nav.linear.x),
            'nav_az':    float(self.last_nav.angular.z),
            'teleop_lx': float(self.last_teleop.linear.x),
            'teleop_az': float(self.last_teleop.angular.z),
            'final_lx':  float(self.last_final.linear.x),
            'final_az':  float(self.last_final.angular.z),
        })

        connected   = self.bridge_flag == 0 and self.last_hb_time > 0
        status_code = {0: 0, 3: 1}.get(self.bridge_flag, 2)
        self._zput('nev/vehicle/network', {
            'connected':   connected,
            'status_code': status_code,
        })

        if self.hunter_status:
            hs = self.hunter_status
            self._zput('nev/vehicle/hunter', {
                'ts':             time.time(),
                'linear_vel':     float(hs.linear_velocity),
                'steering_angle': float(hs.steering_angle),
                'vehicle_state':  int(hs.vehicle_state),
                'control_mode':   int(hs.control_mode),
                'error_code':     int(hs.error_code),
                'battery_voltage':float(hs.battery_voltage),
            })

        es = self.estop_status
        self._zput('nev/vehicle/estop', {
            'ts':         time.time(),
            'is_estop':   bool(es.is_estop),
            'bridge_flag':int(es.bridge_flag),
            'mux_flag':   int(es.mux_flag),
        })

    # ── System resources → GCS ────────────────────────────────────────────────

    def send_resources(self):
        if self.cpu_metrics:
            c = self.cpu_metrics
            self._zput('nev/vehicle/cpu', {
                'cpu_usage': float(c.usage_percent),
                'cpu_temp':  float(c.temperature_celsius),
                'cpu_load':  float(c.load_avg_1m),
            })

        if self.mem_metrics:
            m = self.mem_metrics
            self._zput('nev/vehicle/mem', {
                'ram_total': int(m.total_bytes // (1024 * 1024)),
                'ram_used':  int(m.used_bytes  // (1024 * 1024)),
            })

        if self.gpu_metrics:
            self._zput('nev/vehicle/gpu', [
                {
                    'idx':          int(g.index),
                    'gpu_usage':    float(g.utilization_percent),
                    'gpu_mem_used': float(g.memory_used_mb),
                    'gpu_mem_total':float(g.memory_total_mb),
                    'gpu_temp':     float(g.temperature_celsius),
                    'gpu_power':    float(g.power_watts),
                }
                for g in self.gpu_metrics.gpus
            ])

        if self.disk_metrics:
            d = self.disk_metrics
            self._zput('nev/vehicle/disk', {
                'partitions': [
                    {
                        'idx':        i,
                        'mountpoint': p.mountpoint,
                        'total_bytes':int(p.total_bytes),
                        'used_bytes': int(p.used_bytes),
                        'percent':    float(p.percent),
                        'accessible': bool(p.accessible),
                    }
                    for i, p in enumerate(d.partitions)
                ]
            })

        if self.net_metrics:
            n = self.net_metrics
            self._zput('nev/vehicle/net', {
                'net_total_ifaces':  int(n.total_interfaces),
                'net_active_ifaces': int(n.active_interfaces),
                'net_down_ifaces':   int(n.down_interfaces),
                'interfaces': [
                    {
                        'idx':       i,
                        'name':      iface.name,
                        'is_up':     bool(iface.is_up),
                        'speed_mbps':int(iface.speed_mbps),
                        'in_bps':    float(iface.input_bytes_per_sec),
                        'out_bps':   float(iface.output_bytes_per_sec),
                    }
                    for i, iface in enumerate(n.interfaces)
                ]
            })

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        for sub in self._zsubs:
            sub.undeclare()
        for pub in self._zpubs.values():
            pub.undeclare()
        self._zsession.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NetBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
