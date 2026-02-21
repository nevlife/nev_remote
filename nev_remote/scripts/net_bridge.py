#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from hunter_msgs.msg import HunterStatus
from nev_remote_msgs.msg import (EStopStatus, CmdMode, NetworkStatus, MuxStatus)
from system_monitor_msgs.msg import (CpuMetrics, MemoryMetrics, DiskMetrics, NetworkMetrics, GpuMetrics)

import socket
import struct
import time

class NetBridge(Node):
    def __init__(self):
        super().__init__('net_bridge')
        self.declare_parameter('server_ip', '127.0.0.1')
        self.declare_parameter('server_port', 5000)
        self.declare_parameter('local_port', 5000)
        self.declare_parameter('heartbeat_timeout', 2.0)
        self.declare_parameter('control_timeout', 1.0)
        self.declare_parameter('vehicle_rate', 20.0)
        self.declare_parameter('resource_rate', 1.0)

        self.server_ip = self.get_parameter('server_ip').value
        self.server_port = self.get_parameter('server_port').value
        self.local_port = self.get_parameter('local_port').value
        self.hb_timeout = self.get_parameter('heartbeat_timeout').value
        self.ctrl_timeout = self.get_parameter('control_timeout').value
        v_rate = self.get_parameter('vehicle_rate').value
        r_rate = self.get_parameter('resource_rate').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.local_port))
        self.sock.setblocking(False)
        self.server_addr = (self.server_ip, self.server_port)

        self.send_seq = 0
        self.last_hb_time = 0.0
        self.last_ctrl_time = 0.0
        self.rtt_ms = 0.0
        self.socket_ok = True
        self.bridge_flag = 0
        self.server_estop = False

        self.mux_status = MuxStatus()
        self.estop_status = EStopStatus()
        self.last_nav = Twist()
        self.last_teleop = Twist()
        self.last_final = Twist()
        self.hunter_status = None
        self.current_mode = -1

        self.cpu_metrics: CpuMetrics = None
        self.mem_metrics: MemoryMetrics = None
        self.disk_metrics: DiskMetrics = None
        self.net_metrics: NetworkMetrics = None
        self.gpu_metrics: GpuMetrics = None

        self.create_subscription(MuxStatus, '/vehicle/mux_status', self.mux_status_callback, 10)
        self.create_subscription(EStopStatus, '/vehicle/estop_status', self.estop_status_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10) 
        self.create_subscription(Twist, '/remote/teleop_cmd', self.teleop_cmd_callback, 10)
        self.create_subscription(Twist, '/final_cmd', self.final_cmd_callback, 10)
        self.create_subscription(HunterStatus, '/hunter_status', self.hunter_status_callback, 10)

        self.create_subscription(CpuMetrics, '/system_monitor/cpu', self.cpu_callback, 10)
        self.create_subscription(MemoryMetrics, '/system_monitor/memory', self.mem_callback, 10)
        self.create_subscription(DiskMetrics, '/system_monitor/disk', self.disk_callback, 10)
        self.create_subscription(NetworkMetrics, '/system_monitor/network', self.net_callback, 10)
        self.create_subscription(GpuMetrics, '/system_monitor/gpu', self.gpu_callback, 10)

        self.teleop_cmd_pub = self.create_publisher(Twist, '/remote/teleop_cmd', 10)
        self.estop_status_pub = self.create_publisher(EStopStatus, '/remote/estop_status', 10)
        self.cmd_mode_pub = self.create_publisher(CmdMode, '/remote/cmd_mode', 10)
        self.network_status = self.create_publisher(NetworkStatus, '/remote/network_status', 10)

        self.create_timer(0.02, self.recv_tick)
        self.create_timer(1.0 / v_rate, self.send_vehicle)
        self.create_timer(1.0 / r_rate, self.send_resources)
        self.create_timer(0.2, self.check_heartbeat)

        self.get_logger().info(f'netbridge started: listen={self.local_port}, server={self.server_addr}')

    def mux_status_callback(self, msg: MuxStatus): self.mux_status = msg
    def estop_status_callback(self, msg: EStopStatus): self.estop_status = msg
    def cmd_vel_callback(self, msg: Twist): self.last_nav = msg
    def teleop_cmd_callback(self, msg: Twist): self.last_teleop = msg
    def final_cmd_callback(self, msg: Twist): self.last_final = msg
    def hunter_status_callback(self, msg: HunterStatus): self.hunter_status = msg

    def cpu_callback(self, msg: CpuMetrics): self.cpu_metrics = msg
    def mem_callback(self, msg: MemoryMetrics): self.mem_metrics = msg
    def disk_callback(self, msg: DiskMetrics): self.disk_metrics = msg
    def net_callback(self, msg: NetworkMetrics): self.net_metrics = msg
    def gpu_callback(self, msg: GpuMetrics): self.gpu_metrics = msg

    def recv_tick(self):
        for retry_index in range(50):
            try:
                data, addr = self.sock.recvfrom(1024)
            except BlockingIOError:
                break
            except OSError as e:
                self.socket_ok = False
                self.get_logger().warning(f'Socket recv error: {e}')
                break

            if len(data) < 2: continue
            self.socket_ok = True
            header = data[:2]

            try:
                if header == b'TC': self.handle_teleop_control(data)
                elif header == b'HB': self.handle_heartbeat(data)
                elif header == b'ES': self.handle_estop(data)
                elif header == b'CM': self.handle_cmd_mode(data)
            except struct.error as e:
                self.get_logger().warning(f'Packet parse error ({header}): {e}')

    def handle_teleop_control(self, data: bytes):
        packet_type, linear_x, angular_z, seq = struct.unpack('<2sffH', data[:12])
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear_x), float(angular_z)
        self.teleop_cmd_pub.publish(msg)
        self.last_ctrl_time = time.monotonic()

    def handle_heartbeat(self, data: bytes):
        packet_type, timestamp, seq = struct.unpack('<2sdH', data[:12])
        now = time.time()
        self.rtt_ms = max(0.0, (now - timestamp) * 1000.0)
        self.last_hb_time = time.monotonic()

    def handle_estop(self, data: bytes):
        packet_type, command, seq = struct.unpack('<2sBH', data[:5])
        self.server_estop = (command == 1)
        self.publish_estop()

    def handle_cmd_mode(self, data: bytes):
        packet_type, mode, seq = struct.unpack('<2sbH', data[:5])
        self.current_mode = mode
        msg = CmdMode(mode=int(mode))
        self.cmd_mode_pub.publish(msg)

    def check_heartbeat(self):
        now = time.monotonic()
        old_flag = self.bridge_flag

        if self.server_estop: 
            self.bridge_flag = 1 # estop by server command
        elif not self.socket_ok: 
            self.bridge_flag = 2 # socket error state
        elif self.last_hb_time > 0 and (now - self.last_hb_time) > self.hb_timeout:
            self.bridge_flag = 3 # heartbeat timeout
        elif self.current_mode == 2 and self.last_ctrl_time > 0 and (now - self.last_ctrl_time) > self.ctrl_timeout:
            self.bridge_flag = 4 # teleop control timeout
        else:
            self.bridge_flag = 0

        if self.bridge_flag != old_flag:
            self.publish_estop()

        net_msg = NetworkStatus()
        if self.bridge_flag == 0 and self.last_hb_time > 0:
            net_msg.connected = True
        else:
            net_msg.connected = False

        if self.bridge_flag == 0:
            net_msg.status_code = 0 # normal
        elif self.bridge_flag == 3:
            net_msg.status_code = 1 # heartbeat timeout
        else:
            net_msg.status_code = 2 # etc

        net_msg.rtt_ms = float(self.rtt_ms)
        net_msg.bandwidth_mbps = 0.0
        self.network_status.publish(net_msg)

    def publish_estop(self):
        msg = EStopStatus(is_estop=(self.bridge_flag != 0), bridge_flag=int(self.bridge_flag), mux_flag=0)
        self.estop_status_pub.publish(msg)

    def next_seq(self):
        seq = self.send_seq
        self.send_seq = (self.send_seq + 1) % 65536
        return seq

    def send_vehicle(self):
        if not self.socket_ok: return
        try:
            self.send_mux_status()
            self.send_twist_vales()
            self.send_network_status()
            self.send_hunter_status()
            self.send_estop_status()
            self.send_remote_enabled()
        except OSError as e:
            self.socket_ok = False
            self.get_logger().warning(f'Socket send error: {e}')

    def send_mux_status(self):
        ms = self.mux_status
        pkt = struct.pack('<2sbbbbbbBH', b'MS', 
                          int(ms.mode), 
                          int(ms.cmd_source),
                          int(ms.remote_status), 
                          int(ms.nav_active), 
                          int(ms.teleop_active), 
                          int(ms.final_active), 
                          0, 
                          self.next_seq())
        
        self.sock.sendto(pkt, self.server_addr)

    def send_twist_vales(self):
        pkt = struct.pack('<2sffffffH', b'TV',
                          float(self.last_nav.linear.x), 
                          float(self.last_nav.angular.z),
                          float(self.last_teleop.linear.x),
                          float(self.last_teleop.angular.z),
                          float(self.last_final.linear.x),
                          float(self.last_final.angular.z),
                          self.next_seq())
        
        self.sock.sendto(pkt, self.server_addr)

    def send_network_status(self):
        if self.bridge_flag == 0 and self.last_hb_time > 0:
            connected = True
        else:
            connected = False

        if self.bridge_flag == 0:
            status = 0
        elif self.bridge_flag == 3:
            status = 1
        else:
            status = 2

        pkt = struct.pack('<2sBbffH', b'NS', 
                          int(connected), 
                          int(status), 
                          float(self.rtt_ms), 
                          0.0, 
                          self.next_seq())
        
        self.sock.sendto(pkt, self.server_addr)

    def send_hunter_status(self):
        hs = self.hunter_status
        if hs is None: return

        pkt = struct.pack('<2sddBBHdH', b'HS', 
                          float(hs.linear_velocity), 
                          float(hs.steering_angle),
                          int(hs.vehicle_state), 
                          int(hs.control_mode), 
                          int(hs.error_code), 
                          float(hs.battery_voltage), 
                          self.next_seq())
        
        self.sock.sendto(pkt, self.server_addr)

    def send_estop_status(self):
        es = self.estop_status
        pkt = struct.pack('<2sBbbH', b'EP', 
                          int(es.is_estop), 
                          int(es.bridge_flag), 
                          int(es.mux_flag), 
                          self.next_seq())
        
        self.sock.sendto(pkt, self.server_addr)

    def send_remote_enabled(self):
        pkt = struct.pack('<2sBH', b'RE', 
                          int(self.mux_status.remote_status), 
                          self.next_seq())
        
        self.sock.sendto(pkt, self.server_addr)

    def send_resources(self):
        if not self.socket_ok: return

        try:
            if self.cpu_metrics: self.send_cpu()
            if self.mem_metrics: self.send_mem()
            if self.disk_metrics: self.send_disk()
            if self.net_metrics: self.send_net()
            if self.gpu_metrics: self.send_gpu()
        except OSError as e:
            self.socket_ok = False
            self.get_logger().warning(f'Socket send error (Resources): {e}')

    def send_cpu(self):
        c = self.cpu_metrics
        pkt = struct.pack('<2sffffffqH', b'CR',
                          c.usage_percent, 
                          c.freq_current_mhz, 
                          c.temperature_celsius,
                          c.load_avg_1m,
                          c.load_avg_5m, 
                          c.load_avg_15m,
                          c.ctx_switches, 
                          self.next_seq())
        
        self.sock.sendto(pkt, self.server_addr)

    def send_mem(self):
        m = self.mem_metrics
        pkt = struct.pack('<2sqqqqfH', b'MR',
                          m.total_bytes,
                          m.available_bytes, 
                          m.used_bytes,
                          m.free_bytes, 
                          m.percent, 
                          self.next_seq())
        
        self.sock.sendto(pkt, self.server_addr)

    def send_disk(self):
        d = self.disk_metrics
        pkt = struct.pack('<2sqqqqqqqH', b'DI',
                          d.io_read_count,
                          d.io_write_count,
                          d.io_read_bytes,
                          d.io_write_bytes,
                          d.io_read_time_ms,
                          d.io_write_time_ms,
                          d.io_busy_time_ms,
                          self.next_seq())
        self.sock.sendto(pkt, self.server_addr)

        for i, p in enumerate(d.partitions):
            mp = p.mountpoint.encode('utf-8')[:32].ljust(32, b'\x00')
            pkt = struct.pack('<2sB32sqqqfBH', b'DP',
                              i, 
                              mp, 
                              p.total_bytes, 
                              p.used_bytes,
                              p.free_bytes, 
                              p.percent, 
                              int(p.accessible),
                              self.next_seq())
            self.sock.sendto(pkt, self.server_addr)

    def send_net(self):
        n = self.net_metrics
        pkt = struct.pack('<2siiiH', b'NM',
                          n.total_interfaces, 
                          n.active_interfaces, 
                          n.down_interfaces,
                          self.next_seq())
        self.sock.sendto(pkt, self.server_addr)

        for i, iface in enumerate(n.interfaces):
            name = iface.name.encode('utf-8')[:16].ljust(16, b'\x00')
            pkt = struct.pack('<2sB16sBiiddqqqqqqqqH', b'NF',
                              i, 
                              name, 
                              int(iface.is_up), 
                              iface.mtu, 
                              iface.speed_mbps,
                              iface.input_bytes_per_sec, 
                              iface.output_bytes_per_sec,
                              iface.rx_bytes, 
                              iface.tx_bytes,
                              iface.rx_packets, 
                              iface.tx_packets,
                              iface.rx_errors, 
                              iface.tx_errors,
                              iface.rx_dropped, 
                              iface.tx_dropped,
                              self.next_seq())
            self.sock.sendto(pkt, self.server_addr)

    def send_gpu(self):
        for g in self.gpu_metrics.gpus:
            pkt = struct.pack('<2sifffffH', b'GR',
                              g.index, 
                              g.utilization_percent, 
                              g.memory_used_mb,
                              g.memory_total_mb, 
                              g.temperature_celsius, 
                              g.power_watts,
                              self.next_seq())
            self.sock.sendto(pkt, self.server_addr)


    def destroy_node(self):
        self.sock.close()
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