#!/usr/bin/env python3
"""
Test server for net_bridge.

Sends heartbeat, teleop control, e-stop, and mode commands to the vehicle.
Receives and displays vehicle status packets.

Usage:
    python3 test_server.py [--vehicle-ip IP] [--vehicle-port PORT] [--listen-port PORT]

Keyboard controls:
    w/s     - increase/decrease linear velocity
    a/d     - increase/decrease angular velocity
    space   - stop (zero velocities)
    e       - toggle e-stop
    0-2     - set mode (0=ctrl_on, 1=nav, 2=remote)
    i       - set mode idle (-1)
    q       - quit
"""
import socket
import struct
import time
import threading
import sys
import argparse
import select
import termios
import tty
import os

# Packet format definitions (vehicle -> server)
PACKET_FORMATS = {
    b'MS': ('<2sbbbbbbBH', [
        'header', 'requested_mode', 'active_source', 'remote_enabled',
        'nav_active', 'teleop_active', 'final_active', 'pad', 'seq']),
    b'TV': ('<2sffffffH', [
        'header', 'nav_lx', 'nav_az', 'teleop_lx', 'teleop_az',
        'final_lx', 'final_az', 'seq']),
    b'NS': ('<2sBbffH', [
        'header', 'connected', 'status_code', 'rtt_ms', 'bandwidth_mbps', 'seq']),
    b'HS': ('<2sddBBHfH', [
        'header', 'linear_vel', 'steering_angle', 'vehicle_state',
        'control_mode', 'error_code', 'battery_voltage', 'seq']),
    b'SR': ('<2shhfffihfihfH', [
        'header', 'cpu_phys', 'cpu_logic', 'cpu_usage', 'cpu_temp',
        'cpu_load', 'ram_total', 'ram_used', 'gpu_usage', 'gpu_mem_total',
        'gpu_mem_used', 'gpu_temp', 'seq']),
    b'EP': ('<2sBbbH', [
        'header', 'is_estop', 'bridge_flag', 'mux_flag', 'seq']),
    b'RE': ('<2sBH', [
        'header', 'enabled', 'seq']),
}


class TestServer:
    def __init__(self, vehicle_ip, vehicle_port, listen_port):
        self.vehicle_addr = (vehicle_ip, vehicle_port)
        self.listen_port = listen_port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', listen_port))
        self.sock.setblocking(False)

        self.seq = 0
        self.running = True

        # Control state
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.estop_active = False
        self.current_mode = -1  # idle

        # Received data for display
        self.last_packets = {}
        self.lock = threading.Lock()

    def next_seq(self):
        s = self.seq
        self.seq = (self.seq + 1) % 65536
        return s

    # --- Send packets ---
    def send_heartbeat(self):
        """HB: <2sdH (12B)"""
        pkt = struct.pack('<2sdH', b'HB', time.time(), self.next_seq())
        self.sock.sendto(pkt, self.vehicle_addr)

    def send_teleop(self):
        """TC: <2sffH (12B)"""
        pkt = struct.pack('<2sffH', b'TC',
                          self.linear_x, self.angular_z, self.next_seq())
        self.sock.sendto(pkt, self.vehicle_addr)

    def send_estop(self, activate: bool):
        """ES: <2sBH (5B)"""
        pkt = struct.pack('<2sBH', b'ES', 1 if activate else 0, self.next_seq())
        self.sock.sendto(pkt, self.vehicle_addr)

    def send_cmd_mode(self, mode: int):
        """CM: <2sbH (5B)"""
        pkt = struct.pack('<2sbH', b'CM', mode, self.next_seq())
        self.sock.sendto(pkt, self.vehicle_addr)

    # --- Receive packets ---
    def recv_loop(self):
        while self.running:
            try:
                ready, _, _ = select.select([self.sock], [], [], 0.1)
                if not ready:
                    continue
                data, addr = self.sock.recvfrom(1024)
            except OSError:
                continue

            if len(data) < 2:
                continue

            header = data[:2]
            fmt_info = PACKET_FORMATS.get(header)
            if fmt_info is None:
                continue

            fmt, fields = fmt_info
            expected_size = struct.calcsize(fmt)
            if len(data) < expected_size:
                continue

            try:
                values = struct.unpack(fmt, data[:expected_size])
                pkt_dict = dict(zip(fields, values))
                del pkt_dict['header']
                with self.lock:
                    self.last_packets[header.decode()] = pkt_dict
            except struct.error:
                pass

    # --- Heartbeat + teleop loop ---
    def send_loop(self):
        while self.running:
            self.send_heartbeat()
            # Send teleop only if in remote mode (2) and not e-stop
            if self.current_mode == 2 and not self.estop_active:
                self.send_teleop()
            time.sleep(0.05)  # 20 Hz

    # --- Display ---
    def display_loop(self):
        while self.running:
            lines = [
                '\033[2J\033[H',  # clear screen
                '=== NEV Test Server ===',
                f'Target: {self.vehicle_addr[0]}:{self.vehicle_addr[1]}  '
                f'Listen: {self.listen_port}',
                f'Mode: {self.current_mode}  E-Stop: {self.estop_active}  '
                f'Lin: {self.linear_x:.2f}  Ang: {self.angular_z:.2f}',
                '',
                'Controls: w/s=lin a/d=ang space=stop e=estop 0-2/i=mode q=quit',
                '-' * 60,
            ]

            with self.lock:
                # Sort keys to keep display stable
                for ptype in sorted(self.last_packets.keys()):
                    pkt = self.last_packets[ptype]
                    items = '  '.join(f'{k}={v}' for k, v in pkt.items()
                                      if k not in ('seq', 'pad'))
                    lines.append(f'[{ptype}] {items}')

            sys.stdout.write('\r' + '\n'.join(lines))
            sys.stdout.flush()
            time.sleep(0.2)

    # --- Keyboard input (Raw Mode) ---
    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def input_loop(self):
        print('Test server started. Use keyboard controls.')
        while self.running:
            try:
                # Check if input is available to avoid blocking completely
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = self.get_key().lower()
                    
                    if key == 'q':
                        self.running = False
                        break
                    elif key == '\x03': # Ctrl+C
                        self.running = False
                        break
                    elif key == 'w':
                        self.linear_x = min(self.linear_x + 0.1, 1.5)
                    elif key == 's':
                        self.linear_x = max(self.linear_x - 0.1, -1.5)
                    elif key == 'a':
                        self.angular_z = min(self.angular_z + 0.1, 2.0)
                    elif key == 'd':
                        self.angular_z = max(self.angular_z - 0.1, -2.0)
                    elif key == ' ':
                        self.linear_x = 0.0
                        self.angular_z = 0.0
                    elif key == 'e':
                        self.estop_active = not self.estop_active
                        self.send_estop(self.estop_active)
                    elif key == 'i':
                        self.current_mode = -1
                        self.send_cmd_mode(-1)
                    elif key == '0':
                        self.current_mode = 0
                        self.send_cmd_mode(0)
                    elif key == '1':
                        self.current_mode = 1
                        self.send_cmd_mode(1)
                    elif key == '2':
                        self.current_mode = 2
                        self.send_cmd_mode(2)
            except Exception as e:
                pass

    def run(self):
        threads = [
            threading.Thread(target=self.recv_loop, daemon=True),
            threading.Thread(target=self.send_loop, daemon=True),
            threading.Thread(target=self.display_loop, daemon=True),
        ]
        for t in threads:
            t.start()

        try:
            self.input_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            # Clean up slightly
            time.sleep(0.5)
            self.sock.close()
            print('\nServer stopped.')


def main():
    parser = argparse.ArgumentParser(description='NEV Test Server')
    parser.add_argument('--vehicle-ip', default='127.0.0.1')
    parser.add_argument('--vehicle-port', type=int, default=5000)
    parser.add_argument('--listen-port', type=int, default=5001)
    args = parser.parse_args()

    server = TestServer(args.vehicle_ip, args.vehicle_port, args.listen_port)
    server.run()


if __name__ == '__main__':
    main()