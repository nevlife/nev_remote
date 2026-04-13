import time


class TelemetrySerializer:

    @staticmethod
    def serialize_vehicle(
        vehicle_id: str,
        mux_status,
        last_nav,
        last_teleop,
        last_final,
        connected: bool,
        status_code: int,
        estop_status,
    ) -> dict[str, dict]:
        ts = time.time()
        prefix = f"nev/robot/{vehicle_id}"
        payloads = {}

        payloads[f"{prefix}/mux"] = {
            "ts": ts,
            "requested_mode": int(mux_status.mode),
            "active_source": int(mux_status.cmd_source),
            "remote_enabled": bool(mux_status.remote_status),
            "nav_active": bool(mux_status.nav_active),
            "teleop_active": bool(mux_status.teleop_active),
            "final_active": bool(mux_status.final_active),
        }

        payloads[f"{prefix}/twist"] = {
            "ts": ts,
            "nav_lx": float(last_nav.linear.x),
            "nav_az": float(last_nav.angular.z),
            "teleop_lx": float(last_teleop.linear.x),
            "teleop_az": float(last_teleop.angular.z),
            "final_lx": float(last_final.linear.x),
            "final_az": float(last_final.angular.z),
        }

        payloads[f"{prefix}/network"] = {
            "connected": connected,
            "status_code": status_code,
        }

        payloads[f"{prefix}/estop"] = {
            "ts": ts,
            "is_estop": bool(estop_status.is_estop),
            "bridge_flag": int(estop_status.bridge_flag),
            "mux_flag": int(estop_status.mux_flag),
        }

        return payloads

    @staticmethod
    def serialize_resources(vehicle_id: str, cpu, mem, gpu, disk, net) -> dict[str, dict]:
        prefix = f"nev/robot/{vehicle_id}"
        payloads = {}

        if cpu is not None:
            payloads[f"{prefix}/cpu"] = {
                "cpu_usage": float(cpu.usage_percent),
                "cpu_temp": float(cpu.temperature_celsius),
                "cpu_load": float(cpu.load_avg_1m),
            }

        if mem is not None:
            payloads[f"{prefix}/mem"] = {
                "ram_total": int(mem.total_bytes // (1024 * 1024)),
                "ram_used": int(mem.used_bytes // (1024 * 1024)),
            }

        if gpu is not None:
            payloads[f"{prefix}/gpu"] = [
                {
                    "idx": int(g.index),
                    "gpu_usage": float(g.utilization_percent),
                    "gpu_mem_used": float(g.memory_used_mb),
                    "gpu_mem_total": float(g.memory_total_mb),
                    "gpu_temp": float(g.temperature_celsius),
                    "gpu_power": float(g.power_watts),
                }
                for g in gpu.gpus
            ]

        if disk is not None:
            payloads[f"{prefix}/disk"] = {
                "partitions": [
                    {
                        "idx": i,
                        "mountpoint": p.mountpoint,
                        "total_bytes": int(p.total_bytes),
                        "used_bytes": int(p.used_bytes),
                        "percent": float(p.percent),
                        "accessible": bool(p.accessible),
                    }
                    for i, p in enumerate(disk.partitions)
                ]
            }

        if net is not None:
            payloads[f"{prefix}/net"] = {
                "net_total_ifaces": int(net.total_interfaces),
                "net_active_ifaces": int(net.active_interfaces),
                "net_down_ifaces": int(net.down_interfaces),
                "interfaces": [
                    {
                        "idx": i,
                        "name": iface.name,
                        "is_up": bool(iface.is_up),
                        "speed_mbps": int(iface.speed_mbps),
                        "in_bps": float(iface.input_bytes_per_sec),
                        "out_bps": float(iface.output_bytes_per_sec),
                    }
                    for i, iface in enumerate(net.interfaces)
                ],
            }

        return payloads
