import json
import math
import threading
import time
from dataclasses import dataclass

from .zenoh_transport import ZenohTransport


@dataclass
class PendingCommands:
    teleop: tuple | None = None
    estop: bool | None = None
    mode: int | None = None


class InboundHandler:

    def __init__(
        self, vehicle_id: str, transport: ZenohTransport,
        logger, wheelbase: float = 0.650,
    ):
        self._vid = vehicle_id
        self._transport = transport
        self._logger = logger
        self._wheelbase = wheelbase

        self._lock = threading.Lock()
        self._pending_teleop: tuple | None = None
        self._pending_estop: bool | None = None
        self._pending_mode: int | None = None

        self.last_hb_time: float | None = None
        self.last_ctrl_time: float = 0.0

    def on_server_ping(self, sample):
        self.last_hb_time = time.monotonic()
        try:
            data = json.loads(bytes(sample.payload))
            ts = data.get("ts")
            if ts is None:
                return
            self._transport.put(f"nev/robot/{self._vid}/server_pong", {"ts": ts})
        except Exception as e:
            self._logger.warning(f"server_ping parse error: {e}")

    def on_bot_ping(self, sample):
        try:
            data = json.loads(bytes(sample.payload))
            ts = data.get("ts")
            if ts is None:
                return
            self._transport.put(f"nev/robot/{self._vid}/bot_pong", {"ts": ts})
        except Exception as e:
            self._logger.warning(f"bot_ping parse error: {e}")

    def on_teleop(self, sample):
        try:
            data = json.loads(bytes(sample.payload))
        except Exception as e:
            self._logger.warning(f"teleop JSON parse error: {e}")
            return
        lx = max(min(float(data.get("linear_x", 0.0)), 2.0), -2.0)
        steer = float(data.get("steer_angle", 0.0))
        if abs(steer) < 1e-6:
            az = 0.0
        elif abs(lx) < 0.05:
            az = steer
        elif self._wheelbase > 0:
            az = lx * math.tan(steer) / self._wheelbase
        else:
            az = 0.0
        az = max(min(az, 2.0), -2.0)
        with self._lock:
            self._pending_teleop = (lx, az)

    def on_estop(self, sample):
        try:
            data = json.loads(bytes(sample.payload))
        except Exception as e:
            self._logger.warning(f"estop JSON parse error: {e}")
            return
        with self._lock:
            self._pending_estop = bool(data.get("active", False))

    def on_cmd_mode(self, sample):
        try:
            data = json.loads(bytes(sample.payload))
        except Exception as e:
            self._logger.warning(f"cmd_mode JSON parse error: {e}")
            return
        with self._lock:
            self._pending_mode = int(data.get("mode", -1))

    def drain_pending(self) -> PendingCommands:
        with self._lock:
            cmds = PendingCommands(
                teleop=self._pending_teleop,
                estop=self._pending_estop,
                mode=self._pending_mode,
            )
            self._pending_teleop = None
            self._pending_estop = None
            self._pending_mode = None
        if cmds.teleop is not None:
            self.last_ctrl_time = time.monotonic()
        return cmds
