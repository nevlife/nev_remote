import time
from dataclasses import dataclass

_STATUS_MAP = {0: 0, 3: 1}


@dataclass
class HealthState:
    bridge_flag: int
    flag_changed: bool
    connected: bool
    status_code: int


class HealthMonitor:

    def __init__(self, hb_timeout: float, ctrl_timeout: float):
        self._hb_timeout = hb_timeout
        self._ctrl_timeout = ctrl_timeout
        self._prev_flag = 0

    def evaluate(
        self,
        last_hb_time: float | None,
        last_ctrl_time: float,
        current_mode: int,
        server_estop: bool,
    ) -> HealthState:
        now = time.monotonic()

        if server_estop:
            flag = 1
        elif last_hb_time is not None and (now - last_hb_time) > self._hb_timeout:
            flag = 3
        elif (
            current_mode == 2
            and last_ctrl_time > 0
            and (now - last_ctrl_time) > self._ctrl_timeout
        ):
            flag = 4
        else:
            flag = 0

        changed = flag != self._prev_flag
        self._prev_flag = flag

        connected = flag == 0 and last_hb_time is not None
        status_code = _STATUS_MAP.get(flag, 2)

        return HealthState(
            bridge_flag=flag,
            flag_changed=changed,
            connected=connected,
            status_code=status_code,
        )
