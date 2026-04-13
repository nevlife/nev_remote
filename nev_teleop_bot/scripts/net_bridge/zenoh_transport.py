import json

import zenoh


class ZenohTransport:

    def __init__(self, locator: str, logger):
        self._logger = logger
        self._pubs: dict[str, zenoh.Publisher] = {}
        self._subs: list = []

        conf = zenoh.Config()
        if locator:
            conf.insert_json5("connect/endpoints", json.dumps([locator]))

        try:
            self._session = zenoh.open(conf)
        except Exception as e:
            logger.fatal(f"Zenoh connect failed: {e}")
            raise SystemExit(1)

        logger.info(f'Zenoh connected -> {locator or "auto-discovery"}')

    @property
    def session(self) -> zenoh.Session:
        return self._session

    def declare_publisher(self, key: str, **qos) -> None:
        self._pubs[key] = self._session.declare_publisher(key, **qos)

    def declare_subscriber(self, key: str, callback) -> None:
        self._subs.append(self._session.declare_subscriber(key, callback))

    def put(self, key: str, data) -> None:
        try:
            self._pubs[key].put(json.dumps(data))
        except Exception as e:
            self._logger.warning(f"zenoh put [{key}]: {e}")

    def close(self) -> None:
        for sub in self._subs:
            try:
                sub.undeclare()
            except Exception:
                pass
        self._subs.clear()
        for key, pub in self._pubs.items():
            try:
                pub.undeclare()
            except Exception as e:
                self._logger.warning(f"Error undeclaring publisher [{key}]: {e}")
        self._pubs.clear()
        try:
            self._session.close()
        except Exception:
            pass
