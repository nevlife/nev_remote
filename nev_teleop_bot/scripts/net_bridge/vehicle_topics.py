import importlib
import re
import time

import zenoh

ZENOH_RELIABILITY = {
    "reliable": zenoh.Reliability.RELIABLE,
    "best_effort": zenoh.Reliability.BEST_EFFORT,
}
ZENOH_CONGESTION = {
    "drop": zenoh.CongestionControl.DROP,
    "block": zenoh.CongestionControl.BLOCK,
}
ZENOH_PRIORITY = {
    "real_time": zenoh.Priority.REAL_TIME,
    "interactive_high": zenoh.Priority.INTERACTIVE_HIGH,
    "interactive_low": zenoh.Priority.INTERACTIVE_LOW,
    "data_high": zenoh.Priority.DATA_HIGH,
    "data": zenoh.Priority.DATA,
    "data_low": zenoh.Priority.DATA_LOW,
    "background": zenoh.Priority.BACKGROUND,
}


def _derive_name(msg_type: str) -> str:
    class_name = msg_type.rsplit("/", 1)[-1]
    return re.sub(r"(?<!^)(?=[A-Z])", "_", class_name).lower()


def _import_msg_class(msg_type: str):
    parts = msg_type.split("/")
    module = importlib.import_module(f"{parts[0]}.{parts[1]}")
    return getattr(module, parts[2])


def _parse_zenoh_qos(cfg: dict) -> dict:
    return dict(
        reliability=ZENOH_RELIABILITY.get(
            cfg.get("reliability", "best_effort"), zenoh.Reliability.BEST_EFFORT
        ),
        congestion_control=ZENOH_CONGESTION.get(
            cfg.get("congestion", "drop"), zenoh.CongestionControl.DROP
        ),
        priority=ZENOH_PRIORITY.get(cfg.get("priority", "data"), zenoh.Priority.DATA),
    )


class VehicleTopicLoader:

    def __init__(self):
        self._cache: dict[str, dict] = {}

    def load(self, path: str, node, transport, vehicle_id: str) -> None:
        import yaml
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        ROS_QOS = {
            "reliable": QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            "best_effort": QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            "sensor_data": QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        }

        try:
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            node.get_logger().error(f"Failed to load teleop_topics config: {e}")
            return

        for t in cfg.get("teleop_topics", []):
            topic_name = t["topic_name"]
            msg_type = t["msg_type"]
            name = t.get("name") or _derive_name(msg_type)
            msg_class = _import_msg_class(msg_type)

            zenoh_key = f"nev/robot/{vehicle_id}/vehicle/{name}"
            zenoh_qos = _parse_zenoh_qos(t.get("zenoh_qos", {}))
            transport.declare_publisher(zenoh_key, **zenoh_qos)

            self._cache[name] = {
                "msg": None,
                "zenoh_key": zenoh_key,
                "rate": t.get("rate", "vehicle"),
            }

            rq = t.get("ros_qos", {})
            ros_qos = ROS_QOS.get(rq.get("reliability", "reliable"), ROS_QOS["reliable"])
            node.create_subscription(
                msg_class,
                topic_name,
                lambda msg, n=name: self._on_msg(n, msg),
                ros_qos,
            )

            node.get_logger().info(f"Teleop topic: {topic_name} ({msg_type}) -> {zenoh_key}")

    def _on_msg(self, name: str, msg):
        self._cache[name]["msg"] = msg

    def publish(self, rate_group: str, transport) -> None:
        from rosidl_runtime_py import message_to_ordereddict

        ts = time.time()
        for name, entry in self._cache.items():
            if entry["rate"] != rate_group or entry["msg"] is None:
                continue
            data = dict(message_to_ordereddict(entry["msg"]))
            data["ts"] = ts
            transport.put(entry["zenoh_key"], data)
