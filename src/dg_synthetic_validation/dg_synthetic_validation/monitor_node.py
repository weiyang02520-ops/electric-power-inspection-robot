"""Read-only live monitor for DG synthetic validation outputs.

The node subscribes to diagnostics and the safety-routed test Twist only.  It
never publishes and therefore cannot influence the navigation graph.
"""

from __future__ import annotations

import argparse
import time
from typing import Any


def _values(status: Any) -> dict[str, str]:
    return {str(item.key): str(item.value) for item in status.values}


def _level(raw: Any) -> int:
    if isinstance(raw, (bytes, bytearray, memoryview)):
        return int(raw[0]) if raw else 0
    try:
        return int(raw)
    except (TypeError, ValueError):
        return -1


def _state(status: Any) -> str:
    values = _values(status)
    return values.get("current_state") or values.get("overall_state") or values.get("state") or str(status.message)


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Read-only DG ROS2 runtime monitor")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 means until Ctrl-C")
    parser.add_argument("--refresh", type=float, default=1.0)
    parsed = parser.parse_args(args)

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    class Monitor(Node):
        def __init__(self) -> None:
            super().__init__("dg_synthetic_monitor")
            self.started = time.monotonic()
            self.latest: dict[str, str] = {}
            self.cmd = (None, None)
            for topic, key in (
                ("/dg/gnss/quality", "GNSS"),
                ("/dg/lidar/quality", "LIDAR"),
                ("/dg/fusion/status", "FUSION"),
                ("/dg/navigation/status", "NAV"),
                ("/dg/relocalization/status", "RELOCALIZATION"),
            ):
                self.create_subscription(DiagnosticArray, topic, lambda msg, k=key: self.on_diag(k, msg), 10)
            self.create_subscription(Twist, "/dg/test_cmd_vel", self.on_cmd, 10)
            self.create_timer(max(0.2, parsed.refresh), self.render)

        def on_diag(self, key: str, message: Any) -> None:
            if not message.status:
                self.latest[key] = "NO_STATUS"
                return
            status = message.status[0]
            self.latest[key] = f"{_state(status)} (level={_level(status.level)})"

        def on_cmd(self, message: Any) -> None:
            self.cmd = (float(message.linear.x), float(message.angular.z))

        def render(self) -> None:
            elapsed = time.monotonic() - self.started
            if parsed.duration > 0 and elapsed >= parsed.duration:
                rclpy.shutdown()
                return
            rows = [f"DG-202611 READ-ONLY MONITOR  elapsed={elapsed:6.1f}s",
                    "SYNTHETIC SOFTWARE VALIDATION | NOT_REAL_ROBOT_DATA"]
            for key in ("GNSS", "LIDAR", "FUSION", "NAV", "RELOCALIZATION"):
                rows.append(f"{key:14s}: {self.latest.get(key, 'NO_DATA')}")
            linear, angular = self.cmd
            cmd = "NO_DATA" if linear is None else f"linear.x={linear:+.3f} angular.z={angular:+.3f}"
            rows.append(f"TEST CMD_VEL   : {cmd}  (/dg/test_cmd_vel only)")
            print("\033[2J\033[H" + "\n".join(rows), flush=True)

    rclpy.init(args=None)
    node = Monitor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
