"""ROS2 evaluator/recorder for outputs produced by the real navigation nodes."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from .result_writer import SampleRecorder
from .scenario_schema import Scenario


def _values(status: Any) -> dict[str, str]:
    return {str(item.key): str(item.value) for item in status.values}


def _number(values: dict[str, str], key: str) -> float | None:
    try:
        return float(values[key])
    except (KeyError, TypeError, ValueError):
        return None


def _integer(values: dict[str, str], key: str) -> int | None:
    try:
        return int(float(values[key]))
    except (KeyError, TypeError, ValueError):
        return None


def _boolean(values: dict[str, str], key: str) -> bool | None:
    value = values.get(key)
    if value is None:
        return None
    return value.strip().lower() in {"1", "true", "yes", "accepted", "ok"}


def _level(status: Any) -> int:
    raw = status.level
    if isinstance(raw, (bytes, bytearray)):
        return int(raw[0]) if raw else 0
    return int(raw)


class OutputState:
    """Latest values observed from each actual software output topic."""

    def __init__(self) -> None:
        self.gnss: dict[str, Any] = {}
        self.lidar: dict[str, Any] = {}
        self.fusion: dict[str, Any] = {}
        self.navigation: dict[str, Any] = {}
        self.relocalization: dict[str, Any] = {}
        self.cmd_linear_x: float | None = None
        self.cmd_angular_z: float | None = None


def _parse_diagnostic(message: Any, state: OutputState) -> None:
    for status in message.status:
        name = str(status.name).lower()
        values = _values(status)
        if "gnss quality" in name or "gnss" in name and "quality" in name:
            state.gnss = {
                "state": values.get("current_state", str(status.message).split("/", 1)[0]),
                "decision": values.get("decision", str(status.message).split("/", 1)[-1]),
                "satellites": _integer(values, "satellites"),
                "hdop": _number(values, "hdop"),
                "differential_age": _number(values, "differential_age"),
                "accepted": _boolean(values, "accepted"),
            }
        elif "lidar/quality" in name or ("lidar" in name and "quality" in name):
            geometry = _number(values, "geometry_score")
            level = _level(status)
            lidar_state = "REJECTED" if level >= 2 or (geometry is not None and geometry <= 0.0) else ("DEGRADED" if geometry is not None and geometry < 0.2 else "GOOD")
            state.lidar = {
                "state": lidar_state,
                "raw_point_count": _integer(values, "raw_point_count"),
                "valid_point_count": _integer(values, "valid_point_count"),
                "stable_point_count": _integer(values, "stable_point_count"),
                "dynamic_candidate_count": _integer(values, "dynamic_candidate_count"),
                "temporal_match_ratio": _number(values, "temporal_match_ratio"),
                "angular_coverage": _number(values, "angular_coverage"),
                "geometry_score": geometry,
            }
        elif "multi-source fusion" in name or "fusion" in name:
            state.fusion = {
                "mode": values.get("fusion_mode", str(status.message)),
                "accepted_source": values.get("accepted_source"),
                "measurement_confidence": _number(values, "measurement_confidence"),
                "position_uncertainty": _number(values, "position_uncertainty"),
                "yaw_uncertainty": _number(values, "yaw_uncertainty"),
                "adaptive_position_uncertainty": _number(values, "adaptive_position_uncertainty"),
                "adaptive_yaw_uncertainty": _number(values, "adaptive_yaw_uncertainty"),
            }
        elif "navigation health" in name or "navigation" in name:
            state.navigation = {
                "state": values.get("overall_state", str(status.message)),
            }
        elif "active relocalization" in name or "relocalization supervisor" in name:
            state.relocalization = {
                "state": values.get("state", str(status.message)),
            }


def create_node(scenario: Scenario, output_dir: Path, node_name: str = "dg_synthetic_evaluator") -> Any:
    """Create the evaluator node; it subscribes to real outputs only."""
    from rclpy.node import Node
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist

    class SyntheticEvaluatorNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self._scenario = scenario
            self._start = time.monotonic()
            self._state = OutputState()
            self._recorder = SampleRecorder(scenario, output_dir)
            self._finished = False
            self.create_subscription(DiagnosticArray, "/dg/gnss/quality", lambda msg: _parse_diagnostic(msg, self._state), 10)
            self.create_subscription(DiagnosticArray, "/dg/lidar/quality", lambda msg: _parse_diagnostic(msg, self._state), 10)
            self.create_subscription(DiagnosticArray, "/dg/fusion/status", lambda msg: _parse_diagnostic(msg, self._state), 10)
            self.create_subscription(DiagnosticArray, "/dg/navigation/status", lambda msg: _parse_diagnostic(msg, self._state), 10)
            self.create_subscription(DiagnosticArray, "/dg/relocalization/status", lambda msg: _parse_diagnostic(msg, self._state), 10)
            self.create_subscription(Twist, "/dg/test_cmd_vel", self._on_cmd, 10)
            self._timer = self.create_timer(1.0 / 10.0, self._sample)

        @property
        def recorder(self) -> SampleRecorder:
            return self._recorder

        @property
        def elapsed(self) -> float:
            return time.monotonic() - self._start

        def _on_cmd(self, message: Any) -> None:
            self._state.cmd_linear_x = float(message.linear.x)
            self._state.cmd_angular_z = float(message.angular.z)

        def _sample(self) -> None:
            elapsed = self.elapsed
            if elapsed > self._scenario.duration_sec:
                return
            phase = self._scenario.phase_at(elapsed)
            gnss = self._state.gnss
            lidar = self._state.lidar
            fusion = self._state.fusion
            navigation = self._state.navigation
            relocalization = self._state.relocalization
            self._recorder.add({
                "scenario_id": self._scenario.scenario_id,
                "phase": phase.phase_id,
                "elapsed_time": elapsed,
                "ros_timestamp": self.get_clock().now().nanoseconds / 1e9,
                "gnss_state": gnss.get("state"),
                "gnss_decision": gnss.get("decision"),
                "gnss_satellites": gnss.get("satellites"),
                "gnss_hdop": gnss.get("hdop"),
                "gnss_differential_age": gnss.get("differential_age"),
                "gnss_accepted": gnss.get("accepted"),
                **{key: lidar.get(key) for key in (
                    "lidar_state", "raw_point_count", "valid_point_count", "stable_point_count",
                    "dynamic_candidate_count", "temporal_match_ratio", "angular_coverage", "geometry_score")},
                "lidar_state": lidar.get("state"),
                "fusion_mode": fusion.get("mode"),
                "accepted_source": fusion.get("accepted_source"),
                "measurement_confidence": fusion.get("measurement_confidence"),
                "position_uncertainty": fusion.get("position_uncertainty"),
                "yaw_uncertainty": fusion.get("yaw_uncertainty"),
                "adaptive_position_uncertainty": fusion.get("adaptive_position_uncertainty"),
                "adaptive_yaw_uncertainty": fusion.get("adaptive_yaw_uncertainty"),
                "navigation_state": navigation.get("state"),
                "relocalization_state": relocalization.get("state"),
                # The Twist message has no source metadata.  Keeping this
                # empty is intentional; the arbiter's source is not guessed.
                "cmd_vel_source": "",
                "cmd_linear_x": self._state.cmd_linear_x,
                "cmd_angular_z": self._state.cmd_angular_z,
            })

        def finish(self) -> None:
            if self._finished:
                return
            self._finished = True
            self._recorder.write_csvs()

    return SyntheticEvaluatorNode()


def main(args: list[str] | None = None) -> None:
    import argparse
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    parser = argparse.ArgumentParser(description="Record actual DG ROS2 output diagnostics")
    parser.add_argument("scenario_file", type=Path)
    parser.add_argument("output_dir", type=Path)
    parsed = parser.parse_args(args)
    from .scenario_schema import load_scenario

    scenario = load_scenario(parsed.scenario_file)
    rclpy.init(args=None)
    node = create_node(scenario, parsed.output_dir)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
