"""CSV/timeline/result writers with no ROS dependency.

Keeping this part ROS-free makes the evaluator contract unit-testable and
prevents the report generator from inventing values when a topic is absent.
Missing observations are written as empty cells and reported as warnings.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

from .scenario_schema import Scenario


TRUTH_LABELS = {
    "authenticity_marker": "THIS_IS_SYNTHETIC_SOFTWARE_VALIDATION",
    "validation_class": "SYNTHETIC_SOFTWARE_VALIDATION",
    "data_class": "NOT_REAL_ROBOT_DATA",
    "simulator_class": "NOT_GAZEBO_DATA",
    "performance_claim_label": "NOT_COMPETITION_PERFORMANCE_EVIDENCE",
}

SAMPLE_COLUMNS = [
    "scenario_id", "phase", "elapsed_time", "ros_timestamp",
    "gnss_state", "gnss_decision", "gnss_satellites", "gnss_hdop",
    "gnss_differential_age", "gnss_accepted", "lidar_state",
    "raw_point_count", "valid_point_count", "stable_point_count",
    "dynamic_candidate_count", "temporal_match_ratio", "angular_coverage",
    "geometry_score", "fusion_mode", "accepted_source",
    "measurement_confidence", "position_uncertainty", "yaw_uncertainty",
    "adaptive_position_uncertainty", "adaptive_yaw_uncertainty",
    "navigation_state", "relocalization_state", "cmd_vel_source",
    "cmd_linear_x", "cmd_angular_z", "safety_cmd_vel_publishers",
]


def _csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float):
        return value if math.isfinite(value) else ""
    return value


@dataclass
class TimelineTracker:
    """Record only state changes, preserving the first observation."""

    rows: list[dict[str, Any]] = field(default_factory=list)
    _last: dict[str, str] = field(default_factory=dict)

    def update(self, elapsed: float, ros_timestamp: float, phase: str, values: dict[str, Any]) -> None:
        for component, raw in values.items():
            state = "" if raw is None else str(raw)
            if self._last.get(component) == state:
                continue
            previous = self._last.get(component, "")
            self._last[component] = state
            self.rows.append({
                "elapsed_time": elapsed,
                "ros_timestamp": ros_timestamp,
                "phase": phase,
                "component": component,
                "previous_state": previous,
                "new_state": state,
                "reason": "STATE_CHANGE" if previous else "INITIAL_OBSERVATION",
            })


@dataclass
class SampleRecorder:
    scenario: Scenario
    output_dir: Path
    samples: list[dict[str, Any]] = field(default_factory=list)
    timeline: TimelineTracker = field(default_factory=TimelineTracker)

    def add(self, row: dict[str, Any]) -> None:
        normalized = {column: _csv_value(row.get(column)) for column in SAMPLE_COLUMNS}
        self.samples.append(normalized)
        self.timeline.update(
            float(row.get("elapsed_time", 0.0)),
            float(row.get("ros_timestamp", 0.0)),
            str(row.get("phase", "")),
            {
                "gnss": row.get("gnss_state"),
                "lidar": row.get("lidar_state"),
                "fusion": row.get("fusion_mode"),
                "navigation": row.get("navigation_state"),
                "relocalization": row.get("relocalization_state"),
            },
        )

    def write_csvs(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        with (self.output_dir / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=SAMPLE_COLUMNS)
            writer.writeheader()
            writer.writerows(self.samples)
        timeline_columns = ["elapsed_time", "ros_timestamp", "phase", "component", "previous_state", "new_state", "reason"]
        with (self.output_dir / "timeline.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=timeline_columns)
            writer.writeheader()
            writer.writerows(self.timeline.rows)


def _states(samples: Iterable[dict[str, Any]], key: str) -> list[str]:
    return [str(row.get(key, "")).upper() for row in samples if row.get(key) not in (None, "")]


def _has_in_order(values: list[str], sequence: tuple[str, ...]) -> bool:
    index = 0
    for value in values:
        if value == sequence[index]:
            index += 1
            if index == len(sequence):
                return True
    return False


def _finite_samples(samples: Iterable[dict[str, Any]], keys: tuple[str, ...]) -> bool:
    for row in samples:
        for key in keys:
            value = row.get(key)
            if value in (None, ""):
                continue
            try:
                if not math.isfinite(float(value)):
                    return False
            except (TypeError, ValueError):
                return False
    return True


def evaluate_scenario(
    scenario: Scenario,
    samples: list[dict[str, Any]],
    safety_cmd_vel_publishers: int,
    integration_alive: bool,
) -> dict[str, Any]:
    """Apply conservative software checks appropriate to S01-S04."""
    states_gnss = _states(samples, "gnss_state")
    states_lidar = _states(samples, "lidar_state")
    states_nav = _states(samples, "navigation_state")
    states_reloc = _states(samples, "relocalization_state")
    errors: list[str] = []
    warnings: list[str] = []
    checks: dict[str, bool] = {}

    checks["samples_present"] = bool(samples)
    checks["integration_alive"] = integration_alive
    checks["real_cmd_vel_is_unpublished"] = safety_cmd_vel_publishers == 0
    checks["finite_fusion_outputs"] = _finite_samples(
        samples,
        ("measurement_confidence", "position_uncertainty", "yaw_uncertainty",
         "adaptive_position_uncertainty", "adaptive_yaw_uncertainty"),
    )
    checks["relocalization_not_unexpectedly_active"] = not any(
        state in {"TRIGGERED", "STOPPING", "ACTIVE_SCAN", "WAITING_CANDIDATE", "VERIFYING", "FAILED", "MANUAL_REQUIRED"}
        for state in states_reloc
    )
    if not checks["samples_present"]:
        errors.append("NO_EVALUATOR_SAMPLES")
    if not integration_alive:
        errors.append("INTEGRATION_LAUNCH_EXITED_EARLY")
    if safety_cmd_vel_publishers != 0:
        errors.append(f"UNSAFE_REAL_CMD_VEL_PUBLISHERS={safety_cmd_vel_publishers}")
    if not checks["finite_fusion_outputs"]:
        errors.append("NONFINITE_FUSION_OUTPUT")
    if not checks["relocalization_not_unexpectedly_active"]:
        errors.append("UNEXPECTED_RELOCALIZATION_STATE")

    if scenario.scenario_id == "S01":
        checks["gnss_nominal_seen"] = "GOOD" in states_gnss
        checks["lidar_nominal_seen"] = "GOOD" in states_lidar
        checks["navigation_not_failed"] = not any(state in {"FAILED", "MANUAL_REQUIRED"} for state in states_nav)
        for name in ("gnss_nominal_seen", "lidar_nominal_seen", "navigation_not_failed"):
            if not checks[name]:
                errors.append(name.upper())
    elif scenario.scenario_id == "S02":
        checks["gnss_good_to_degraded"] = _has_in_order(states_gnss, ("GOOD", "DEGRADED"))
        checks["gnss_rejection_seen"] = any(state in {"REJECTED", "STALE"} for state in states_gnss)
        checks["gnss_rejected_fix_not_accepted"] = all(
            str(row.get("gnss_accepted", "")).lower() not in {"true", "1"}
            for row in samples
            if str(row.get("gnss_state", "")).upper() in {"REJECTED", "STALE"}
        )
        for name in ("gnss_good_to_degraded", "gnss_rejection_seen", "gnss_rejected_fix_not_accepted"):
            if not checks[name]:
                errors.append(name.upper())
    elif scenario.scenario_id == "S03":
        scores = [float(row["geometry_score"]) for row in samples if row.get("geometry_score") not in (None, "")]
        checks["lidar_good_seen"] = any(score >= 0.2 for score in scores)
        checks["lidar_degraded_score_seen"] = any(score < 0.2 for score in scores)
        checks["lidar_state_degraded_or_rejected"] = any(state in {"DEGRADED", "REJECTED"} for state in states_lidar)
        for name in ("lidar_good_seen", "lidar_degraded_score_seen", "lidar_state_degraded_or_rejected"):
            if not checks[name]:
                errors.append(name.upper())
    elif scenario.scenario_id == "S04":
        checks["gnss_rejection_seen"] = any(state in {"REJECTED", "STALE"} for state in states_gnss)
        checks["lidar_degradation_seen"] = any(state in {"DEGRADED", "REJECTED"} for state in states_lidar)
        checks["navigation_degradation_seen"] = any(state in {"DEGRADED", "LOCALIZATION_SUSPECT", "RECOVERING"} for state in states_nav)
        for name in ("gnss_rejection_seen", "lidar_degradation_seen", "navigation_degradation_seen"):
            if not checks[name]:
                errors.append(name.upper())
    else:
        warnings.append("UNKNOWN_SCENARIO_NO_SCENARIO_SPECIFIC_ASSERTIONS")

    if not states_reloc:
        warnings.append("RELOCALIZATION_STATUS_NOT_OBSERVED")
    if not any(row.get("fusion_mode") not in (None, "") for row in samples):
        warnings.append("FUSION_STATUS_NOT_OBSERVED")

    return {
        **TRUTH_LABELS,
        "scenario_id": scenario.scenario_id,
        "scenario_name": scenario.name,
        "result": "PASS" if not errors else "FAIL",
        "overall_result": "PASS" if not errors else "FAIL",
        "evidence_level": "SYNTHETIC_SOFTWARE_VALIDATION",
        "gazebo": False,
        "real_robot": False,
        "performance_claim": False,
        "seed": scenario.seed,
        "duration_sec": scenario.duration_sec,
        "sample_count": len(samples),
        "checks": checks,
        "failures": errors,
        "errors": errors,
        "warnings": warnings,
    }


def write_result(output_dir: Path, result: dict[str, Any]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "result.json").open("w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, ensure_ascii=False, sort_keys=True)
    with (output_dir / "result.md").open("w", encoding="utf-8") as handle:
        handle.write(f"# {result['scenario_id']} — {result['result']}\n\n")
        handle.write("- Authenticity marker: `THIS_IS_SYNTHETIC_SOFTWARE_VALIDATION`\n")
        handle.write("- Validation class: `SYNTHETIC_SOFTWARE_VALIDATION`\n")
        handle.write("- Data class: `NOT_REAL_ROBOT_DATA`\n")
        handle.write("- Simulator class: `NOT_GAZEBO_DATA`\n")
        handle.write("- Performance claim: `NOT_COMPETITION_PERFORMANCE_EVIDENCE` (boolean claim field is false)\n\n")
        handle.write(f"Samples: {result['sample_count']}\n\n")
        handle.write("## Checks\n\n")
        for key, value in result["checks"].items():
            handle.write(f"- `{key}`: {value}\n")
        if result["errors"]:
            handle.write("\n## Errors\n\n" + "\n".join(f"- {item}" for item in result["errors"]) + "\n")
        if result["warnings"]:
            handle.write("\n## Warnings\n\n" + "\n".join(f"- {item}" for item in result["warnings"]) + "\n")
