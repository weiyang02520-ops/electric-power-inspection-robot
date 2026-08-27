"""Small, explicit scenario schema used by the injector and evaluator.

The schema intentionally describes sensor inputs only.  It contains no knobs
for publishing any ``/dg/*`` health or fusion output, so a scenario cannot
accidentally bypass the software under test.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class GnssPhase:
    quality: int = 4
    satellites: int = 12
    hdop: float = 0.8
    differential_age: float = 0.5
    fix_available: bool = True
    publish_fix: bool = True
    publish_status: bool = True
    diagnostic_level: int = 0


@dataclass(frozen=True)
class LidarPhase:
    mode: str = "normal"
    valid_fraction: float = 1.0
    angle_min: float = -3.141592653589793
    angle_max: float = 3.141592653589793
    range_m: float = 3.0


@dataclass(frozen=True)
class Phase:
    phase_id: str
    start_sec: float
    end_sec: float
    gnss: GnssPhase
    lidar: LidarPhase

    def contains(self, elapsed: float) -> bool:
        return self.start_sec <= elapsed < self.end_sec


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    name: str
    duration_sec: float
    seed: int
    sensor_hz: float
    map_hz: float
    phases: tuple[Phase, ...]
    expected: dict[str, Any]

    def phase_at(self, elapsed: float) -> Phase:
        if not self.phases:
            raise ValueError(f"scenario {self.scenario_id} has no phases")
        for phase in self.phases:
            if phase.contains(elapsed):
                return phase
        # A final sample can land exactly on duration_sec.  It belongs to the
        # last phase for reporting, while publishers stop after the duration.
        return self.phases[-1]


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _gnss(value: Any) -> GnssPhase:
    raw = _mapping(value, "gnss")
    return GnssPhase(
        quality=int(raw.get("quality", 4)),
        satellites=int(raw.get("satellites", 12)),
        hdop=float(raw.get("hdop", 0.8)),
        differential_age=float(raw.get("differential_age", 0.5)),
        fix_available=bool(raw.get("fix_available", True)),
        publish_fix=bool(raw.get("publish_fix", True)),
        publish_status=bool(raw.get("publish_status", True)),
        diagnostic_level=int(raw.get("diagnostic_level", 0)),
    )


def _lidar(value: Any) -> LidarPhase:
    raw = _mapping(value, "lidar")
    return LidarPhase(
        mode=str(raw.get("mode", "normal")),
        valid_fraction=float(raw.get("valid_fraction", 1.0)),
        angle_min=float(raw.get("angle_min", -3.141592653589793)),
        angle_max=float(raw.get("angle_max", 3.141592653589793)),
        range_m=float(raw.get("range_m", 3.0)),
    )


def load_scenario(path: str | Path) -> Scenario:
    """Load and validate one YAML scenario."""
    source = Path(path)
    with source.open("r", encoding="utf-8") as handle:
        raw = _mapping(yaml.safe_load(handle), "scenario")
    scenario_id = str(raw.get("scenario_id", "")).strip()
    if not scenario_id:
        raise ValueError(f"{source}: scenario_id is required")
    duration = float(raw.get("duration_sec", 0.0))
    if duration <= 0.0:
        raise ValueError(f"{source}: duration_sec must be positive")
    phases: list[Phase] = []
    previous_end = 0.0
    for index, item in enumerate(raw.get("phases", [])):
        phase_raw = _mapping(item, f"phases[{index}]")
        start = float(phase_raw.get("start_sec", previous_end))
        end = float(phase_raw.get("end_sec", start))
        phase_id = str(phase_raw.get("id", f"P{index + 1:02d}"))
        if start < previous_end or end <= start:
            raise ValueError(f"{source}: phase {phase_id} is not ordered")
        phases.append(Phase(phase_id, start, end, _gnss(phase_raw.get("gnss")), _lidar(phase_raw.get("lidar"))))
        previous_end = end
    if not phases or phases[0].start_sec != 0.0 or phases[-1].end_sec < duration:
        raise ValueError(f"{source}: phases must cover [0, duration_sec)")
    rates = _mapping(raw.get("rates"), "rates")
    return Scenario(
        scenario_id=scenario_id,
        name=str(raw.get("name", scenario_id)),
        duration_sec=duration,
        seed=int(raw.get("seed", 202611)),
        sensor_hz=float(rates.get("sensor_hz", 10.0)),
        map_hz=float(rates.get("map_hz", 1.0)),
        phases=tuple(phases),
        expected=_mapping(raw.get("expected"), "expected"),
    )
