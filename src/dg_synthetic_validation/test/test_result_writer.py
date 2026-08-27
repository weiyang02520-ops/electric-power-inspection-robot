from pathlib import Path

from dg_synthetic_validation.result_writer import SampleRecorder, evaluate_scenario
from dg_synthetic_validation.scenario_schema import load_scenario


def _row(scenario_id: str, elapsed: float, **updates):
    row = {
        "scenario_id": scenario_id,
        "phase": "P",
        "elapsed_time": elapsed,
        "ros_timestamp": elapsed,
        "gnss_state": "GOOD",
        "gnss_decision": "ACCEPT",
        "gnss_accepted": True,
        "lidar_state": "GOOD",
        "geometry_score": 0.8,
        "fusion_mode": "NOMINAL",
        "measurement_confidence": 0.8,
        "position_uncertainty": 0.1,
        "yaw_uncertainty": 0.1,
        "adaptive_position_uncertainty": 0.1,
        "adaptive_yaw_uncertainty": 0.1,
        "navigation_state": "NOMINAL",
        "relocalization_state": "NORMAL",
    }
    row.update(updates)
    return row


def test_timeline_deduplicates_state_changes(tmp_path: Path):
    scenario = load_scenario(Path(__file__).parents[1] / "config/S01.yaml")
    recorder = SampleRecorder(scenario, tmp_path)
    recorder.add(_row("S01", 0.0))
    recorder.add(_row("S01", 0.1))
    recorder.add(_row("S01", 0.2, gnss_state="DEGRADED"))
    recorder.write_csvs()
    assert len(recorder.timeline.rows) == 6
    assert (tmp_path / "samples.csv").exists()
    assert (tmp_path / "timeline.csv").exists()


def test_s02_requires_real_transition_evidence():
    scenario = load_scenario(Path(__file__).parents[1] / "config/S02.yaml")
    samples = [
        _row("S02", 1.0),
        _row("S02", 5.0, gnss_state="DEGRADED", gnss_decision="ACCEPT_DEGRADED"),
        _row("S02", 9.0, gnss_state="REJECTED", gnss_decision="REJECT", gnss_accepted=False),
    ]
    result = evaluate_scenario(scenario, samples, safety_cmd_vel_publishers=0, integration_alive=True)
    assert result["result"] == "PASS"
    assert result["validation_class"] == "SYNTHETIC_SOFTWARE_VALIDATION"
    assert result["data_class"] == "NOT_REAL_ROBOT_DATA"
