import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from navigation_health_core import (  # noqa: E402
    DEGRADED,
    FAILED,
    GOOD,
    LOCALIZATION_SUSPECT,
    MANUAL_REQUIRED,
    NOMINAL,
    RECOVERED,
    RECOVERING,
    REJECTED,
    STALE,
    NavigationHealthAggregator,
    NavigationHealthInput,
    parse_signal_diagnostic,
)


def event(**changes: object) -> NavigationHealthInput:
    values = dict(
        now=0.0,
        gnss_state="GOOD",
        gnss_fresh=True,
        lidar_state="GOOD",
        lidar_fresh=True,
        amcl_covariance=0.1,
        amcl_fresh=True,
        odom_fresh=True,
        scan_fresh=True,
        scan_match_state="GOOD",
        scan_match_fresh=True,
        relocalization_state="NORMAL",
    )
    values.update(changes)
    return NavigationHealthInput(**values)


class NavigationHealthCoreTests(unittest.TestCase):
    def test_all_healthy_is_nominal(self) -> None:
        output = NavigationHealthAggregator().evaluate(event())
        self.assertEqual(output.overall_state, NOMINAL)
        self.assertEqual(output.gnss_state, GOOD)
        self.assertFalse(output.reasons)
        self.assertTrue(output.transition)

    def test_gnss_degraded_continues_navigation(self) -> None:
        output = NavigationHealthAggregator().evaluate(event(gnss_state="DEGRADED"))
        self.assertEqual(output.overall_state, DEGRADED)
        self.assertEqual(output.gnss_state, DEGRADED)

    def test_gnss_rejected_is_not_amcl_lost(self) -> None:
        output = NavigationHealthAggregator().evaluate(event(gnss_state="REJECTED"))
        self.assertEqual(output.overall_state, DEGRADED)
        self.assertNotEqual(output.overall_state, LOCALIZATION_SUSPECT)

    def test_lidar_degradation_is_degraded(self) -> None:
        output = NavigationHealthAggregator().evaluate(event(lidar_state="DEGRADED"))
        self.assertEqual(output.overall_state, DEGRADED)

    def test_amcl_continuous_abnormality_is_localization_suspect(self) -> None:
        output = NavigationHealthAggregator().evaluate(event(amcl_covariance=1.0))
        self.assertEqual(output.overall_state, LOCALIZATION_SUSPECT)

    def test_relocalization_active_states_are_recovering(self) -> None:
        for state in ("TRIGGERED", "STOPPING", "ACTIVE_SCAN", "WAITING_CANDIDATE", "VERIFYING"):
            output = NavigationHealthAggregator().evaluate(event(relocalization_state=state))
            self.assertEqual(output.overall_state, RECOVERING, state)

    def test_recovered_state_is_exposed(self) -> None:
        output = NavigationHealthAggregator().evaluate(event(relocalization_state="RECOVERED"))
        self.assertEqual(output.overall_state, RECOVERED)

    def test_failed_and_manual_states_have_priority(self) -> None:
        aggregator = NavigationHealthAggregator()
        self.assertEqual(aggregator.evaluate(event(relocalization_state="FAILED")).overall_state, FAILED)
        self.assertEqual(aggregator.evaluate(event(relocalization_state="MANUAL_REQUIRED")).overall_state, MANUAL_REQUIRED)

    def test_stale_amcl_is_localization_suspect(self) -> None:
        output = NavigationHealthAggregator().evaluate(event(amcl_fresh=False))
        self.assertEqual(output.overall_state, LOCALIZATION_SUSPECT)
        self.assertEqual(output.amcl_state, STALE)

    def test_stale_side_channel_is_degraded_not_nominal(self) -> None:
        output = NavigationHealthAggregator().evaluate(event(lidar_fresh=False))
        self.assertEqual(output.overall_state, DEGRADED)
        self.assertEqual(output.lidar_state, STALE)

    def test_optional_missing_signals_do_not_block_nominal(self) -> None:
        output = NavigationHealthAggregator().evaluate(
            event(gnss_state=None, gnss_fresh=None, lidar_state=None, lidar_fresh=None, scan_match_state=None, scan_match_fresh=None)
        )
        self.assertEqual(output.overall_state, NOMINAL)

    def test_required_stale_signal_is_a_strong_reason(self) -> None:
        output = NavigationHealthAggregator().evaluate(event(gnss_fresh=False, required_signals=("gnss",)))
        self.assertEqual(output.gnss_state, STALE)
        self.assertIn("GNSS_STALE", output.reasons)
        self.assertIn("GNSS_REQUIRED_STALE", output.reasons)
        self.assertEqual(output.overall_state, LOCALIZATION_SUSPECT)

    def test_transition_is_only_true_when_state_changes(self) -> None:
        aggregator = NavigationHealthAggregator()
        first = aggregator.evaluate(event())
        second = aggregator.evaluate(event(now=0.1))
        third = aggregator.evaluate(event(now=0.2, gnss_state="DEGRADED"))
        fourth = aggregator.evaluate(event(now=0.3, gnss_state="DEGRADED"))
        self.assertTrue(first.transition)
        self.assertFalse(second.transition)
        self.assertTrue(third.transition)
        self.assertFalse(fourth.transition)

    def test_existing_gnss_diagnostic_mapping_is_explicit(self) -> None:
        observation = parse_signal_diagnostic(
            "gnss", {"current_state": "GOOD", "decision": "ACCEPT"}, 0, 1.0, 0.5, 1.0
        )
        self.assertEqual(observation.state, GOOD)
        self.assertTrue(observation.fresh)

    def test_existing_lidar_diagnostic_mapping_uses_valid_ratio(self) -> None:
        observation = parse_signal_diagnostic(
            "lidar", {"temporal_status": "NO_PREVIOUS", "valid_ratio": "0.9"}, 0, 1.0, 0.5, 1.0
        )
        self.assertEqual(observation.state, GOOD)

    def test_rejected_scan_match_diagnostic_is_rejected(self) -> None:
        observation = parse_signal_diagnostic(
            "scan_match", {"accepted": "false", "reason": "NO_MAP"}, 2, 1.0, 0.5, 1.0
        )
        self.assertEqual(observation.state, REJECTED)

    def test_required_signal_names_are_case_sensitive_contract_values(self) -> None:
        output = NavigationHealthAggregator().evaluate(event(lidar_fresh=False, required_signals=("lidar",)))
        self.assertEqual(output.overall_state, LOCALIZATION_SUSPECT)


if __name__ == "__main__":
    unittest.main()
