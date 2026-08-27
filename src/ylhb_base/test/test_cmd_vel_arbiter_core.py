import math
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from cmd_vel_arbiter_core import (  # noqa: E402
    NAV,
    NONE,
    RECOVERY,
    ArbiterConfig,
    ArbiterInput,
    CmdVelArbiter,
    TwistCommand,
)


def nav_cmd() -> TwistCommand:
    return TwistCommand(0.4, 0.1)


def recovery_cmd() -> TwistCommand:
    return TwistCommand(0.0, 0.2)


class CmdVelArbiterTests(unittest.TestCase):
    def test_nominal_forwards_navigation_after_initial_guard(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.2))
        first = arbiter.process(ArbiterInput(0.0, nav_cmd(), 0.0, navigation_state="NOMINAL"))
        second = arbiter.process(ArbiterInput(0.3, nav_cmd(), 0.3, navigation_state="NOMINAL"))
        self.assertEqual(first.source, NAV)
        self.assertTrue(first.guard_active)
        self.assertEqual(second.command, nav_cmd())

    def test_recovering_forwards_only_recovery(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0))
        arbiter.process(ArbiterInput(0.0, nav_cmd(), 0.0))
        arbiter.process(ArbiterInput(1.0, nav_cmd(), 1.0, recovery_cmd(), 1.0, "RECOVERING"))
        output = arbiter.process(ArbiterInput(1.1, nav_cmd(), 1.1, recovery_cmd(), 1.1, "RECOVERING"))
        self.assertEqual(output.source, RECOVERY)
        self.assertEqual(output.command, recovery_cmd())

    def test_recovering_does_not_forward_navigation(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0))
        output = arbiter.process(ArbiterInput(0.0, nav_cmd(), 0.0, None, None, "RECOVERING"))
        self.assertEqual(output.command, TwistCommand())
        self.assertEqual(output.reason, "RECOVERY_SOURCE_STALE")

    def test_failed_and_manual_states_are_zero(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0))
        for state in ("FAILED", "MANUAL_REQUIRED"):
            output = arbiter.process(ArbiterInput(0.0, nav_cmd(), 0.0, recovery_cmd(), 0.0, state))
            self.assertEqual(output.command, TwistCommand(), state)
        manual = arbiter.process(ArbiterInput(0.1, nav_cmd(), 0.1, recovery_cmd(), 0.1, "RECOVERING", True))
        self.assertEqual(manual.command, TwistCommand())
        self.assertEqual(manual.reason, "MANUAL_TAKEOVER")

    def test_stale_navigation_source_is_zero(self) -> None:
        output = CmdVelArbiter().process(ArbiterInput(1.0, nav_cmd(), 0.0, navigation_state="NOMINAL"))
        self.assertEqual(output.command, TwistCommand())
        self.assertEqual(output.source, NONE)

    def test_stale_recovery_source_is_zero(self) -> None:
        output = CmdVelArbiter().process(ArbiterInput(1.0, nav_cmd(), 0.0, recovery_cmd(), 0.0, "RECOVERING"))
        self.assertEqual(output.command, TwistCommand())
        self.assertEqual(output.reason, "RECOVERY_SOURCE_STALE")

    def test_source_switch_inserts_zero_guard(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.5))
        arbiter.process(ArbiterInput(0.0, nav_cmd(), 0.0))
        arbiter.process(ArbiterInput(0.6, nav_cmd(), 0.6))
        switched = arbiter.process(ArbiterInput(0.7, nav_cmd(), 0.7, recovery_cmd(), 0.7, "RECOVERING"))
        self.assertTrue(switched.guard_active)
        self.assertEqual(switched.command, TwistCommand())
        resumed = arbiter.process(ArbiterInput(1.3, nav_cmd(), 1.3, recovery_cmd(), 1.3, "RECOVERING"))
        self.assertEqual(resumed.source, RECOVERY)
        self.assertEqual(resumed.command, recovery_cmd())

    def test_recovered_switches_back_to_navigation_with_guard(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.2))
        arbiter.process(ArbiterInput(0.0, nav_cmd(), 0.0))
        arbiter.process(ArbiterInput(0.3, nav_cmd(), 0.3, recovery_cmd(), 0.3, "RECOVERING"))
        output = arbiter.process(ArbiterInput(0.6, nav_cmd(), 0.6, recovery_cmd(), 0.6, "RECOVERED"))
        self.assertTrue(output.guard_active)
        self.assertEqual(output.command, TwistCommand())

    def test_nonfinite_command_is_never_forwarded(self) -> None:
        output = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0)).process(
            ArbiterInput(0.0, TwistCommand(math.nan, 0.0), 0.0)
        )
        self.assertEqual(output.command, TwistCommand())

    def test_manual_takeover_does_not_change_to_recovery_source(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0))
        arbiter.process(ArbiterInput(0.0, nav_cmd(), 0.0))
        output = arbiter.process(ArbiterInput(1.0, nav_cmd(), 1.0, recovery_cmd(), 1.0, "RECOVERING", True))
        self.assertEqual(output.source, NONE)


if __name__ == "__main__":
    unittest.main()
