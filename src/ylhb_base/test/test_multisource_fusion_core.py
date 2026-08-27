import math
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from multisource_fusion_core import (  # noqa: E402
    DEAD_RECKONING,
    GNSS_AIDED,
    GOOD,
    LIDAR_AIDED,
    REJECTED_UPDATE,
    WAITING_ALIGNMENT,
    Alignment2D,
    ENUPoint,
    FusionConfig,
    FusionInput,
    GeodeticPoint,
    GeodeticReference,
    GnssPosition,
    MapPoseMeasurement,
    MultisourceFusionCore,
    Pose2D,
    estimate_alignment,
    geodetic_to_ecef,
    geodetic_to_enu,
)


REFERENCE = GeodeticReference(30.0, 120.0, 10.0)


def odom(x: float, y: float = 0.0, yaw: float = 0.0, timestamp: float = 0.0) -> Pose2D:
    return Pose2D(x, y, yaw, timestamp)


def gnss(
    longitude: float = 120.0,
    latitude: float = 30.0,
    timestamp: float = 0.0,
    state: str = GOOD,
    fresh: bool = True,
    accepted: bool = True,
) -> GnssPosition:
    return GnssPosition(latitude, longitude, 10.0, timestamp, state, fresh, accepted)


def lidar(
    x: float,
    y: float = 0.0,
    yaw: float = 0.0,
    timestamp: float = 0.0,
    state: str = GOOD,
    fresh: bool = True,
    accepted: bool = True,
    source: str = "amcl",
) -> MapPoseMeasurement:
    return MapPoseMeasurement(odom(x, y, yaw, timestamp), timestamp, state, fresh, accepted, source)


def core(**changes: object) -> MultisourceFusionCore:
    values = dict(
        gnss_reference=REFERENCE,
        alignment=Alignment2D(0.0, 0.0, 0.0),
        max_correction_step=10.0,
        gnss_residual_gate=8.0,
        lidar_residual_gate=5.0,
    )
    values.update(changes)
    return MultisourceFusionCore(FusionConfig(**values))


class GeodeticAndAlignmentTests(unittest.TestCase):
    def test_geodetic_reference_is_zero_enu(self) -> None:
        point = geodetic_to_enu(GeodeticPoint(30.0, 120.0, 10.0), REFERENCE)
        self.assertAlmostEqual(point.east, 0.0, places=6)
        self.assertAlmostEqual(point.north, 0.0, places=6)
        self.assertAlmostEqual(point.up, 0.0, places=6)

    def test_geodetic_to_ecef_is_finite(self) -> None:
        self.assertTrue(all(math.isfinite(value) for value in geodetic_to_ecef(GeodeticPoint(30.0, 120.0, 10.0))))

    def test_small_longitude_change_maps_to_east(self) -> None:
        point = geodetic_to_enu(GeodeticPoint(30.0, 120.00001, 10.0), REFERENCE)
        self.assertGreater(point.east, 0.8)
        self.assertLess(abs(point.north), 0.1)

    def test_alignment_estimator_recovers_rigid_transform(self) -> None:
        expected = Alignment2D(0.4, 3.0, -2.0)
        enu_points = [ENUPoint(0.0, 0.0, 0.0), ENUPoint(2.0, 0.0, 0.0), ENUPoint(0.0, 3.0, 0.0)]
        pairs = [
            (point, Pose2D(*expected.apply(point), 0.0, float(index)))
            for index, point in enumerate(enu_points)
        ]
        estimated = estimate_alignment(pairs)
        self.assertIsNotNone(estimated)
        self.assertAlmostEqual(estimated.map_enu_yaw, expected.map_enu_yaw, places=6)  # type: ignore[union-attr]
        self.assertAlmostEqual(estimated.offset_x, expected.offset_x, places=6)  # type: ignore[union-attr]
        self.assertAlmostEqual(estimated.offset_y, expected.offset_y, places=6)  # type: ignore[union-attr]

    def test_alignment_requires_two_non_degenerate_samples(self) -> None:
        self.assertIsNone(estimate_alignment([(ENUPoint(0.0, 0.0, 0.0), odom(1.0, 1.0))]))
        self.assertIsNone(
            estimate_alignment(
                [
                    (ENUPoint(1.0, 1.0, 0.0), odom(1.0, 1.0)),
                    (ENUPoint(1.0, 1.0, 0.0), odom(2.0, 2.0)),
                ]
            )
        )


class FusionCoreTests(unittest.TestCase):
    def test_local_odometry_initializes_and_propagates(self) -> None:
        fusion = core()
        first = fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        second = fusion.process(FusionInput(1.0, local_odom=odom(1.0, timestamp=1.0)))
        self.assertIsNotNone(first.map_pose)
        self.assertAlmostEqual(second.map_pose.x, 1.0, places=6)  # type: ignore[union-attr]
        self.assertEqual(second.fusion_mode, "NOMINAL")

    def test_gnss_good_corrects_xy_without_changing_yaw(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(longitude=120.00001, timestamp=1.0)))
        self.assertEqual(output.fusion_mode, GNSS_AIDED)
        self.assertEqual(output.accepted_source, "GNSS")
        self.assertGreater(output.map_pose.x, 0.8)  # type: ignore[union-attr]
        self.assertAlmostEqual(output.map_pose.yaw, 0.0, places=6)  # type: ignore[union-attr]

    def test_gnss_degraded_is_allowed_as_a_lower_confidence_correction(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(longitude=120.00001, timestamp=1.0, state="DEGRADED")))
        self.assertEqual(output.accepted_source, "GNSS")

    def test_gnss_and_lidar_can_correct_same_cycle_without_double_counting_lidar_sources(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(
            FusionInput(
                1.0,
                local_odom=odom(0.0, timestamp=1.0),
                gnss=gnss(longitude=120.000005, timestamp=1.0),
                scan_match_pose=lidar(0.2, timestamp=1.0, source="scan_match"),
                amcl_pose=lidar(0.3, timestamp=1.0, source="amcl"),
            )
        )
        self.assertEqual(output.accepted_source, "SCAN_MATCH")
        self.assertTrue(output.updated)

    def test_gnss_rejected_does_not_update(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(longitude=120.00001, timestamp=1.0, state="REJECTED", accepted=False)))
        self.assertIsNone(output.accepted_source)
        self.assertEqual(output.map_pose.x, 0.0)  # type: ignore[union-attr]

    def test_gnss_stale_does_not_update(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(3.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(longitude=120.00001, timestamp=0.0, fresh=False)))
        self.assertIsNone(output.accepted_source)
        self.assertIn("GNSS_STALE", output.reasons)

    def test_lidar_pose_corrects_xy_yaw(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), amcl_pose=lidar(1.0, yaw=0.2, timestamp=1.0)))
        self.assertEqual(output.fusion_mode, LIDAR_AIDED)
        self.assertEqual(output.accepted_source, "AMCL")
        self.assertAlmostEqual(output.map_pose.x, 1.0, places=6)  # type: ignore[union-attr]
        self.assertAlmostEqual(output.map_pose.yaw, 0.2, places=6)  # type: ignore[union-attr]

    def test_lidar_degraded_does_not_update(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), amcl_pose=lidar(1.0, timestamp=1.0, state="DEGRADED", accepted=False)))
        self.assertIsNone(output.accepted_source)
        self.assertEqual(output.map_pose.x, 0.0)  # type: ignore[union-attr]

    def test_scan_match_has_priority_over_amcl_and_is_not_double_counted(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(
            FusionInput(
                1.0,
                local_odom=odom(0.0, timestamp=1.0),
                scan_match_pose=lidar(1.0, timestamp=1.0, source="scan_match"),
                amcl_pose=lidar(4.0, timestamp=1.0, source="amcl"),
            )
        )
        self.assertEqual(output.accepted_source, "SCAN_MATCH")
        self.assertAlmostEqual(output.map_pose.x, 1.0, places=6)  # type: ignore[union-attr]

    def test_gnss_jump_is_rejected(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(longitude=120.0002, timestamp=1.0)))
        self.assertEqual(output.fusion_mode, REJECTED_UPDATE)
        self.assertIn("GNSS_OUTLIER", output.reasons)

    def test_lidar_jump_is_rejected(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), amcl_pose=lidar(6.0, timestamp=1.0)))
        self.assertEqual(output.fusion_mode, REJECTED_UPDATE)
        self.assertIn("AMCL_OUTLIER", output.reasons)

    def test_correction_step_is_bounded(self) -> None:
        fusion = core(max_correction_step=0.2, gnss_residual_gate=8.0)
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(longitude=120.00001, timestamp=1.0)))
        self.assertAlmostEqual(output.map_pose.x, 0.2, places=3)  # type: ignore[union-attr]

    def test_gnss_outage_continues_dead_reckoning(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(1.0, timestamp=1.0), gnss=gnss(timestamp=0.0, fresh=False)))
        self.assertEqual(output.fusion_mode, DEAD_RECKONING)
        self.assertAlmostEqual(output.map_pose.x, 1.0, places=6)  # type: ignore[union-attr]

    def test_lidar_degradation_still_allows_gnss(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(
            FusionInput(
                1.0,
                local_odom=odom(0.0, timestamp=1.0),
                gnss=gnss(longitude=120.00001, timestamp=1.0),
                amcl_pose=lidar(1.0, timestamp=1.0, state="DEGRADED", accepted=False),
            )
        )
        self.assertEqual(output.accepted_source, "GNSS")
        self.assertEqual(output.fusion_mode, GNSS_AIDED)

    def test_concurrent_degradation_is_continuous_and_uncertainty_grows(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        before = fusion.process(FusionInput(1.0, local_odom=odom(1.0, timestamp=1.0)))
        after = fusion.process(
            FusionInput(
                2.0,
                local_odom=odom(2.0, timestamp=2.0),
                gnss=gnss(timestamp=0.0, fresh=False),
                amcl_pose=lidar(2.0, timestamp=2.0, state="DEGRADED", accepted=False),
            )
        )
        self.assertEqual(after.fusion_mode, DEAD_RECKONING)
        self.assertIsNotNone(after.map_pose)
        self.assertGreater(after.position_uncertainty, before.position_uncertainty)

    def test_recovery_correction_reduces_uncertainty(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        before = fusion.process(FusionInput(1.0, local_odom=odom(1.0, timestamp=1.0)))
        after = fusion.process(FusionInput(2.0, local_odom=odom(1.0, timestamp=2.0), gnss=gnss(timestamp=2.0)))
        self.assertLess(after.position_uncertainty, before.position_uncertainty)

    def test_gnss_requires_alignment(self) -> None:
        fusion = MultisourceFusionCore(FusionConfig(gnss_reference=REFERENCE))
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(longitude=120.00001, timestamp=1.0)))
        self.assertEqual(output.fusion_mode, WAITING_ALIGNMENT)
        self.assertIsNone(output.accepted_source)

    def test_alignment_completion_enables_gnss(self) -> None:
        fusion = MultisourceFusionCore(FusionConfig(gnss_reference=REFERENCE, max_correction_step=10.0))
        fusion.set_alignment(Alignment2D(0.0, 0.0, 0.0))
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(longitude=120.00001, timestamp=1.0)))
        self.assertEqual(output.accepted_source, "GNSS")

    def test_nan_measurement_never_enters_output(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(timestamp=1.0, latitude=math.nan)))
        self.assertTrue(all(math.isfinite(value) for value in (output.position_uncertainty, output.yaw_uncertainty)))
        self.assertIsNone(output.accepted_source)

    def test_old_absolute_measurement_is_rejected(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(timestamp=1.0)))
        output = fusion.process(FusionInput(2.0, local_odom=odom(0.0, timestamp=2.0), gnss=gnss(timestamp=0.5)))
        self.assertIn("GNSS_OLD_MEASUREMENT", output.reasons)

    def test_local_odom_time_reversal_is_rejected(self) -> None:
        fusion = core()
        fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0)))
        output = fusion.process(FusionInput(2.0, local_odom=odom(1.0, timestamp=0.5)))
        self.assertIn("LOCAL_ODOM_TIME_REVERSED", output.reasons)


if __name__ == "__main__":
    unittest.main()
