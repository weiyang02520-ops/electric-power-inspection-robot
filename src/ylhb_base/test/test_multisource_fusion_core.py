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
        self.assertEqual(second.fusion_mode, "INITIALIZING")

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
        fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(timestamp=1.0)))
        output = fusion.process(FusionInput(2.0, local_odom=odom(0.0, timestamp=2.0), gnss=gnss(longitude=120.0002, timestamp=2.0)))
        self.assertEqual(output.fusion_mode, REJECTED_UPDATE)
        self.assertIn("GNSS_OUTLIER", output.reasons)

    def test_lidar_jump_is_rejected(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), amcl_pose=lidar(0.0, timestamp=1.0)))
        output = fusion.process(FusionInput(2.0, local_odom=odom(0.0, timestamp=2.0), amcl_pose=lidar(6.0, timestamp=2.0)))
        self.assertEqual(output.fusion_mode, REJECTED_UPDATE)
        self.assertIn("AMCL_OUTLIER", output.reasons)

    def test_correction_step_is_bounded(self) -> None:
        fusion = core(max_correction_step=0.2, gnss_residual_gate=8.0)
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(timestamp=1.0)))
        output = fusion.process(FusionInput(2.0, local_odom=odom(0.0, timestamp=2.0), gnss=gnss(longitude=120.00001, timestamp=2.0)))
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
        self.assertIn("GNSS_ALIGNMENT_UNAVAILABLE", output.reasons)

    def test_amcl_anchor_is_not_overridden_by_missing_gnss_alignment(self) -> None:
        fusion = MultisourceFusionCore(FusionConfig(gnss_reference=REFERENCE, max_correction_step=10.0))
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(
            FusionInput(
                1.0,
                local_odom=odom(0.0, timestamp=1.0),
                gnss=gnss(longitude=120.00001, timestamp=1.0),
                amcl_pose=lidar(20.0, 15.0, timestamp=1.0),
            )
        )
        self.assertEqual(output.fusion_mode, LIDAR_AIDED)
        self.assertEqual(output.accepted_source, "AMCL")
        self.assertNotEqual(output.fusion_mode, WAITING_ALIGNMENT)
        self.assertIn("GNSS_ALIGNMENT_UNAVAILABLE", output.reasons)

    def test_anchored_system_keeps_operating_when_new_gnss_lacks_alignment(self) -> None:
        fusion = MultisourceFusionCore(FusionConfig(gnss_reference=REFERENCE, max_correction_step=10.0))
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        anchored = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), amcl_pose=lidar(20.0, 15.0, timestamp=1.0)))
        output = fusion.process(FusionInput(2.0, local_odom=odom(0.0, timestamp=2.0), gnss=gnss(longitude=120.00001, timestamp=2.0)))
        self.assertEqual(anchored.fusion_mode, LIDAR_AIDED)
        self.assertNotEqual(output.fusion_mode, WAITING_ALIGNMENT)
        self.assertIsNone(output.accepted_source)
        self.assertAlmostEqual(output.map_pose.x, anchored.map_pose.x, places=6)  # type: ignore[union-attr]
        self.assertAlmostEqual(output.map_pose.y, anchored.map_pose.y, places=6)  # type: ignore[union-attr]
        self.assertIn("GNSS_ALIGNMENT_UNAVAILABLE", output.reasons)

    def test_scan_match_anchor_is_not_overridden_by_missing_gnss_alignment(self) -> None:
        fusion = MultisourceFusionCore(FusionConfig(gnss_reference=REFERENCE))
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(
            FusionInput(
                1.0,
                local_odom=odom(0.0, timestamp=1.0),
                gnss=gnss(timestamp=1.0),
                scan_match_pose=lidar(12.0, 4.0, 0.3, 1.0, source="scan_match"),
            )
        )
        self.assertEqual(output.fusion_mode, LIDAR_AIDED)
        self.assertEqual(output.accepted_source, "SCAN_MATCH")
        self.assertNotEqual(output.fusion_mode, WAITING_ALIGNMENT)

    def test_missing_alignment_gnss_never_updates_map_pose(self) -> None:
        fusion = MultisourceFusionCore(FusionConfig(gnss_reference=REFERENCE))
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        before = fusion.process(FusionInput(1.0, local_odom=odom(1.0, timestamp=1.0)))
        output = fusion.process(FusionInput(2.0, local_odom=odom(1.0, timestamp=2.0), gnss=gnss(longitude=120.0001, timestamp=2.0)))
        self.assertAlmostEqual(output.map_pose.x, before.map_pose.x, places=6)  # type: ignore[union-attr]
        self.assertIsNone(output.accepted_source)
        self.assertIn("GNSS_ALIGNMENT_UNAVAILABLE", output.reasons)

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

    def test_duplicate_odom_is_ignored_without_time_reversal(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(0.1, local_odom=odom(0.0, timestamp=0.0)))
        self.assertNotIn("LOCAL_ODOM_TIME_REVERSED", output.reasons)
        self.assertNotIn("REJECTED_UPDATE", output.reasons)

    def test_first_amcl_global_anchor_can_be_far_from_odom_origin(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), amcl_pose=lidar(20.0, 15.0, timestamp=1.0)))
        self.assertTrue(output.global_anchored)
        self.assertAlmostEqual(output.map_pose.x, 20.0, places=6)  # type: ignore[union-attr]
        self.assertAlmostEqual(output.map_pose.y, 15.0, places=6)  # type: ignore[union-attr]

    def test_first_scan_match_global_anchor_has_priority_over_amcl(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(
            FusionInput(
                1.0,
                local_odom=odom(0.0, timestamp=1.0),
                scan_match_pose=lidar(12.0, 4.0, 0.3, 1.0, source="scan_match"),
                amcl_pose=lidar(20.0, 15.0, timestamp=1.0),
            )
        )
        self.assertEqual(output.accepted_source, "SCAN_MATCH")
        self.assertAlmostEqual(output.map_pose.x, 12.0, places=6)  # type: ignore[union-attr]

    def test_first_aligned_gnss_anchor_can_establish_global_translation(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        output = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(longitude=120.0001, timestamp=1.0)))
        self.assertEqual(output.accepted_source, "GNSS")
        self.assertGreater(output.map_pose.x, 8.0)  # type: ignore[union-attr]

    def test_post_anchor_large_jump_is_rejected_by_residual_gate(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), amcl_pose=lidar(20.0, 15.0, timestamp=1.0)))
        output = fusion.process(FusionInput(2.0, local_odom=odom(0.0, timestamp=2.0), amcl_pose=lidar(100.0, 100.0, timestamp=2.0)))
        self.assertEqual(output.fusion_mode, REJECTED_UPDATE)
        self.assertIn("AMCL_OUTLIER", output.reasons)

    def test_gnss_correction_reduces_position_but_not_yaw_uncertainty(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=gnss(timestamp=1.0)))
        before = fusion.process(FusionInput(2.0, local_odom=odom(0.0, timestamp=2.0)))
        after = fusion.process(FusionInput(2.0, local_odom=odom(0.0, timestamp=2.0), gnss=gnss(longitude=120.00001, timestamp=2.0)))
        self.assertLess(after.position_uncertainty, before.position_uncertainty)
        self.assertAlmostEqual(after.yaw_uncertainty, before.yaw_uncertainty, places=9)
        self.assertAlmostEqual(after.map_pose.yaw, before.map_pose.yaw, places=9)  # type: ignore[union-attr]

    def test_lidar_correction_can_reduce_yaw_uncertainty(self) -> None:
        fusion = core()
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), amcl_pose=lidar(0.0, timestamp=1.0)))
        before = fusion.process(FusionInput(2.0, local_odom=odom(0.0, timestamp=2.0)))
        after = fusion.process(FusionInput(2.0, local_odom=odom(0.0, timestamp=2.0), scan_match_pose=lidar(0.1, yaw=0.1, timestamp=2.0, source="scan_match")))
        self.assertLess(after.yaw_uncertainty, before.yaw_uncertainty)

    def test_duplicate_gnss_for_ten_ticks_is_not_rejected(self) -> None:
        fusion = core(freshness_timeout=100.0)
        measurement = gnss(timestamp=1.0)
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        first = fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), gnss=measurement))
        for tick in range(2, 12):
            output = fusion.process(FusionInput(float(tick), local_odom=odom(0.0, timestamp=float(tick)), gnss=measurement))
            self.assertNotEqual(output.fusion_mode, REJECTED_UPDATE)
            self.assertNotIn("GNSS_OLD_MEASUREMENT", output.reasons)
        self.assertEqual(first.accepted_source, "GNSS")

    def test_duplicate_amcl_is_not_rejected(self) -> None:
        fusion = core(freshness_timeout=100.0)
        measurement = lidar(1.0, timestamp=1.0)
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), amcl_pose=measurement))
        output = fusion.process(FusionInput(2.0, local_odom=odom(0.0, timestamp=2.0), amcl_pose=measurement))
        self.assertNotEqual(output.fusion_mode, REJECTED_UPDATE)
        self.assertNotIn("AMCL_OLD_MEASUREMENT", output.reasons)

    def test_duplicate_scan_to_map_is_not_rejected(self) -> None:
        fusion = core(freshness_timeout=100.0)
        measurement = lidar(1.0, timestamp=1.0, source="scan_match")
        fusion.process(FusionInput(0.0, local_odom=odom(0.0, timestamp=0.0)))
        fusion.process(FusionInput(1.0, local_odom=odom(0.0, timestamp=1.0), scan_match_pose=measurement))
        output = fusion.process(FusionInput(2.0, local_odom=odom(0.0, timestamp=2.0), scan_match_pose=measurement))
        self.assertNotEqual(output.fusion_mode, REJECTED_UPDATE)
        self.assertNotIn("SCAN_MATCH_OLD_MEASUREMENT", output.reasons)

    def test_mixed_frequency_ten_second_simulation_is_stable(self) -> None:
        fusion = core(freshness_timeout=2.0)
        outputs = []
        for tick in range(1, 101):
            now = tick * 0.1
            current_odom = odom(now * 0.1, timestamp=now)
            current_gnss = gnss(timestamp=now) if tick % 10 == 0 else None
            current_amcl = lidar(now * 0.1, timestamp=now) if tick % 2 == 0 else None
            output = fusion.process(FusionInput(now, current_odom, current_gnss, amcl_pose=current_amcl))
            outputs.append(output)
            self.assertNotIn("LOCAL_ODOM_TIME_REVERSED", output.reasons)
            self.assertNotEqual(output.fusion_mode, REJECTED_UPDATE)
            self.assertTrue(math.isfinite(output.position_uncertainty))
            self.assertTrue(math.isfinite(output.yaw_uncertainty))
        self.assertEqual(fusion._last_absolute_timestamps["GNSS"], 10.0)
        self.assertIsNotNone(outputs[-1].map_pose)


if __name__ == "__main__":
    unittest.main()
