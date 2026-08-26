import math
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from lidar_robust_features import (  # noqa: E402
    RobustFeatureConfig,
    ScanFrame,
    compensate_previous_points,
    process_scan,
)


def vertical_wall_scan(yaw: float = 0.0, count: int = 21) -> ScanFrame:
    angle_min = -1.0
    angle_increment = 2.0 / (count - 1)
    ranges = []
    for index in range(count):
        angle = angle_min + index * angle_increment
        ranges.append(2.0 / math.cos(angle + yaw))
    return ScanFrame(ranges, angle_min, angle_increment, 0.05, 8.0)


class LidarRobustFeatureTests(unittest.TestCase):
    def test_normal_wall_produces_valid_points_and_line_feature(self) -> None:
        result = process_scan(vertical_wall_scan())
        self.assertEqual(result.raw_point_count, 21)
        self.assertEqual(result.quality["valid_point_count"], 21)
        self.assertEqual(result.quality["outlier_count"], 0)
        self.assertGreater(result.quality["angular_coverage"], 1.0)
        self.assertTrue(any(feature["type"] == "line" for feature in result.features))

    def test_nan_inf_range_and_self_points_are_rejected(self) -> None:
        frame = ScanFrame([float("nan"), float("inf"), 0.01, 9.0, 0.1, 1.0], 0.0, 0.1, 0.05, 8.0)
        result = process_scan(frame)
        self.assertEqual(result.quality["raw_point_count"], 6)
        self.assertEqual(result.quality["valid_point_count"], 1)
        self.assertEqual(result.quality["outlier_count"], 5)

    def test_isolated_jump_is_filtered(self) -> None:
        ranges = [2.0] * 9
        ranges[4] = 3.0
        result = process_scan(ScanFrame(ranges, -0.4, 0.1, 0.05, 8.0))
        self.assertNotIn(4, [point.index for point in result.points])
        self.assertEqual(result.quality["outlier_count"], 1)

    def test_static_environment_matches_after_no_motion(self) -> None:
        first = vertical_wall_scan()
        result = process_scan(vertical_wall_scan(), first, (0.0, 0.0, 0.0))
        self.assertEqual(result.quality["dynamic_candidate_count"], 0)
        self.assertAlmostEqual(result.quality["temporal_match_ratio"], 1.0)

    def test_translation_motion_compensation_preserves_static_wall(self) -> None:
        previous = vertical_wall_scan(count=81)
        current_ranges = [1.9 / math.cos(-1.0 + i * 0.025) for i in range(81)]
        current = ScanFrame(current_ranges, -1.0, 0.025, 0.05, 8.0)
        result = process_scan(current, previous, (0.1, 0.0, 0.0))
        self.assertGreaterEqual(result.quality["temporal_match_ratio"], 0.95)
        self.assertLessEqual(result.quality["dynamic_candidate_count"], 1)

    def test_rotation_motion_compensation_preserves_static_wall(self) -> None:
        previous = vertical_wall_scan(0.0)
        current = vertical_wall_scan(0.1)
        result = process_scan(current, previous, (0.0, 0.0, 0.1))
        self.assertGreaterEqual(result.quality["temporal_match_ratio"], 0.95)
        self.assertLessEqual(result.quality["dynamic_candidate_count"], 1)

    def test_clustered_moving_target_becomes_dynamic_candidate(self) -> None:
        previous = vertical_wall_scan()
        ranges = list(previous.ranges)
        ranges[9:12] = [1.0, 1.0, 1.0]
        current = ScanFrame(ranges, previous.angle_min, previous.angle_increment, previous.range_min, previous.range_max)
        result = process_scan(current, previous, (0.0, 0.0, 0.0))
        self.assertGreaterEqual(result.quality["dynamic_candidate_count"], 3)
        self.assertGreater(result.quality["stable_point_count"], 0)

    def test_zero_valid_points_is_safe(self) -> None:
        result = process_scan(ScanFrame([float("nan"), float("inf"), 0.01], 0.0, 0.1, 0.05, 8.0))
        self.assertEqual(result.quality["valid_point_count"], 0)
        self.assertEqual(result.quality["feature_count"], 0)
        self.assertEqual(result.quality["valid_ratio"], 0.0)

    def test_feature_extraction_is_deterministic(self) -> None:
        first = process_scan(vertical_wall_scan())
        second = process_scan(vertical_wall_scan())
        self.assertEqual(first.features, second.features)
        self.assertEqual(first.quality, second.quality)

    def test_temporal_threshold_is_inclusive(self) -> None:
        config = RobustFeatureConfig(temporal_match_distance=0.12)
        previous = ScanFrame([1.0], 0.0, 0.1, 0.05, 8.0)
        current = ScanFrame([1.12], 0.0, 0.1, 0.05, 8.0)
        result = process_scan(current, previous, (0.0, 0.0, 0.0), config)
        self.assertEqual(result.quality["dynamic_candidate_count"], 0)
        self.assertAlmostEqual(result.quality["temporal_match_ratio"], 1.0)

    def test_compensation_convention_is_explicit(self) -> None:
        previous = vertical_wall_scan()
        result = process_scan(previous, previous, (0.1, 0.0, 0.0))
        compensated = compensate_previous_points(result.points, (0.1, 0.0, 0.0))
        self.assertAlmostEqual(compensated[0].x, result.points[0].x - 0.1)


if __name__ == "__main__":
    unittest.main()
