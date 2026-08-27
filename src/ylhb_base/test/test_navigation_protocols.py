import ast
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
LAUNCH = ROOT / "launch" / "dg_navigation_integration.launch.py"
CMAKE = ROOT / "CMakeLists.txt"


class NavigationProtocolAndLaunchTests(unittest.TestCase):
    def test_all_new_python_files_parse_without_ros_imports_at_module_scope(self) -> None:
        for path in (
            SCRIPTS / "navigation_health_core.py",
            SCRIPTS / "navigation_health_node.py",
            SCRIPTS / "cmd_vel_arbiter_core.py",
            SCRIPTS / "cmd_vel_arbiter_node.py",
            LAUNCH,
        ):
            ast.parse(path.read_text(encoding="utf-8"))

    def test_launch_contains_all_integration_nodes(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        for executable in (
            "lidar_robust_node",
            "gnss_quality_node",
            "scan_map_relocalization_node",
            "active_relocalization_node",
            "navigation_health_node",
            "cmd_vel_arbiter_node",
        ):
            self.assertIn(f'executable="{executable}"', text)

    def test_launch_has_explicit_topic_boundaries(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        for topic in (
            "/scan",
            "/odom",
            "/gps/fix",
            "/gps/rtk_status",
            "/dg/lidar/quality",
            "/dg/gnss/quality",
            "/dg/relocalization/seed",
            "/dg/relocalization/match_quality",
            "/cmd_vel_nav",
            "/cmd_vel_recovery",
            "/cmd_vel",
            "/dg/navigation/status",
        ):
            self.assertIn(topic, text)

    def test_launch_defaults_nav2_disabled_and_remaps_cmd_vel(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('DeclareLaunchArgument("enable_nav2", default_value="false"', text)
        self.assertIn('SetRemap(src="/cmd_vel", dst="/cmd_vel_nav")', text)

    def test_cmake_installs_new_wrappers_and_registers_tests(self) -> None:
        text = CMAKE.read_text(encoding="utf-8")
        for script in (
            "navigation_health_core.py",
            "navigation_health_node.py",
            "cmd_vel_arbiter_core.py",
            "cmd_vel_arbiter_node.py",
        ):
            self.assertIn(f"scripts/{script}", text)
        for test_name in (
            "test_navigation_health_core",
            "test_cmd_vel_arbiter_core",
            "test_navigation_integration_core",
            "test_navigation_protocols",
        ):
            self.assertIn(test_name, text)

    def test_default_bringup_does_not_include_competition_launch(self) -> None:
        text = (ROOT / "launch" / "bringup.launch.py").read_text(encoding="utf-8")
        self.assertNotIn("dg_navigation_integration", text)

    def test_active_relocalization_default_cmd_vel_remains_compatible(self) -> None:
        text = (SCRIPTS / "active_relocalization_node.py").read_text(encoding="utf-8")
        self.assertIn('"cmd_vel_topic": "/cmd_vel"', text)

    def test_gnss_is_not_wired_into_ekf_by_integration_launch(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertNotIn("ekf.yaml", text)
        self.assertNotIn("NavSatTransform", text)

    def test_poc_publishers_and_health_parser_share_real_field_names(self) -> None:
        lidar = (SCRIPTS / "lidar_robust_features.py").read_text(encoding="utf-8")
        gnss = (SCRIPTS / "gnss_quality_node.py").read_text(encoding="utf-8")
        scan_map = (SCRIPTS / "scan_map_relocalization_node.py").read_text(encoding="utf-8")
        active = (SCRIPTS / "active_relocalization_node.py").read_text(encoding="utf-8")
        health = (SCRIPTS / "navigation_health_node.py").read_text(encoding="utf-8")
        for field in ("valid_ratio", "temporal_status"):
            self.assertIn(field, lidar)
        for field in ("current_state", "decision"):
            self.assertIn(field, gnss)
        for field in ("accepted", "score", "mean_distance", "inlier_ratio", "candidate_x", "candidate_y", "candidate_yaw", "reason"):
            self.assertIn(field, scan_map)
        for field in ("state", "attempt_id", "request_candidate", "seed_source"):
            self.assertIn(field, active)
        for field in ("gnss_state", "lidar_state", "amcl_state", "scan_match_state", "overall_state", "reasons", "timestamp"):
            self.assertIn(field, health)


if __name__ == "__main__":
    unittest.main()
