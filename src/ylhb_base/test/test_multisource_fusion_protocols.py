import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
LAUNCH = ROOT / "launch" / "dg_navigation_integration.launch.py"
CMAKE = ROOT / "CMakeLists.txt"


class MultisourceFusionProtocolTests(unittest.TestCase):
    def test_new_python_modules_parse_without_ros_imports_at_module_scope(self) -> None:
        for name in ("multisource_fusion_core.py", "multisource_fusion_node.py"):
            ast.parse((SCRIPTS / name).read_text(encoding="utf-8"))

    def test_core_defines_required_modes_and_outputs(self) -> None:
        text = (SCRIPTS / "multisource_fusion_core.py").read_text(encoding="utf-8")
        for value in (
            "LOCAL_ODOM",
            "GNSS_POSITION",
            "LIDAR_AIDED",
            "INITIALIZING",
            "GNSS_AIDED",
            "DEAD_RECKONING",
            "REJECTED_UPDATE",
            "WAITING_ALIGNMENT",
            "position_uncertainty",
        ):
            self.assertIn(value, text)

    def test_wrapper_uses_real_repository_topics(self) -> None:
        text = (SCRIPTS / "multisource_fusion_node.py").read_text(encoding="utf-8")
        for topic in (
            '"/odom"',
            '"/dg/gnss/accepted_fix"',
            '"/dg/gnss/quality"',
            '"/amcl_pose"',
            '"/scan_match_pose"',
            '"/dg/relocalization/match_quality"',
            '"/dg/fusion/odom"',
            '"/dg/fusion/pose"',
            '"/dg/fusion/status"',
        ):
            self.assertIn(topic, text)

    def test_wrapper_does_not_publish_tf(self) -> None:
        text = (SCRIPTS / "multisource_fusion_node.py").read_text(encoding="utf-8")
        self.assertIn('self.declare_parameter("publish_tf", False)', text)
        self.assertNotIn("TransformBroadcaster", text)

    def test_wrapper_tracks_and_consumes_message_sequences(self) -> None:
        text = (SCRIPTS / "multisource_fusion_node.py").read_text(encoding="utf-8")
        for field in (
            "_odom_sequence",
            "_gnss_sequence",
            "_amcl_sequence",
            "_scan_match_sequence",
            "_odom_consumed_sequence",
            "_gnss_consumed_sequence",
            "_amcl_consumed_sequence",
            "_scan_match_consumed_sequence",
        ):
            self.assertIn(field, text)

    def test_launch_has_optional_multisource_fusion_switch(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"enable_multisource_fusion"', text)
        self.assertIn('default_value="true"', text)
        self.assertIn('executable="multisource_fusion_node"', text)
        self.assertIn('condition=IfCondition(enable_multisource_fusion)', text)
        for argument in (
            "gnss_origin_latitude",
            "gnss_origin_longitude",
            "gnss_origin_altitude",
            "map_enu_yaw",
            "map_enu_offset_x",
            "map_enu_offset_y",
        ):
            self.assertIn(f'"{argument}"', text)

    def test_launch_alignment_parameters_are_explicit_float_values(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("from launch_ros.parameter_descriptions import ParameterValue", text)
        for argument in (
            "gnss_origin_latitude",
            "gnss_origin_longitude",
            "gnss_origin_altitude",
            "map_enu_yaw",
            "map_enu_offset_x",
            "map_enu_offset_y",
        ):
            self.assertIn(f'ParameterValue({argument}, value_type=float)', text)

    def test_launch_keeps_fusion_as_side_channel(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"publish_tf": False', text)
        self.assertIn('"fusion_odom_topic": "/dg/fusion/odom"', text)
        self.assertIn('"fusion_status_topic": "/dg/fusion/status"', text)
        self.assertNotIn('"map_to_odom_topic"', text)

    def test_cmake_installs_wrapper_and_registers_new_tests(self) -> None:
        text = CMAKE.read_text(encoding="utf-8")
        self.assertIn("scripts/multisource_fusion_core.py", text)
        self.assertIn("scripts/multisource_fusion_node.py", text)
        self.assertIn("test_multisource_fusion_core", text)
        self.assertIn("test_multisource_fusion_protocols", text)

    def test_default_ekf_and_bringup_are_not_wired_to_fusion(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        bringup = (ROOT / "launch" / "bringup.launch.py").read_text(encoding="utf-8")
        self.assertNotIn("ekf.yaml", launch)
        self.assertNotIn("multisource_fusion", bringup)
        self.assertNotIn("TransformBroadcaster", launch)


if __name__ == "__main__":
    unittest.main()
