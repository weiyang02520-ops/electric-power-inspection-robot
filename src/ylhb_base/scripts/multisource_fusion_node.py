#!/usr/bin/env python3
"""ROS2 side-channel wrapper for the DG multi-source fusion POC."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from multisource_fusion_core import (  # noqa: E402
    Alignment2D,
    FusionConfig,
    FusionInput,
    GeodeticReference,
    GnssPosition,
    MapPoseMeasurement,
    MultisourceFusionCore,
    Pose2D,
)


def _finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _stamp_to_seconds(stamp: Any) -> float | None:
    try:
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value > 0.0 else None


def _yaw_from_quaternion(orientation: Any) -> float | None:
    try:
        value = math.atan2(
            2.0 * (float(orientation.w) * float(orientation.z) + float(orientation.x) * float(orientation.y)),
            1.0 - 2.0 * (float(orientation.y) ** 2 + float(orientation.z) ** 2),
        )
    except (AttributeError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _value(values: dict[str, str], key: str, default: str = "") -> str:
    return str(values.get(key, default)).strip()


def _bool_value(value: str) -> bool:
    return value.lower() in {"true", "1", "yes", "accepted", "accept"}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the DG-202611 multi-source fusion POC.")
    parser.add_argument("--node-name", default="multisource_fusion_node", help="ROS node name")
    return parser


def create_node(node_name: str) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix

    class MultisourceFusionNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self.declare_parameter("local_odom_topic", "/odom")
            self.declare_parameter("gnss_fix_topic", "/dg/gnss/accepted_fix")
            self.declare_parameter("gnss_quality_topic", "/dg/gnss/quality")
            self.declare_parameter("amcl_pose_topic", "/amcl_pose")
            self.declare_parameter("scan_match_pose_topic", "/scan_match_pose")
            self.declare_parameter("match_quality_topic", "/dg/relocalization/match_quality")
            self.declare_parameter("fusion_odom_topic", "/dg/fusion/odom")
            self.declare_parameter("fusion_pose_topic", "/dg/fusion/pose")
            self.declare_parameter("fusion_status_topic", "/dg/fusion/status")
            self.declare_parameter("publish_tf", False)
            self.declare_parameter("freshness_timeout", 2.0)
            self.declare_parameter("gnss_origin_latitude", float("nan"))
            self.declare_parameter("gnss_origin_longitude", float("nan"))
            self.declare_parameter("gnss_origin_altitude", 0.0)
            self.declare_parameter("map_enu_yaw", float("nan"))
            self.declare_parameter("map_enu_offset_x", 0.0)
            self.declare_parameter("map_enu_offset_y", 0.0)
            self.declare_parameter("max_correction_step", 0.5)
            self.declare_parameter("gnss_residual_gate", 8.0)
            self.declare_parameter("lidar_residual_gate", 5.0)
            self.declare_parameter("max_amcl_covariance", 0.5)
            self.declare_parameter("timer_period", 0.1)

            origin_values = (
                float(self.get_parameter("gnss_origin_latitude").value),
                float(self.get_parameter("gnss_origin_longitude").value),
            )
            origin = (
                GeodeticReference(origin_values[0], origin_values[1], float(self.get_parameter("gnss_origin_altitude").value))
                if all(_finite(value) for value in origin_values)
                else None
            )
            map_enu_yaw = float(self.get_parameter("map_enu_yaw").value)
            alignment = None
            if _finite(map_enu_yaw):
                alignment = Alignment2D(
                    map_enu_yaw,
                    float(self.get_parameter("map_enu_offset_x").value),
                    float(self.get_parameter("map_enu_offset_y").value),
                )
            self._timeout = float(self.get_parameter("freshness_timeout").value)
            self._core = MultisourceFusionCore(
                FusionConfig(
                    gnss_reference=origin,
                    alignment=alignment,
                    freshness_timeout=self._timeout,
                    max_correction_step=float(self.get_parameter("max_correction_step").value),
                    gnss_residual_gate=float(self.get_parameter("gnss_residual_gate").value),
                    lidar_residual_gate=float(self.get_parameter("lidar_residual_gate").value),
                )
            )
            self._max_amcl_covariance = float(self.get_parameter("max_amcl_covariance").value)
            self._local_odom: Pose2D | None = None
            self._odom_sequence = 0
            self._odom_consumed_sequence = 0
            self._gnss_fix: Any | None = None
            self._gnss_fix_received_at: float | None = None
            self._gnss_sequence = 0
            self._gnss_consumed_sequence = 0
            self._gnss_state = "REJECTED"
            self._gnss_accepted = False
            self._gnss_quality_received_at: float | None = None
            self._amcl_pose: Pose2D | None = None
            self._amcl_received_at: float | None = None
            self._amcl_sequence = 0
            self._amcl_consumed_sequence = 0
            self._amcl_state = "REJECTED"
            self._scan_match_pose: Pose2D | None = None
            self._scan_match_received_at: float | None = None
            self._scan_match_sequence = 0
            self._scan_match_consumed_sequence = 0
            self._scan_match_state = "REJECTED"
            self._scan_match_accepted = False

            self._odom_pub = self.create_publisher(
                Odometry, str(self.get_parameter("fusion_odom_topic").value), 10
            )
            self._pose_pub = self.create_publisher(
                PoseWithCovarianceStamped, str(self.get_parameter("fusion_pose_topic").value), 10
            )
            self._status_pub = self.create_publisher(
                DiagnosticArray, str(self.get_parameter("fusion_status_topic").value), 10
            )
            self.create_subscription(
                Odometry, str(self.get_parameter("local_odom_topic").value), self._on_odom, 10
            )
            self.create_subscription(
                NavSatFix, str(self.get_parameter("gnss_fix_topic").value), self._on_gnss_fix, 10
            )
            self.create_subscription(
                DiagnosticArray, str(self.get_parameter("gnss_quality_topic").value), self._on_gnss_quality, 10
            )
            self.create_subscription(
                PoseWithCovarianceStamped, str(self.get_parameter("amcl_pose_topic").value), self._on_amcl, 10
            )
            self.create_subscription(
                PoseStamped, str(self.get_parameter("scan_match_pose_topic").value), self._on_scan_match_pose, 10
            )
            self.create_subscription(
                DiagnosticArray, str(self.get_parameter("match_quality_topic").value), self._on_match_quality, 10
            )
            self._timer = self.create_timer(float(self.get_parameter("timer_period").value), self._tick)
            if bool(self.get_parameter("publish_tf").value):
                self.get_logger().warn("publish_tf is intentionally ignored; fusion POC never owns map->odom TF")
            self.get_logger().info("DG multi-source fusion POC active; side-channel outputs only")

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

        def _status(self, message: Any, hint: str) -> Any | None:
            selected = None
            for status in message.status:
                if selected is None:
                    selected = status
                if hint in str(status.name).upper():
                    return status
            return selected

        def _on_odom(self, message: Any) -> None:
            now = self._now()
            yaw = _yaw_from_quaternion(message.pose.pose.orientation)
            if yaw is None:
                self._local_odom = None
                return
            stamp = _stamp_to_seconds(message.header.stamp) or now
            self._local_odom = Pose2D(float(message.pose.pose.position.x), float(message.pose.pose.position.y), yaw, stamp)
            self._odom_sequence += 1

        def _on_gnss_fix(self, message: Any) -> None:
            self._gnss_fix = message
            self._gnss_fix_received_at = self._now()
            self._gnss_accepted = True
            self._gnss_sequence += 1

        def _on_gnss_quality(self, message: Any) -> None:
            selected = self._status(message, "GNSS")
            if selected is None:
                return
            values = {str(item.key): str(item.value) for item in selected.values}
            self._gnss_state = _value(values, "current_state", _value(values, "decision", "REJECTED")).upper()
            self._gnss_accepted = _bool_value(_value(values, "accepted", "false"))
            self._gnss_quality_received_at = self._now()
            self._gnss_sequence += 1

        def _on_amcl(self, message: Any) -> None:
            now = self._now()
            yaw = _yaw_from_quaternion(message.pose.pose.orientation)
            if yaw is None:
                self._amcl_pose = None
                self._amcl_state = "REJECTED"
                return
            covariance = float(message.pose.covariance[0])
            stamp = _stamp_to_seconds(message.header.stamp) or now
            self._amcl_pose = Pose2D(float(message.pose.pose.position.x), float(message.pose.pose.position.y), yaw, stamp)
            self._amcl_state = "GOOD" if math.isfinite(covariance) and covariance <= self._max_amcl_covariance else "REJECTED"
            self._amcl_received_at = now
            self._amcl_sequence += 1

        def _on_scan_match_pose(self, message: Any) -> None:
            now = self._now()
            yaw = _yaw_from_quaternion(message.pose.orientation)
            if yaw is None:
                self._scan_match_pose = None
                return
            stamp = _stamp_to_seconds(message.header.stamp) or now
            self._scan_match_pose = Pose2D(float(message.pose.position.x), float(message.pose.position.y), yaw, stamp)
            self._scan_match_received_at = now
            self._scan_match_sequence += 1

        def _on_match_quality(self, message: Any) -> None:
            selected = self._status(message, "MATCH")
            if selected is None:
                return
            values = {str(item.key): str(item.value) for item in selected.values}
            self._scan_match_accepted = _bool_value(_value(values, "accepted", "false"))
            self._scan_match_state = "GOOD" if self._scan_match_accepted and int(selected.level) < 2 else "REJECTED"
            self._scan_match_received_at = self._now()
            self._scan_match_sequence += 1

        def _gnss_measurement(self, now: float) -> GnssPosition | None:
            if self._gnss_fix is None:
                return None
            stamp = _stamp_to_seconds(self._gnss_fix.header.stamp) or self._gnss_fix_received_at or now
            quality_fresh = self._gnss_quality_received_at is not None and now - self._gnss_quality_received_at <= self._timeout
            fix_fresh = now - stamp <= self._timeout
            return GnssPosition(
                latitude=float(self._gnss_fix.latitude),
                longitude=float(self._gnss_fix.longitude),
                altitude=float(self._gnss_fix.altitude),
                timestamp=stamp,
                state=self._gnss_state,
                fresh=bool(quality_fresh and fix_fresh),
                accepted=bool(self._gnss_accepted),
            )

        def _map_measurement(self, pose: Pose2D | None, state: str, accepted: bool, source: str, now: float) -> MapPoseMeasurement | None:
            if pose is None:
                return None
            received_at = self._scan_match_received_at if source == "scan_match" else self._amcl_received_at
            fresh = received_at is not None and now - received_at <= self._timeout
            return MapPoseMeasurement(pose, pose.timestamp, state, fresh, accepted, source)

        def _tick(self) -> None:
            now = self._now()
            new_odom = self._local_odom if self._odom_sequence != self._odom_consumed_sequence else None
            new_gnss = self._gnss_measurement(now) if self._gnss_sequence != self._gnss_consumed_sequence else None
            new_scan_pose = self._scan_match_pose if self._scan_match_sequence != self._scan_match_consumed_sequence else None
            new_amcl_pose = self._amcl_pose if self._amcl_sequence != self._amcl_consumed_sequence else None
            scan = self._map_measurement(new_scan_pose, self._scan_match_state, self._scan_match_accepted, "scan_match", now)
            amcl = self._map_measurement(new_amcl_pose, self._amcl_state, self._amcl_state == "GOOD", "amcl", now)
            output = self._core.process(
                FusionInput(
                    now=now,
                    local_odom=new_odom,
                    gnss=new_gnss,
                    scan_match_pose=scan,
                    amcl_pose=amcl,
                )
            )
            self._odom_consumed_sequence = self._odom_sequence
            self._gnss_consumed_sequence = self._gnss_sequence
            self._scan_match_consumed_sequence = self._scan_match_sequence
            self._amcl_consumed_sequence = self._amcl_sequence
            self._publish(output)

        def _publish(self, output: Any) -> None:
            stamp = self.get_clock().now().to_msg()
            if output.map_pose is not None:
                pose = output.map_pose
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = "map"
                odom.child_frame_id = "base_footprint"
                odom.pose.pose.position.x = pose.x
                odom.pose.pose.position.y = pose.y
                odom.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
                odom.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
                odom.pose.covariance[0] = output.position_uncertainty ** 2
                odom.pose.covariance[35] = output.yaw_uncertainty ** 2
                self._odom_pub.publish(odom)

                pose_message = PoseWithCovarianceStamped()
                pose_message.header = odom.header
                pose_message.pose = odom.pose
                self._pose_pub.publish(pose_message)

            array = DiagnosticArray()
            array.header.stamp = stamp
            status = DiagnosticStatus()
            status.name = "DG Multi-source Fusion"
            status.level = DiagnosticStatus.OK if output.fusion_mode in {"NOMINAL", "GNSS_AIDED", "LIDAR_AIDED"} else DiagnosticStatus.WARN
            status.message = output.fusion_mode
            fields = {
                "fusion_mode": output.fusion_mode,
                "accepted_source": output.accepted_source or "NONE",
                "updated": output.updated,
                "position_uncertainty": output.position_uncertainty,
                "yaw_uncertainty": output.yaw_uncertainty,
                "alignment_ready": self._core.alignment is not None,
                "global_anchored": output.global_anchored,
                "uncertainty_model": "POC_HEURISTIC_NOT_CALIBRATED_COVARIANCE",
                "map_to_odom_x": output.map_to_odom.x if output.map_to_odom else None,
                "map_to_odom_y": output.map_to_odom.y if output.map_to_odom else None,
                "map_to_odom_yaw": output.map_to_odom.yaw if output.map_to_odom else None,
                "reasons": ";".join(output.reasons),
                "timestamp": output.timestamp,
            }
            status.values = [KeyValue(key=str(key), value="" if value is None else str(value)) for key, value in fields.items()]
            array.status = [status]
            self._status_pub.publish(array)

    return MultisourceFusionNode()


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args, ros_args = parser.parse_known_args(argv)
    try:
        import rclpy
    except ImportError:
        print("ROS2/rclpy is required only for the wrapper node; the core is ROS-independent.", file=sys.stderr)
        return 2
    rclpy.init(args=ros_args)
    node = create_node(args.node_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
