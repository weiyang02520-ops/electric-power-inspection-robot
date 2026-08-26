#!/usr/bin/env python3
"""Optional ROS2 side-channel wrapper for the 2D LiDAR robustness POC."""

from __future__ import annotations

import argparse
import copy
import math
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from lidar_robust_features import RobustFeatureConfig, ScanFrame, process_scan  # noqa: E402


def _yaw_from_quaternion(quaternion: Any) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _relative_pose(previous: tuple[float, float, float], current: tuple[float, float, float]) -> tuple[float, float, float]:
    px, py, pyaw = previous
    cx, cy, cyaw = current
    dx_world = cx - px
    dy_world = cy - py
    cosine = math.cos(pyaw)
    sine = math.sin(pyaw)
    dx_previous = cosine * dx_world + sine * dy_world
    dy_previous = -sine * dx_world + cosine * dy_world
    dyaw = math.atan2(math.sin(cyaw - pyaw), math.cos(cyaw - pyaw))
    return dx_previous, dy_previous, dyaw


def _odom_pose(message: Any) -> tuple[float, float, float]:
    pose = message.pose.pose
    return pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the side-channel DG-202611 2D LiDAR robustness node.")
    parser.add_argument("--node-name", default="lidar_robust_node", help="ROS node name")
    return parser


def _config_from_node(node: Any) -> RobustFeatureConfig:
    return RobustFeatureConfig(
        min_range=float(node.get_parameter("min_range").value),
        max_range=float(node.get_parameter("max_range").value),
        self_radius=float(node.get_parameter("self_radius").value),
        isolation_range_jump=float(node.get_parameter("isolation_range_jump").value),
        temporal_match_distance=float(node.get_parameter("temporal_match_distance").value),
        corner_deviation=float(node.get_parameter("corner_deviation").value),
        corner_neighbor_distance=float(node.get_parameter("corner_neighbor_distance").value),
        line_neighbor_distance=float(node.get_parameter("line_neighbor_distance").value),
        min_line_points=int(node.get_parameter("min_line_points").value),
    )


def create_node(node_name: str) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import PoseArray
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan

    class LidarRobustNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self.declare_parameter("scan_topic", "/scan")
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("filtered_scan_topic", "/dg/lidar/filtered_scan")
            self.declare_parameter("stable_features_topic", "/dg/lidar/stable_features")
            self.declare_parameter("quality_topic", "/dg/lidar/quality")
            self.declare_parameter("min_range", 0.05)
            self.declare_parameter("max_range", 8.0)
            self.declare_parameter("self_radius", 0.18)
            self.declare_parameter("isolation_range_jump", 0.35)
            self.declare_parameter("temporal_match_distance", 0.12)
            self.declare_parameter("corner_deviation", math.radians(25.0))
            self.declare_parameter("corner_neighbor_distance", 0.35)
            self.declare_parameter("line_neighbor_distance", 0.25)
            self.declare_parameter("min_line_points", 3)

            config = _config_from_node(self)
            self._config = config
            self._previous_frame: ScanFrame | None = None
            self._previous_odom_pose: tuple[float, float, float] | None = None
            self._latest_odom_pose: tuple[float, float, float] | None = None
            self._warned_without_odom = False
            scan_topic = str(self.get_parameter("scan_topic").value)
            odom_topic = str(self.get_parameter("odom_topic").value)
            self._filtered_pub = self.create_publisher(LaserScan, str(self.get_parameter("filtered_scan_topic").value), 10)
            self._features_pub = self.create_publisher(PoseArray, str(self.get_parameter("stable_features_topic").value), 10)
            self._quality_pub = self.create_publisher(DiagnosticArray, str(self.get_parameter("quality_topic").value), 10)
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
            self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
            self.get_logger().info(
                f"side-channel LiDAR robustness POC active: {scan_topic} + {odom_topic}; "
                "existing scan consumers are not replaced"
            )

        def _on_odom(self, message: Any) -> None:
            self._latest_odom_pose = _odom_pose(message)

        def _on_scan(self, message: Any) -> None:
            frame = ScanFrame(
                ranges=list(message.ranges),
                angle_min=float(message.angle_min),
                angle_increment=float(message.angle_increment),
                range_min=float(message.range_min),
                range_max=float(message.range_max),
            )
            relative_pose = (0.0, 0.0, 0.0)
            if self._latest_odom_pose is not None and self._previous_odom_pose is not None:
                relative_pose = _relative_pose(self._previous_odom_pose, self._latest_odom_pose)
            elif self._previous_frame is not None and not self._warned_without_odom:
                self.get_logger().warning("no two odometry poses yet; temporal compensation is not available")
                self._warned_without_odom = True
            try:
                result = process_scan(frame, self._previous_frame, relative_pose, self._config)
            except ValueError as exc:
                self.get_logger().error(f"invalid LaserScan for robustness POC: {exc}")
                return
            self._publish_filtered_scan(message, result)
            self._publish_features(message, result)
            self._publish_quality(message, result.quality)
            self._previous_frame = frame
            self._previous_odom_pose = self._latest_odom_pose

        def _publish_filtered_scan(self, source: Any, result: Any) -> None:
            filtered = copy.deepcopy(source)
            remove_indices = set(result.rejected_indices)
            remove_indices.update(item.point.index for item in result.classifications if item.dynamic_candidate)
            filtered.ranges = [float("nan") if index in remove_indices else value for index, value in enumerate(source.ranges)]
            self._filtered_pub.publish(filtered)

        def _publish_features(self, source: Any, result: Any) -> None:
            message = PoseArray()
            message.header = source.header
            for feature in result.features:
                pose = message.poses.add() if hasattr(message.poses, "add") else None
                if pose is None:
                    from geometry_msgs.msg import Pose

                    pose = Pose()
                    message.poses.append(pose)
                pose.position.x = float(feature["x"])
                pose.position.y = float(feature["y"])
                pose.position.z = 0.0
                pose.orientation.w = 1.0
            self._features_pub.publish(message)

        def _publish_quality(self, source: Any, quality: dict[str, Any]) -> None:
            message = DiagnosticArray()
            message.header = source.header
            status = DiagnosticStatus()
            status.name = "dg/lidar/quality"
            status.level = DiagnosticStatus.OK if quality["valid_point_count"] else DiagnosticStatus.ERROR
            status.message = str(quality["temporal_status"])
            status.values = [KeyValue(key=str(key), value="" if value is None else str(value)) for key, value in sorted(quality.items())]
            message.status = [status]
            self._quality_pub.publish(message)

    return LidarRobustNode()


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args, ros_args = parser.parse_known_args(argv)
    try:
        import rclpy
    except ImportError:
        print("ROS2/rclpy is required only for the wrapper node; the pure core is ROS-independent.", file=sys.stderr)
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
