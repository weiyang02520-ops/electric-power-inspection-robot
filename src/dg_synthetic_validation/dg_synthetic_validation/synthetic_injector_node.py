"""ROS2 node that publishes only deterministic sensor/input messages.

No ``/dg/*`` output is published here.  The nodes under test are the only
writers of health, fusion, relocalization, and command-arbitration topics.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

from .scenario_schema import LidarPhase, Scenario, load_scenario


def _stamp(node: Any) -> Any:
    return node.get_clock().now().to_msg()


def _header(node: Any, frame_id: str) -> Any:
    from std_msgs.msg import Header

    header = Header()
    header.stamp = _stamp(node)
    header.frame_id = frame_id
    return header


def _map_message(node: Any) -> Any:
    from nav_msgs.msg import OccupancyGrid

    message = OccupancyGrid()
    message.header = _header(node, "map")
    message.info.resolution = 0.1
    message.info.width = 80
    message.info.height = 80
    message.info.origin.position.x = -4.0
    message.info.origin.position.y = -4.0
    message.info.origin.orientation.w = 1.0
    data = [0] * (message.info.width * message.info.height)
    # Four deterministic boundary walls plus two interior strips give the
    # scan-to-map node a valid map message without claiming a realistic map.
    for y in range(message.info.height):
        for x in range(message.info.width):
            if x in {0, 79} or y in {0, 79} or (30 <= x <= 31 and 15 <= y <= 65):
                data[y * message.info.width + x] = 100
    message.data = data
    return message


def _make_scan(node: Any, phase: LidarPhase) -> Any:
    from sensor_msgs.msg import LaserScan

    beam_count = 360
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / beam_count
    message = LaserScan()
    message.header = _header(node, "laser")
    message.angle_min = angle_min
    message.angle_max = math.pi - angle_increment
    message.angle_increment = angle_increment
    message.time_increment = 0.0
    message.scan_time = 0.1
    message.range_min = 0.05
    message.range_max = 8.0
    ranges: list[float] = []
    mode = phase.mode.lower()
    for index in range(beam_count):
        angle = angle_min + index * angle_increment
        in_window = phase.angle_min <= angle <= phase.angle_max
        keep = in_window and (phase.valid_fraction >= 1.0 or index % max(1, int(round(1.0 / max(phase.valid_fraction, 1e-6)))) == 0)
        if mode in {"empty", "none", "all_invalid"}:
            keep = False
        if keep:
            # A fixed, gently varying range provides repeatable geometry and
            # enough temporal matches while avoiding random noise.
            ranges.append(float(phase.range_m + 0.12 * math.sin(3.0 * angle) + 0.04 * math.cos(7.0 * angle)))
        else:
            ranges.append(float("inf"))
    message.ranges = ranges
    message.intensities = [0.0] * beam_count
    return message


def _make_fix(node: Any, available: bool) -> Any:
    from sensor_msgs.msg import NavSatFix, NavSatStatus

    message = NavSatFix()
    message.header = _header(node, "gps")
    message.status.status = NavSatStatus.STATUS_FIX if available else NavSatStatus.STATUS_NO_FIX
    message.status.service = NavSatStatus.SERVICE_GPS
    message.latitude = 31.2304
    message.longitude = 121.4737
    message.altitude = 10.0
    message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    message.position_covariance[0] = 0.25
    message.position_covariance[4] = 0.25
    message.position_covariance[8] = 0.5
    return message


def _make_status(node: Any, phase: Any) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    array = DiagnosticArray()
    array.header = _header(node, "gps")
    status = DiagnosticStatus()
    status.name = "Synthetic WTRTK980 RTK Status"
    # diagnostic_msgs/msg/DiagnosticStatus.level is a uint8 represented as a
    # one-byte ``bytes`` value by the ROS2 Humble Python generator.
    status.level = bytes([max(0, min(255, int(phase.diagnostic_level)))])
    status.message = "synthetic sensor input"
    status.values = [
        KeyValue(key="quality", value=str(phase.quality)),
        KeyValue(key="quality_text", value="RTK_FIXED" if phase.quality in (4, 5) else "RTK_FLOAT"),
        KeyValue(key="satellites", value=str(phase.satellites)),
        KeyValue(key="hdop", value=str(phase.hdop)),
        KeyValue(key="differential_age", value=str(phase.differential_age)),
        KeyValue(key="base_station_id", value="SYNTHETIC"),
    ]
    array.status = [status]
    return array


def _make_odom(node: Any) -> Any:
    from nav_msgs.msg import Odometry

    message = Odometry()
    message.header = _header(node, "odom")
    message.child_frame_id = "base_footprint"
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.02
    message.pose.covariance[7] = 0.02
    message.pose.covariance[35] = 0.01
    message.twist.twist.linear.x = 0.05
    message.twist.covariance[0] = 0.02
    return message


def _make_amcl(node: Any) -> Any:
    from geometry_msgs.msg import PoseWithCovarianceStamped

    message = PoseWithCovarianceStamped()
    message.header = _header(node, "map")
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.05
    message.pose.covariance[7] = 0.05
    message.pose.covariance[35] = 0.03
    return message


def _make_initialpose(node: Any) -> Any:
    return _make_amcl(node)


def create_node(scenario: Scenario, node_name: str = "dg_synthetic_injector") -> Any:
    """Create an injector node; imports ROS types lazily for unit-testability."""
    import rclpy
    from rclpy.node import Node

    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan, NavSatFix

    class SyntheticInjectorNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self._scenario = scenario
            self._start = time.monotonic()
            self._last_map = 0.0
            self._fix_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
            self._status_pub = self.create_publisher(DiagnosticArray, "/gps/rtk_status", 10)
            self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
            self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
            self._amcl_pub = self.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
            self._map_pub = self.create_publisher(OccupancyGrid, "/map", 1)
            self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
            period = 1.0 / max(1.0, scenario.sensor_hz)
            self._timer = self.create_timer(period, self._tick)
            self.get_logger().info(
                f"synthetic injector active: {scenario.scenario_id}, duration={scenario.duration_sec:.1f}s; "
                "sensor topics only"
            )

        def _tick(self) -> None:
            elapsed = time.monotonic() - self._start
            if elapsed > self._scenario.duration_sec:
                return
            phase = self._scenario.phase_at(elapsed)
            if phase.gnss.publish_status:
                self._status_pub.publish(_make_status(self, phase.gnss))
            if phase.gnss.publish_fix:
                self._fix_pub.publish(_make_fix(self, phase.gnss.fix_available))
            self._scan_pub.publish(_make_scan(self, phase.lidar))
            self._odom_pub.publish(_make_odom(self))
            self._amcl_pub.publish(_make_amcl(self))
            if elapsed - self._last_map >= 1.0 / max(0.2, self._scenario.map_hz):
                self._map_pub.publish(_map_message(self))
                self._initialpose_pub.publish(_make_initialpose(self))
                self._last_map = elapsed

    return SyntheticInjectorNode()


def main(args: list[str] | None = None) -> None:
    import argparse
    import rclpy

    parser = argparse.ArgumentParser(description="Publish deterministic S01-S04 sensor inputs")
    parser.add_argument("scenario_file", type=Path)
    parser.add_argument("--node-name", default="dg_synthetic_injector")
    parsed = parser.parse_args(args)
    scenario = load_scenario(parsed.scenario_file)
    rclpy.init(args=None)
    node = create_node(scenario, parsed.node_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
