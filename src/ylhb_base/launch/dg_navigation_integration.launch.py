"""Standalone DG-202611 navigation integration POC.

Start the existing robot bringup separately so that /scan, /odom, TF and
optionally Nav2 are available.  This launch only adds the competition-side
observation, recovery and command-arbitration layer.  It is intentionally not
included by bringup.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    package_share = get_package_share_directory("ylhb_base")
    workspace_dir = os.environ.get("WS_DIR", os.path.expanduser("~/ros2_DL"))
    preferred_map = os.path.join(workspace_dir, "maps", "my_map.yaml")
    fallback_map = os.path.join(workspace_dir, "src", "my_map.yaml")
    default_map = preferred_map if os.path.exists(preferred_map) else fallback_map
    enable_nav2 = LaunchConfiguration("enable_nav2")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    required_signals = LaunchConfiguration("required_signals")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, "launch", "navigation.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_file,
            "params_file": params_file,
            "autostart": "true",
        }.items(),
    )

    lidar = Node(
        package="ylhb_base",
        executable="lidar_robust_node",
        name="dg_lidar_robust_node",
        output="screen",
        parameters=[{"scan_topic": "/scan", "odom_topic": "/odom", "quality_topic": "/dg/lidar/quality"}],
    )
    gnss = Node(
        package="ylhb_base",
        executable="gnss_quality_node",
        name="dg_gnss_quality_node",
        output="screen",
        parameters=[
            {
                "fix_topic": "/gps/fix",
                "status_topic": "/gps/rtk_status",
                "quality_topic": "/dg/gnss/quality",
            }
        ],
    )
    scan_map = Node(
        package="ylhb_base",
        executable="scan_map_relocalization_node",
        name="dg_scan_map_relocalization_node",
        output="screen",
        parameters=[
            {
                "map_topic": "/map",
                "scan_topic": "/scan",
                "initialpose_topic": "/initialpose",
                "seed_topic": "/dg/relocalization/seed",
                "scan_match_pose_topic": "/scan_match_pose",
                "match_quality_topic": "/dg/relocalization/match_quality",
            }
        ],
    )
    relocalization = Node(
        package="ylhb_base",
        executable="active_relocalization_node",
        name="dg_active_relocalization_node",
        output="screen",
        parameters=[
            {
                "scan_topic": "/scan",
                "odom_topic": "/odom",
                "amcl_pose_topic": "/amcl_pose",
                "lidar_quality_topic": "/dg/lidar/quality",
                "gnss_quality_topic": "/dg/gnss/quality",
                "match_quality_topic": "/dg/relocalization/match_quality",
                "cmd_vel_topic": "/cmd_vel_recovery",
                "required_signals": required_signals,
            }
        ],
    )
    health = Node(
        package="ylhb_base",
        executable="navigation_health_node",
        name="dg_navigation_health_node",
        output="screen",
        parameters=[
            {
                "required_signals": required_signals,
                "status_topic": "/dg/navigation/status",
            }
        ],
    )
    arbiter = Node(
        package="ylhb_base",
        executable="cmd_vel_arbiter_node",
        name="dg_cmd_vel_arbiter_node",
        output="screen",
        parameters=[
            {
                "navigation_topic": "/cmd_vel_nav",
                "recovery_topic": "/cmd_vel_recovery",
                "output_topic": "/cmd_vel",
                "navigation_status_topic": "/dg/navigation/status",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_nav2", default_value="false", description="Start existing Nav2 launch"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "required_signals",
                default_value="[]",
                description="Optional required health signals, for example ['lidar']",
            ),
            GroupAction([SetRemap(src="/cmd_vel", dst="/cmd_vel_nav"), nav2_launch], condition=IfCondition(enable_nav2)),
            lidar,
            gnss,
            scan_map,
            relocalization,
            health,
            arbiter,
        ]
    )
