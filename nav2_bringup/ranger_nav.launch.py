#!/usr/bin/env python3
# =============================================================================
# ranger_nav.launch.py — Nav2 bringup for the AgileX Ranger Mini 3 (M1.2)
# ROS2 Jazzy / Nav2 1.3.x
# =============================================================================
# Brings up (all lifecycle-managed, autostarted):
#   map_server, planner_server, controller_server, behavior_server, bt_navigator
# plus a lifecycle_manager that configures+activates them in order.
#
# Consumes : /scan, /odom, /tf, /tf_static          (published by the Isaac worker)
# Produces : /cmd_vel (geometry_msgs/Twist), /plan, costmaps
# Action   : /navigate_to_pose  (NavigateToPose)
#
# We deliberately DO NOT use nav2_common.RewrittenYaml so this launch parses and
# runs even where only nav2 binaries (not nav2_common's python helper) differ;
# per-node overrides (use_sim_time, map path) are passed as extra param dicts.
#
# Localization note: per docs/integration_contract.md the map->odom TF is
# published by Isaac (scene is ground-truth localized), so NO AMCL runs here.
#
# Invocation (this dir is not an installed ament package, so launch by path):
#   ros2 launch <repo>/nav2_bringup/ranger_nav.launch.py \
#       map:=<repo>/runs/maps/<scene>.yaml
#   # or:  python3 <repo>/nav2_bringup/ranger_nav.launch.py
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory  # noqa: F401  (kept for parity)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    default_params = os.path.join(pkg_dir, "nav2_params.yaml")
    default_map = os.path.join(pkg_dir, "maps", "placeholder_map.yaml")

    # ---- launch args -------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    log_level = LaunchConfiguration("log_level")
    localization = LaunchConfiguration("localization")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use the /clock published by Isaac (sim time).")
    declare_autostart = DeclareLaunchArgument(
        "autostart", default_value="true",
        description="Auto-configure+activate the Nav2 lifecycle nodes on launch.")
    declare_params_file = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="Full path to the Nav2 params YAML.")
    declare_map = DeclareLaunchArgument(
        "map", default_value=default_map,
        description="Full path to the map YAML. Default is a placeholder; point "
                    "this at runs/maps/<scene>.yaml exported by the scene assembler.")
    declare_localization = DeclareLaunchArgument(
        "localization", default_value="amcl",
        description="'amcl'  -> map_server serves the saved map and AMCL provides map->odom. "
                    "'slam'  -> neither is started; slam_toolbox supplies BOTH /map and map->odom "
                    "(mapping or localization mode). "
                    "EXACTLY ONE source may publish /map and map->odom; running Nav2's map_server "
                    "alongside slam_toolbox puts two publishers on /map, and the costmap then uses "
                    "whichever it happened to latch.")

    declare_log_level = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="Logging level for all Nav2 nodes.")

    # Per-node param override applied on top of the shared params_file.
    common_overrides = {"use_sim_time": use_sim_time}

    # HARDWARE FIX: resolve the behavior-tree XMLs from THIS FILE's location.
    # nav2_params.yaml ships them as absolute paths into a specific developer's home directory
    # (/home/minghanwei/...), so bt_navigator fails to activate on every other machine with
    # "Couldn't open input XML file" -- and because the lifecycle manager then aborts the whole
    # bringup, Nav2 does not come up at all. Overriding here rather than editing the copied YAML
    # keeps the diff against the sim repo readable, and computing from __file__ means this works
    # from any checkout path.
    _bt_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "behavior_trees")
    bt_overrides = {
        "default_nav_to_pose_bt_xml": os.path.join(_bt_dir, "navigate_to_pose_no_spin.xml"),
        "default_nav_through_poses_bt_xml": os.path.join(_bt_dir, "navigate_through_poses_no_spin.xml"),
    }

    # The set of lifecycle nodes the manager will bring up, in order.
    lifecycle_nodes = [
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    arguments = ["--ros-args", "--log-level", log_level]

    # map_server and amcl run ONLY in localization:=amcl. In slam mode slam_toolbox owns both
    # /map and map->odom, and starting these would duplicate the first and contend for the second.
    _amcl_mode = IfCondition(PythonExpression(["'", localization, "' == 'amcl'"]))

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        condition=_amcl_mode,
        parameters=[params_file, common_overrides, {"yaml_filename": map_yaml}],
        arguments=arguments,
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        condition=_amcl_mode,
        parameters=[params_file, common_overrides],
        arguments=arguments,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, common_overrides],
        arguments=arguments,
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, common_overrides],
        # HARDWARE CHANGE: Nav2's controller output is REMAPPED to /cmd_vel_nav.
        # config/safety.yaml makes safety/twist_mux_node.py the ONLY publisher of /cmd_vel; Nav2 is
        # one arbitrated source among three (nav, priority 10). Letting the controller publish
        # /cmd_vel directly would put a second publisher on that topic and silently bypass the
        # E-stop, the arm interlock, the speed ceilings and the slew limiter -- every protection
        # this repo has for base motion. In sim there was no mux, so publishing /cmd_vel was right
        # there and is wrong here.
        # enable_stamped_cmd_vel:=false stays: the mux consumes geometry_msgs/Twist, and Jazzy
        # would otherwise default to TwistStamped and the mux would receive nothing at all.
        remappings=[("/cmd_vel", "/cmd_vel_nav")],
        arguments=arguments,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, common_overrides],
        # The behavior server DRIVES THE BASE too -- BackUp and DriveOnHeading are recovery
        # behaviours that emit their own /cmd_vel, independently of controller_server. Remapping
        # only the controller leaves three publishers on /cmd_vel that bypass the mux entirely:
        # no E-stop, no arm interlock, no speed ceiling, no slew limit. arbiter.py names this exact
        # case ("a Nav2 recovery behaviour ... produces base motion without going through act()")
        # as a reason the interlock exists. Verified with `ros2 topic info /cmd_vel -v`
        # on 2026-08-20: publisher count 3, all behavior_server.
        remappings=[("/cmd_vel", "/cmd_vel_nav")],
        arguments=arguments,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, common_overrides, bt_overrides],
        arguments=arguments,
    )

    # The Isaac-only `RPLidar_S2E -> lidar_link` alias TF is REMOVED on hardware.
    # It existed because Isaac named the lidar TF after its USD prim while stamping /scan with
    # frame_id "lidar_link". The real rplidar_node stamps lidar_link natively and
    # bringup/lidar3d.sh publishes base_link -> lidar_link, so re-publishing that edge here would
    # put two publishers on one transform -- a bug in its own right.

    # The manager must be told EXACTLY the nodes that exist: it waits on a bond from each, and a
    # name that was never started stalls the whole bringup rather than being skipped.
    node_names = PythonExpression([
        "['map_server','amcl'] + ", str(lifecycle_nodes[1:]),
        " if '", localization, "' == 'amcl' else ", str(lifecycle_nodes[1:]),
    ])

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": node_names,
            "bond_timeout": 4.0,
            "attempt_respawn_reconnection": True,
        }],
        arguments=arguments,
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_map,
        declare_log_level,
        declare_localization,
        map_server,
        amcl,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])


if __name__ == "__main__":
    # Allow `python3 ranger_nav.launch.py` for a quick standalone bringup.
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    raise SystemExit(ls.run())
