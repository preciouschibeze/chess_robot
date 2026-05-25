#!/usr/bin/env python3
from __future__ import absolute_import

import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.robot.ik_validation import CONFIRM_TEXT
from chess_robot.robot.ik_validation import DEFAULT_CONFIG_PATH
from chess_robot.robot.ik_validation import run_single_pose_validation
from chess_robot.robot.reachability import LIMIT_SOURCE_INTERSECTION
from chess_robot.robot.reachability import LIMIT_SOURCE_SOFTWARE
from chess_robot.robot.reachability import LIMIT_SOURCE_URDF
from chess_robot.robot.urdf_model import DEFAULT_END_LINK


DEFAULT_URDF_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
DEFAULT_SCENE_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "scene_geometry.yaml")
DEFAULT_JOINT_CALIBRATION_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
DEFAULT_JOINT_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_JOINT_SAFETY_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")
DEFAULT_HOME_POSE_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "home_pose.yaml")
DEFAULT_TOOL_FRAMES_PATH = os.path.join(REPO_ROOT, "data", "calibration", "gripper", "tool_frames.yaml")


def build_parser():
    parser = argparse.ArgumentParser(description="Validate one physical IK pose for the SO101 chess robot.")
    parser.add_argument("--urdf", default=DEFAULT_URDF_PATH, help="URDF model path.")
    parser.add_argument("--scene", default=DEFAULT_SCENE_PATH, help="Scene geometry YAML path.")
    parser.add_argument("--joint-calibration", default=DEFAULT_JOINT_CALIBRATION_PATH, help="Joint calibration YAML path.")
    parser.add_argument("--joint-limits", default=DEFAULT_JOINT_LIMITS_PATH, help="Legacy joint limits YAML path.")
    parser.add_argument("--joint-safety-limits", default=DEFAULT_JOINT_SAFETY_LIMITS_PATH, help="Joint safety limits YAML path.")
    parser.add_argument("--home-pose", default=DEFAULT_HOME_POSE_PATH, help="Home pose YAML path.")
    parser.add_argument("--tool-frames", default=DEFAULT_TOOL_FRAMES_PATH, help="Tool frame YAML path.")
    parser.add_argument("--tcp-frame", default="gripper_frame", help="TCP frame name.")
    parser.add_argument("--end-link", default=DEFAULT_END_LINK, help="URDF end link before tool-frame offset.")
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="Robot config path for execute mode.")
    parser.add_argument("--output", required=True, help="JSON output path.")

    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--square", help="Board square target, for example e4.")
    target_group.add_argument("--capture", action="store_true", help="Use the capture zone target.")
    target_group.add_argument("--target-world", nargs=3, type=float, help="Explicit world target XYZ in metres.")
    target_group.add_argument("--target-home-pose", action="store_true", help="Use saved home/stow TCP as the IK target.")
    target_group.add_argument("--target-world-offset-from-home", nargs=3, type=float, help="World XYZ offset from saved home/stow TCP in metres.")
    parser.add_argument("--target-type", choices=("above", "surface"), default="above", help="Named target height type.")
    parser.add_argument("--above-board-offset-m", type=float, default=0.080, help="Above-square height above board top.")
    parser.add_argument("--pick-offset-m", type=float, default=0.030, help="Surface/pick proxy height above board top.")
    parser.add_argument("--capture-above-offset-m", type=float, default=0.080, help="Capture above height above capture base.")
    parser.add_argument("--board-clearance-m", type=float, default=0.060, help="Minimum TCP clearance above board top for XY-changing moves.")
    parser.add_argument("--transit-clearance-m", type=float, default=0.090, help="Preferred transit clearance above board top, reported for planning diagnostics.")
    parser.add_argument("--path-samples", type=int, default=25, help="Joint-interpolated FK samples for board-clearance validation.")
    parser.add_argument("--xy-motion-epsilon-m", type=float, default=0.005, help="XY delta below this is treated as mostly vertical motion.")
    parser.set_defaults(enforce_board_clearance=None)
    parser.add_argument("--enforce-board-clearance", dest="enforce_board_clearance", action="store_true", help="Enforce board-clearance path validation. Defaults true in execute mode.")
    parser.add_argument("--no-enforce-board-clearance", dest="enforce_board_clearance", action="store_false", help="Report but do not enforce board-clearance validation.")
    parser.add_argument("--prefer-vertical-approach", action="store_true", help="Prefer a vertical world-down tool approach during IK solve.")
    parser.add_argument("--approach-axis-name", choices=("plus_x", "minus_x", "plus_y", "minus_y", "plus_z", "minus_z"), help="Named local tool axis to treat as the approach direction.")
    parser.add_argument("--approach-axis-local", nargs=3, type=float, help="Explicit local tool approach axis XYZ.")
    parser.add_argument("--max-approach-tilt-deg", type=float, default=10.0, help="Maximum allowed approach-axis tilt from world down.")
    parser.add_argument("--max-edge-approach-tilt-deg", type=float, default=20.0, help="Maximum allowed approach-axis tilt for edge squares.")
    parser.add_argument("--approach-weight", type=float, default=0.05, help="Residual weight for vertical-approach preference in IK.")
    parser.add_argument("--enforce-approach-angle", action="store_true", help="Abort when final approach angle exceeds the configured tilt limit.")

    parser.add_argument(
        "--limit-source",
        choices=(LIMIT_SOURCE_URDF, LIMIT_SOURCE_SOFTWARE, LIMIT_SOURCE_INTERSECTION),
        default=LIMIT_SOURCE_INTERSECTION,
        help="IK joint limit source.",
    )
    parser.add_argument("--random-seeds", type=int, default=20, help="Number of random IK seeds.")
    parser.add_argument("--workspace-seed-samples", type=int, default=1000, help="Sampled workspace seeds for initial guess.")
    parser.add_argument("--seed", type=int, default=7, help="Random seed.")
    parser.add_argument("--tolerance-m", type=float, default=0.005, help="IK success tolerance in metres.")
    parser.add_argument("--damping", type=float, default=0.05, help="Damped least-squares damping.")
    parser.add_argument("--max-iters", type=int, default=200, help="Maximum IK iterations.")
    parser.add_argument("--step-scale", type=float, default=1.0, help="Scale factor applied to each IK update.")
    parser.add_argument("--lock-wrist-roll-home", action="store_true", help="Lock wrist_roll to the saved home pose during IK.")
    parser.add_argument("--lock-joint", action="append", default=None, help="Lock one arm joint with joint=tick or joint_rad=value.")

    parser.add_argument("--execute", action="store_true", help="Command hardware after all safety checks.")
    parser.add_argument("--confirm", default=None, help="Typed execute confirmation.")
    parser.add_argument("--max-joint-delta-ticks", type=int, default=350, help="Maximum allowed per-joint execute delta.")
    parser.add_argument("--max-total-l1-delta-ticks", type=int, default=1200, help="Maximum total absolute execute delta.")
    parser.add_argument("--allow-large-delta", action="store_true", help="Override execute delta limits after inspection.")
    parser.add_argument("--speed-scale", type=float, default=0.25, help="Slow execution speed scale in (0, 1].")
    parser.add_argument("--settle-time-s", type=float, default=1.0, help="Final settle time before readback.")
    parser.add_argument("--readback-tolerance-ticks", type=int, default=25, help="Final readback tolerance.")
    return parser


def print_report(log, output_path):
    print("Mode: %s" % log["mode"])
    print("Target: %s" % log["target_name"])
    if log.get("saved_home_tcp_world_xyz_m") is not None:
        print("Saved home TCP world XYZ (m): %s" % format_xyz(log["saved_home_tcp_world_xyz_m"]))
    if log.get("requested_offset_world_m") is not None:
        print("Requested home world offset (m): %s" % format_xyz(log["requested_offset_world_m"]))
    print("Target world XYZ (m): %s" % format_xyz(log["target_world_xyz_m"]))
    print("Target robot XYZ (m): %s" % format_xyz(log["target_robot_xyz_m"]))
    print("Final IK TCP world XYZ (m): %s" % format_xyz(log["final_tcp_world_xyz_m"]))
    print("Final IK TCP robot XYZ (m): %s" % format_xyz(log["final_tcp_robot_xyz_m"]))
    print("IK: success=%s status=%s error=%.3f mm iterations=%d" % (
        log["ik_success"],
        log["ik_status"],
        float(log["ik_error_m"]) * 1000.0,
        int(log["ik_iterations"]),
    ))
    print("Joint angles (deg):")
    for joint_name in sorted(log["joint_angles_deg"]):
        print("  %s: %.3f" % (joint_name, log["joint_angles_deg"][joint_name]))
    print("Target servo ticks:")
    for joint_name in sorted(log["target_ticks"]):
        print("  %s: %s" % (joint_name, log["target_ticks"][joint_name]))
    if log.get("locked_joints_ticks"):
        print("Locked joints:")
        for joint_name in sorted(log["locked_joints_ticks"]):
            print(
                "  %s: tick=%s source=%s"
                % (
                    joint_name,
                    log["locked_joints_ticks"][joint_name],
                    log.get("locked_joint_sources", {}).get(joint_name),
                )
            )
    print("Safety margins (ticks):")
    for joint_name in sorted(log["safety_limit_margins_ticks"]):
        print("  %s: %s" % (joint_name, log["safety_limit_margins_ticks"][joint_name]))
    if log.get("saved_home_tick_deltas"):
        print("Target deltas from saved home (ticks):")
        for joint_name in sorted(log["saved_home_tick_deltas"]):
            print("  %s: %+d" % (joint_name, log["saved_home_tick_deltas"][joint_name]))
    if log.get("tick_deltas"):
        print("Motion deltas from readback (ticks):")
        for joint_name in sorted(log["tick_deltas"]):
            print("  %s: %+d" % (joint_name, log["tick_deltas"][joint_name]))
    else:
        print("Motion deltas from readback: unavailable in dry-run")
    if log.get("path_validation"):
        path = log["path_validation"]
        print("Path validation: passed=%s xy_delta=%s min_z=%s low_zone_z=%.6f samples=%s" % (
            path.get("passed"),
            format_optional_float(path.get("xy_delta_m")),
            format_optional_float(path.get("min_z_m")),
            float(path.get("low_zone_z_m")),
            path.get("samples_count"),
        ))
        if path.get("failure_reason"):
            print("Path validation reason: %s" % path.get("failure_reason"))
        print("Path validation note: approximate joint-space FK safety gate, not a full collision checker.")
    if log.get("approach_axis_world") is not None:
        print("Approach axis local: %s" % format_xyz(log["approach_axis_local"]))
        print("Approach axis name: %s source=%s" % (log.get("approach_axis_name"), log.get("approach_axis_source")))
        print("Approach axis world: %s" % format_xyz(log["approach_axis_world"]))
        print("Approach tilt: %.3f deg (limit %.3f deg, passed=%s)" % (
            float(log.get("approach_tilt_deg")),
            float(log.get("max_approach_tilt_deg")),
            log.get("approach_angle_check", {}).get("passed"),
        ))
        print("Best candidate axis: %s tilt=%.3f deg" % (log.get("best_candidate_axis_name"), float(log.get("best_candidate_axis_tilt_deg"))))
        if log.get("approach_axis_local_warning"):
            print("Approach axis warning: %s" % log.get("approach_axis_local_warning"))
    if log.get("abort_reason"):
        print("Abort reason: %s" % log["abort_reason"])
    print("Command sent: %s" % log["command_sent"])
    print("Saved JSON log: %s" % output_path)
    print("Execute confirmation phrase: %s" % CONFIRM_TEXT)


def format_xyz(values):
    return "[%.6f, %.6f, %.6f]" % (float(values[0]), float(values[1]), float(values[2]))


def format_optional_float(value):
    if value is None:
        return "unavailable"
    return "%.6f" % float(value)


def main():
    args = build_parser().parse_args()
    log = run_single_pose_validation(args)
    print_report(log, args.output)
    if args.execute and log.get("abort_reason"):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
