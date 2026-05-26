#!/usr/bin/env python3
from __future__ import absolute_import, print_function

import argparse
import json
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.robot.safe_recovery import CONFIRM_TEXT_RECOVER_HOME
from chess_robot.robot.safe_recovery import DEFAULT_RECOVERY_CLEARANCE_M
from chess_robot.robot.safe_recovery import run_safe_recovery
from chess_robot.robot.reachability import LIMIT_SOURCE_INTERSECTION
from chess_robot.robot.reachability import LIMIT_SOURCE_SOFTWARE
from chess_robot.robot.reachability import LIMIT_SOURCE_URDF
from chess_robot.robot.urdf_model import DEFAULT_END_LINK
from chess_robot.robot.ik_validation import DEFAULT_CONFIG_PATH


DEFAULT_URDF_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
DEFAULT_SCENE_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "scene_geometry.yaml")
DEFAULT_JOINT_CALIBRATION_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
DEFAULT_JOINT_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_JOINT_SAFETY_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")
DEFAULT_HOME_POSE_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "home_pose.yaml")
DEFAULT_TOOL_FRAMES_PATH = os.path.join(REPO_ROOT, "data", "calibration", "gripper", "tool_frames.yaml")
DEFAULT_APPROACH_POLICY_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "approach_policy.yaml")
DEFAULT_OUTPUT_PATH = os.path.join(REPO_ROOT, "data", "debug", "recover_home_dryrun.json")


def parse_square_list(value):
    if value is None:
        return None
    squares = []
    for item in str(value).split(","):
        normalized = item.strip().lower()
        if normalized:
            squares.append(normalized)
    return squares


def build_parser():
    parser = argparse.ArgumentParser(description="Validate and optionally execute a safe staged recovery-to-home path.")
    parser.add_argument("--urdf", default=DEFAULT_URDF_PATH, help="URDF model path.")
    parser.add_argument("--scene", default=DEFAULT_SCENE_PATH, help="Scene geometry YAML path.")
    parser.add_argument("--joint-calibration", default=DEFAULT_JOINT_CALIBRATION_PATH, help="Joint calibration YAML path.")
    parser.add_argument("--joint-limits", default=DEFAULT_JOINT_LIMITS_PATH, help="Legacy joint limits YAML path.")
    parser.add_argument("--joint-safety-limits", default=DEFAULT_JOINT_SAFETY_LIMITS_PATH, help="Joint safety limits YAML path.")
    parser.add_argument("--home-pose", default=DEFAULT_HOME_POSE_PATH, help="Home pose YAML path.")
    parser.add_argument("--tool-frames", default=DEFAULT_TOOL_FRAMES_PATH, help="Tool frame YAML path.")
    parser.add_argument("--tcp-frame", default="gripper_frame", help="TCP frame name.")
    parser.add_argument("--end-link", default=DEFAULT_END_LINK, help="URDF end link before tool-frame offset.")
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="Robot config path for bus access.")
    parser.add_argument("--approach-policy", default=DEFAULT_APPROACH_POLICY_PATH, help="Optional approach policy YAML path for route defaults.")
    parser.add_argument("--output", default=DEFAULT_OUTPUT_PATH, help="Recovery JSON output path.")

    parser.add_argument("--recovery-clearance-m", type=float, default=DEFAULT_RECOVERY_CLEARANCE_M, help="Minimum recovery height above board top.")
    parser.add_argument("--recovery-route-squares", type=parse_square_list, default=None, help="Optional comma-separated recovery route squares, for example e4 or a2,c3,e4.")
    parser.add_argument("--execute", action="store_true", help="Command hardware after all segment checks.")
    parser.add_argument("--confirm", default=None, help="Typed execute confirmation.")

    parser.add_argument("--max-joint-delta-ticks", type=int, default=2000, help="Maximum per-joint segment delta.")
    parser.add_argument("--max-total-l1-delta-ticks", type=int, default=6000, help="Maximum total absolute segment delta.")
    parser.add_argument("--speed-scale", type=float, default=0.20, help="Slow execution speed scale in (0, 1].")
    parser.add_argument("--settle-time-s", type=float, default=2.0, help="Settle time before readback when per-stage values are not provided.")
    parser.add_argument("--intermediate-settle-time-s", type=float, default=0.5, help="Settle time before readback for intermediate staged waypoints.")
    parser.add_argument("--final-settle-time-s", type=float, default=1.5, help="Settle time before readback for final target/home poses.")
    parser.add_argument("--readback-tolerance-ticks", type=int, default=80, help="Final readback tolerance per joint.")
    parser.add_argument("--board-clearance-m", type=float, default=0.060, help="Minimum TCP clearance above board top for XY-changing path samples.")
    parser.add_argument("--path-samples", type=int, default=25, help="Joint-interpolated FK path samples per segment.")
    parser.add_argument("--xy-motion-epsilon-m", type=float, default=0.005, help="XY delta below this is treated as mostly vertical motion.")

    parser.add_argument("--prefer-vertical-approach", action="store_true", help="Prefer a vertical world-down approach.")
    parser.add_argument("--enforce-approach-angle", action="store_true", help="Abort when an enforced segment exceeds approach tilt limits.")
    parser.add_argument("--max-approach-tilt-deg", type=float, default=10.0, help="Maximum approach-axis tilt from world down.")
    parser.add_argument("--max-edge-approach-tilt-deg", type=float, default=20.0, help="Maximum approach-axis tilt for edge squares.")
    parser.add_argument("--approach-weight", type=float, default=0.05, help="Residual weight for vertical-approach preference in IK.")
    parser.add_argument("--approach-axis-name", choices=("plus_x", "minus_x", "plus_y", "minus_y", "plus_z", "minus_z"), default=None, help="Named local tool axis for approach diagnostics.")
    parser.add_argument("--approach-axis-local", nargs=3, type=float, default=None, help="Explicit local tool approach axis XYZ.")

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
    parser.add_argument("--lock-joint", action="append", default=None, help="Lock one arm joint with joint=tick or joint_rad=value.")
    parser.set_defaults(lock_wrist_roll_home=True)
    parser.add_argument("--lock-wrist-roll-home", dest="lock_wrist_roll_home", action="store_true", help="Lock wrist_roll to saved home during IK. Default true.")
    parser.add_argument("--no-lock-wrist-roll", dest="lock_wrist_roll_home", action="store_false", help="Disable wrist_roll home lock for debugging only.")
    parser.add_argument("--square", default=None, help="Optional square context for approach-policy override selection.")
    return parser


def format_optional_float(value):
    if value is None:
        return "unavailable"
    return "%.6f" % float(value)


def print_report(log, args):
    print("Mode: %s" % log.get("mode"))
    print("Recovery needed: %s" % log.get("recovery_needed"))
    print("Recovery available: %s" % log.get("recovery_available"))
    print("Recovery clearance m: %s" % format_optional_float(log.get("recovery_clearance_m")))
    route_squares = log.get("recovery_route_squares") or []
    print("Recovery route squares: %s" % (", ".join(route_squares) if route_squares else "none"))
    print("Segments:")
    for segment in log.get("segments", []):
        path = segment.get("path_validation") or {}
        print(
            "  %d. %s path=%s min_z=%s command=%s abort=%s"
            % (
                int(segment.get("segment_index", 0)),
                segment.get("segment_name"),
                path.get("passed"),
                format_optional_float(path.get("min_z_m")),
                segment.get("command_sent"),
                segment.get("abort_reason") or "",
            )
        )
    if log.get("abort_reason"):
        print("Abort reason: %s" % log.get("abort_reason"))
    print("Command sent any: %s" % log.get("command_sent_any"))
    print("Saved JSON log: %s" % args.output)
    print("Execute confirmation phrase: %s" % CONFIRM_TEXT_RECOVER_HOME)


def main():
    args = build_parser().parse_args()
    log = run_safe_recovery(args)
    print_report(log, args)
    if log.get("aborted"):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
