#!/usr/bin/env python3
from __future__ import absolute_import, print_function

import argparse
import json
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.robot.approach_policy import ApproachPolicyError
from chess_robot.robot.approach_policy import apply_approach_policy
from chess_robot.robot.safe_transfer import CONFIRM_TEXT
from chess_robot.robot.safe_transfer import DEFAULT_CSV_LOG_PATH
from chess_robot.robot.safe_transfer import RETURN_STRATEGY_ACHIEVED_REVERSE_REPLAY
from chess_robot.robot.safe_transfer import RETURN_STRATEGY_RESOLVE_NEW
from chess_robot.robot.safe_transfer import RETURN_STRATEGY_REVERSE_REPLAY
from chess_robot.robot.safe_transfer import run_safe_square_transfer
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
DEFAULT_IK_SEED_POSES_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "ik_seed_poses.yaml")


class PolicyAwareArgumentParser(argparse.ArgumentParser):
    def parse_args(self, args=None, namespace=None):
        explicit_dests = collect_explicit_argument_dests(self, args)
        parsed = argparse.ArgumentParser.parse_args(self, args=args, namespace=namespace)
        try:
            return apply_approach_policy(parsed, explicit_dests)
        except ApproachPolicyError as exc:
            self.error(str(exc))


def collect_explicit_argument_dests(parser, args):
    values = sys.argv[1:] if args is None else list(args)
    option_to_dest = {}
    for action in parser._actions:
        for option_string in action.option_strings:
            option_to_dest[option_string] = action.dest

    explicit_dests = set()
    for token in values:
        if token == "--":
            break
        if not token.startswith("-"):
            continue
        option_name = token.split("=", 1)[0]
        destination = option_to_dest.get(option_name)
        if destination is not None:
            explicit_dests.add(destination)
    return explicit_dests


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
    parser = PolicyAwareArgumentParser(description="Validate a staged safe square-above transfer for the SO101 chess robot.")
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
    parser.add_argument("--csv-log", default=DEFAULT_CSV_LOG_PATH, help="CSV summary append path.")
    parser.add_argument("--square", required=True, help="Board square target, for example e4.")
    parser.add_argument("--approach-policy", default=None, help="Approach policy YAML path, for example %s." % DEFAULT_APPROACH_POLICY_PATH)
    parser.add_argument("--ik-seed-poses", default=None, help="IK seed pose YAML path, for example %s." % DEFAULT_IK_SEED_POSES_PATH)
    parser.add_argument("--ignore-ik-seed-poses", action="store_true", help="Ignore square-specific IK seed poses and preserve current seeding behaviour.")

    parser.add_argument("--normal-above-offset-m", type=float, default=0.080, help="Normal above-square height above board top.")
    parser.add_argument("--high-above-offset-m", type=float, default=0.120, help="High above-square height above board top.")
    parser.add_argument("--route-above-offset-m", type=float, default=None, help="Route waypoint height above board top. Defaults to the high above-square height.")
    parser.add_argument("--transit-clearance-m", type=float, default=0.120, help="Lift height above board top for XY transit.")
    parser.add_argument("--board-clearance-m", type=float, default=0.060, help="Minimum TCP clearance above board top for XY-changing path samples.")
    parser.add_argument("--path-samples", type=int, default=25, help="Joint-interpolated FK path samples per segment.")
    parser.add_argument("--xy-motion-epsilon-m", type=float, default=0.005, help="XY delta below this is treated as mostly vertical motion.")
    parser.add_argument("--prefer-vertical-approach", action="store_true", help="Prefer a vertical world-down approach on square target segments.")
    parser.add_argument("--approach-axis-name", choices=("plus_x", "minus_x", "plus_y", "minus_y", "plus_z", "minus_z"), help="Named local tool axis to treat as the approach direction.")
    parser.add_argument("--approach-axis-local", nargs=3, type=float, help="Explicit local tool approach axis XYZ.")
    parser.add_argument("--enforce-approach-angle", action="store_true", help="Abort when an enforced target segment exceeds its approach tilt limit.")
    parser.add_argument("--max-approach-tilt-deg", type=float, default=10.0, help="Maximum allowed approach-axis tilt from world down.")
    parser.add_argument("--max-edge-approach-tilt-deg", type=float, default=20.0, help="Maximum allowed approach-axis tilt for edge squares.")
    parser.add_argument("--approach-weight", type=float, default=0.05, help="Residual weight for vertical-approach preference in IK.")

    parser.set_defaults(lock_wrist_roll_home=True)
    parser.add_argument("--lock-wrist-roll-home", dest="lock_wrist_roll_home", action="store_true", help="Lock wrist_roll to saved home during IK. Default true.")
    parser.add_argument("--no-lock-wrist-roll", dest="lock_wrist_roll_home", action="store_false", help="Disable wrist_roll home lock for debugging only.")

    parser.add_argument("--execute", action="store_true", help="Command hardware after all segment checks.")
    parser.add_argument("--confirm", default=None, help="Typed execute confirmation.")
    parser.add_argument("--return-home", action="store_true", help="Return to saved home through high waypoints after reaching target normal-above.")
    parser.add_argument("--return-route-squares", type=parse_square_list, default=None, help="Comma-separated high-clearance return route squares, for example a2,c3,e4.")
    parser.add_argument("--return-strategy", choices=(RETURN_STRATEGY_ACHIEVED_REVERSE_REPLAY, RETURN_STRATEGY_REVERSE_REPLAY, RETURN_STRATEGY_RESOLVE_NEW), default=RETURN_STRATEGY_ACHIEVED_REVERSE_REPLAY, help="How to build return segments after the forward target stages.")
    parser.add_argument("--allow-planned-replay-fallback", action="store_true", help="Allow planned-target replay when achieved readback ticks are unavailable for achieved reverse replay.")
    parser.set_defaults(assume_start_home=True)
    parser.add_argument("--assume-start-home", dest="assume_start_home", action="store_true", help="Dry-run start state is saved home. Default true.")
    parser.add_argument("--no-assume-start-home", dest="assume_start_home", action="store_false", help="Reject dry-run until a non-home start source exists.")
    parser.add_argument("--start-from-readback", action="store_true", help="Use live readback as the execute-mode start state.")

    parser.add_argument("--max-joint-delta-ticks", type=int, default=2000, help="Maximum per-joint segment delta.")
    parser.add_argument("--max-total-l1-delta-ticks", type=int, default=6000, help="Maximum total absolute segment delta.")
    parser.add_argument("--speed-scale", type=float, default=0.20, help="Slow execution speed scale in (0, 1].")
    parser.add_argument("--settle-time-s", type=float, default=2.0, help="Settle time before readback when per-stage values are not provided.")
    parser.add_argument("--intermediate-settle-time-s", type=float, default=None, help="Settle time before readback for intermediate staged waypoints.")
    parser.add_argument("--final-settle-time-s", type=float, default=None, help="Settle time before readback for final target/home poses.")
    parser.add_argument("--readback-tolerance-ticks", type=int, default=80, help="Final readback tolerance per joint.")

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
    return parser


def print_report(log, output_path):
    print("Mode: %s" % log["mode"])
    print("Square: %s" % log["square"])
    print("Approach policy path: %s" % (log.get("approach_policy_path") or "none"))
    print("Approach policy square: %s" % (log.get("approach_policy_square") or "none"))
    print("Policy override applied: %s" % bool(log.get("policy_override_applied", False)))
    print("IK seed poses path: %s" % (log.get("ik_seed_poses_path") or "none"))
    print("IK seed square: %s" % (log.get("ik_seed_square") or "none"))
    print("IK seed applied: %s" % bool(log.get("ik_seed_applied", False)))
    if log.get("ik_seed_notes"):
        print("IK seed notes: %s" % json.dumps(log.get("ik_seed_notes"), sort_keys=False))
    if log.get("resolved_policy") is not None:
        print("Resolved policy: %s" % json.dumps(log["resolved_policy"], sort_keys=True))
    print("Return strategy: %s" % log.get("return_strategy"))
    route_squares = log.get("return_route_squares") or []
    print("Return route squares: %s" % (", ".join(route_squares) if route_squares else "none"))
    print("Route above offset m: %s" % format_optional_float(log.get("route_above_offset_m")))
    if log.get("locked_joints"):
        print("Locked joints: %s" % ", ".join(sorted(log["locked_joints"].keys())))
    print("Segments:")
    for segment in log.get("segments", []):
        path = segment.get("path_validation") or {}
        print(
            "  %d. %s replay=%s source=%s ik=%s path=%s tilt=%s min_z=%s settle=%s seed=%s command=%s abort=%s"
            % (
                int(segment["segment_index"]),
                segment["segment_name"],
                segment.get("replay_source_segment") or "-",
                segment.get("replay_source") or "none",
                segment.get("ik_success"),
                path.get("passed"),
                format_optional_float(segment.get("approach_tilt_deg")),
                format_optional_float(path.get("min_z_m")),
                format_optional_float(segment.get("settle_time_s")),
                segment.get("ik_seed_source"),
                segment.get("command_sent"),
                segment.get("abort_reason") or "",
            )
        )
        if segment.get("route_waypoint"):
            print(
                "     %s ik=%s path=%s min_z=%s route_square=%s"
                % (
                    segment["segment_name"],
                    segment.get("ik_success"),
                    path.get("passed"),
                    format_optional_float(path.get("min_z_m")),
                    segment.get("route_square") or "",
                )
            )
    if log.get("abort_reason"):
        print("Abort reason: %s" % log["abort_reason"])
    print("Command sent any: %s" % log.get("command_sent_any"))
    print("Saved JSON log: %s" % output_path)
    print("Execute confirmation phrase: %s" % CONFIRM_TEXT)


def format_optional_float(value):
    if value is None:
        return "unavailable"
    return "%.6f" % float(value)


def main():
    args = build_parser().parse_args()
    log = run_safe_square_transfer(args)
    print_report(log, args.output)
    if log.get("abort_reason"):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
