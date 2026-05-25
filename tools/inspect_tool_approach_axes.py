#!/usr/bin/env python3
from __future__ import absolute_import

import argparse
import json
import os
import sys

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.robot.approach_orientation import WORLD_DOWN_AXIS
from chess_robot.robot.approach_orientation import inspect_candidate_axes
from chess_robot.robot.ik import robot_base_point_to_world
from chess_robot.robot.ik import world_point_to_robot_base
from chess_robot.robot.ik_validation import build_approach_report
from chess_robot.robot.ik_validation import load_validation_context
from chess_robot.robot.ik_validation import select_target_from_args
from chess_robot.robot.ik_validation import solve_single_target_ik
from chess_robot.robot.tool_frames import compute_tcp_transform
from chess_robot.robot.urdf_model import DEFAULT_END_LINK
from chess_robot.robot.reachability import LIMIT_SOURCE_INTERSECTION
from chess_robot.robot.reachability import LIMIT_SOURCE_SOFTWARE
from chess_robot.robot.reachability import LIMIT_SOURCE_URDF

DEFAULT_URDF_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
DEFAULT_SCENE_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "scene_geometry.yaml")
DEFAULT_JOINT_CALIBRATION_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
DEFAULT_JOINT_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_JOINT_SAFETY_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")
DEFAULT_HOME_POSE_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "home_pose.yaml")
DEFAULT_TOOL_FRAMES_PATH = os.path.join(REPO_ROOT, "data", "calibration", "gripper", "tool_frames.yaml")


def build_parser():
    parser = argparse.ArgumentParser(description="Inspect candidate local tool approach axes for one IK target.")
    parser.add_argument("--urdf", default=DEFAULT_URDF_PATH)
    parser.add_argument("--scene", default=DEFAULT_SCENE_PATH)
    parser.add_argument("--joint-calibration", default=DEFAULT_JOINT_CALIBRATION_PATH)
    parser.add_argument("--joint-limits", default=DEFAULT_JOINT_LIMITS_PATH)
    parser.add_argument("--joint-safety-limits", default=DEFAULT_JOINT_SAFETY_LIMITS_PATH)
    parser.add_argument("--home-pose", default=DEFAULT_HOME_POSE_PATH)
    parser.add_argument("--tool-frames", default=DEFAULT_TOOL_FRAMES_PATH)
    parser.add_argument("--tcp-frame", default="gripper_frame")
    parser.add_argument("--end-link", default=DEFAULT_END_LINK)
    parser.add_argument("--output", required=True)

    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--square")
    target_group.add_argument("--capture", action="store_true")
    target_group.add_argument("--target-world", nargs=3, type=float)
    parser.add_argument("--target-type", choices=("above", "surface"), default="above")
    parser.add_argument("--above-board-offset-m", type=float, default=0.080)
    parser.add_argument("--pick-offset-m", type=float, default=0.030)
    parser.add_argument("--capture-above-offset-m", type=float, default=0.080)
    parser.add_argument("--lock-wrist-roll-home", action="store_true")
    parser.add_argument("--lock-joint", action="append", default=None)
    parser.add_argument("--workspace-seed-samples", type=int, default=1000)
    parser.add_argument("--random-seeds", type=int, default=20)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--tolerance-m", type=float, default=0.005)
    parser.add_argument("--damping", type=float, default=0.05)
    parser.add_argument("--max-iters", type=int, default=200)
    parser.add_argument("--step-scale", type=float, default=1.0)
    parser.add_argument(
        "--limit-source",
        choices=(LIMIT_SOURCE_URDF, LIMIT_SOURCE_SOFTWARE, LIMIT_SOURCE_INTERSECTION),
        default=LIMIT_SOURCE_INTERSECTION,
    )
    parser.add_argument("--prefer-vertical-approach", action="store_true")
    parser.add_argument("--approach-axis-name", choices=("plus_x", "minus_x", "plus_y", "minus_y", "plus_z", "minus_z"))
    parser.add_argument("--approach-axis-local", nargs=3, type=float)
    parser.add_argument("--max-approach-tilt-deg", type=float, default=10.0)
    parser.add_argument("--max-edge-approach-tilt-deg", type=float, default=20.0)
    parser.add_argument("--approach-weight", type=float, default=0.05)
    parser.add_argument("--enforce-approach-angle", action="store_true")
    return parser


def ensure_parent_dir(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.exists(parent):
        os.makedirs(parent)


def format_xyz(values):
    return "[%.6f, %.6f, %.6f]" % (float(values[0]), float(values[1]), float(values[2]))


def main():
    args = build_parser().parse_args()
    context = load_validation_context(args)
    target = select_target_from_args(context, args)
    if target.get("target_robot_xyz_m") is not None:
        target_robot = np.asarray(target["target_robot_xyz_m"], dtype=float)
    else:
        target_robot = world_point_to_robot_base(target["target_world_xyz_m"], context["scene_geometry"])
    result = solve_single_target_ik(
        context,
        target_robot,
        args,
        square=target.get("square"),
    )
    robot_T_tcp = compute_tcp_transform(
        context["model"],
        result.joint_positions_rad,
        end_link=context.get("end_link", DEFAULT_END_LINK),
        tool_frame=context["tool_frame"],
    )
    world_T_tcp = np.dot(np.asarray(context["scene_geometry"]["world_T_robot_base"], dtype=float), robot_T_tcp)
    candidate_axes = inspect_candidate_axes(world_T_tcp, reference_down_axis=WORLD_DOWN_AXIS)
    best_candidate = candidate_axes[0]
    selected_report = build_approach_report(
        context,
        args,
        result.joint_positions_rad,
        square=target.get("square"),
        prefer_vertical_approach=bool(getattr(args, "prefer_vertical_approach", False)),
        enforce_approach_angle=bool(getattr(args, "enforce_approach_angle", False)),
    )
    warning = None
    if all(float(item["tilt_deg"]) > 45.0 for item in candidate_axes):
        warning = "All candidate local axes exceed 45 deg tilt from world down."
    output = {
        "target_name": target.get("target_name"),
        "square": target.get("square"),
        "target_type": target.get("target_type"),
        "target_world_xyz_m": target.get("target_world_xyz_m"),
        "target_robot_xyz_m": [float(value) for value in target_robot],
        "final_tcp_world_xyz_m": [float(value) for value in robot_base_point_to_world(result.final_xyz_robot, context["scene_geometry"])],
        "final_tcp_robot_xyz_m": [float(value) for value in result.final_xyz_robot],
        "ik_success": bool(result.success),
        "ik_status": str(result.status),
        "ik_error_m": float(result.error_m),
        "ik_iterations": int(result.iterations),
        "candidate_axes": candidate_axes,
        "best_candidate_axis": best_candidate,
        "selected_axis_report": selected_report,
        "warning": warning,
    }
    ensure_parent_dir(args.output)
    with open(args.output, "w") as handle:
        json.dump(output, handle, indent=2, sort_keys=True)

    print("Target: %s" % output["target_name"])
    print("IK: success=%s status=%s error=%.3f mm" % (output["ik_success"], output["ik_status"], float(output["ik_error_m"]) * 1000.0))
    print("Final TCP world XYZ (m): %s" % format_xyz(output["final_tcp_world_xyz_m"]))
    print("Candidate local axes by tilt to world down:")
    for item in candidate_axes:
        print("  %s tilt=%.3f deg world=%s" % (item["axis_name"], float(item["tilt_deg"]), format_xyz(item["axis_world"])))
    print("Best candidate axis: %s tilt=%.3f deg" % (best_candidate["axis_name"], float(best_candidate["tilt_deg"])))
    if warning:
        print("Warning: %s" % warning)
    print("Saved JSON log: %s" % args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
