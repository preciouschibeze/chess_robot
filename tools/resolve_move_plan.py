#!/usr/bin/env python
"""Resolve symbolic move-plan actions into dry-run primitive steps."""

from __future__ import absolute_import

import argparse
import json
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.robot.motion_primitives import resolve_move_plan


def parse_args():
    parser = argparse.ArgumentParser(description="Dry-run move-plan primitive resolver")
    parser.add_argument(
        "--plan",
        default="data/debug/latest_move_plan.json",
        help="Path to input move plan JSON",
    )
    parser.add_argument(
        "--square-targets",
        default="data/calibration/robot/square_targets.yaml",
        help="Path to square targets YAML",
    )
    parser.add_argument(
        "--home-pose",
        default="data/calibration/robot/home_pose.yaml",
        help="Path to home pose YAML",
    )
    parser.add_argument(
        "--gripper-profile",
        default="data/calibration/gripper/gripper_profile.yaml",
        help="Path to gripper profile YAML",
    )
    parser.add_argument(
        "--output",
        default="data/debug/latest_resolved_move_plan.json",
        help="Path to output resolved JSON",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    with open(args.plan, "r") as handle:
        plan = json.load(handle)

    result = resolve_move_plan(
        plan_dict=plan,
        square_targets_path=args.square_targets,
        home_pose_path=args.home_pose,
        gripper_profile_path=args.gripper_profile,
    )

    result_dict = result.to_dict()
    result_dict["source_plan_path"] = args.plan

    with open(args.output, "w") as handle:
        json.dump(result_dict, handle, indent=2, sort_keys=True)
        handle.write("\n")

    missing = result.missing_calibration
    print("source plan: %s" % args.plan)
    print("supported: %s" % result.supported)
    print("ready_for_execution: %s" % result.ready_for_execution)
    print("steps: %d" % len(result.steps))
    print("missing calibration count: %d" % len(missing))
    if missing:
        print("missing calibration (first entries): %s" % ", ".join(missing[:8]))
    print("output path: %s" % args.output)

    if result.ready_for_execution:
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
