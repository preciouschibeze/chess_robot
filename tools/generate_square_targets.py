#!/usr/bin/env python3
from __future__ import absolute_import

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_TARGETS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "square_targets.yaml")
DEFAULT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate interpolated above-square poses from taught anchors.")
    parser.add_argument("--targets", default=DEFAULT_TARGETS_PATH, help="Square-target YAML path.")
    parser.add_argument("--joint-limits", dest="joint_limits_path", default=DEFAULT_LIMITS_PATH,
                        help="Joint-limits YAML path.")
    parser.add_argument("--method", choices=("idw",), default="idw", help="Interpolation method.")
    parser.add_argument("--write", action="store_true", help="Write generated poses back to YAML.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    targets = robot_square_map.load_square_targets(args.targets)
    joint_limits = robot_square_map.load_joint_limits(args.joint_limits_path)
    result = robot_square_map.generate_square_targets(targets, joint_limits, method=args.method)

    print("Manual anchors used: {}".format(", ".join(result.get("manual_anchor_squares") or []) or "none"))
    print("Missing recommended anchors: {}".format(
        ", ".join(result.get("missing_recommended_anchors") or []) or "none"
    ))
    print("Generated square count: {}".format(result.get("generated_count", 0)))
    print("Skipped manual square count: {}".format(result.get("skipped_manual_count", 0)))
    if result.get("warnings"):
        for warning in result.get("warnings"):
            print("Warning: {}".format(warning))
    if result.get("generated_validation_errors"):
        print("Validation summary: FAILED")
        for square_name in sorted(result.get("generated_validation_errors", {}).keys()):
            print("  {}: {}".format(square_name, "; ".join(result["generated_validation_errors"][square_name])))
    else:
        print("Validation summary: OK")
    print("Output path: {}".format(args.targets))

    if args.write:
        if result.get("manual_anchor_count", 0) < robot_square_map.MIN_MANUAL_ANCHORS_FOR_WRITE:
            print("ERROR: Refusing --write with fewer than 9 manual anchors.")
            return 1
        if result.get("generated_validation_errors"):
            print("ERROR: Refusing --write because generated poses violate joint limits.")
            return 1
        robot_square_map.save_yaml_file(args.targets, result.get("data"))
        print("File written: yes")
    else:
        print("File written: no")
    return 0


if __name__ == "__main__":
    sys.exit(main())
