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
    parser = argparse.ArgumentParser(description="Inspect square-target calibration data.")
    parser.add_argument("--targets", default=DEFAULT_TARGETS_PATH, help="Square-target YAML path.")
    parser.add_argument("--joint-limits", dest="joint_limits_path", default=DEFAULT_LIMITS_PATH,
                        help="Joint-limits YAML path.")
    parser.add_argument("--show-missing", action="store_true", help="Include squares with no above_pose.")
    parser.add_argument("--show-generated", action="store_true", help="Include generated above poses.")
    parser.add_argument("--show-manual", action="store_true", help="Include manual above poses.")
    parser.add_argument("--recommended-only", action="store_true", help="Show recommended anchors only.")
    return parser


def _should_show_source(args, source):
    if not args.show_generated and not args.show_manual:
        return source in ("manual", "generated")
    if source == "manual":
        return bool(args.show_manual)
    if source == "generated":
        return bool(args.show_generated)
    return False


def main() -> int:
    args = build_parser().parse_args()
    targets = robot_square_map.load_square_targets(args.targets)
    joint_limits = robot_square_map.load_joint_limits(args.joint_limits_path)
    counts = robot_square_map.count_pose_sources(targets)
    anchors = robot_square_map.collect_manual_anchor_squares(targets)
    missing_recommended = robot_square_map.missing_recommended_anchors(targets)
    rows = robot_square_map.square_status_rows(targets, joint_limits)
    recommended = set(targets.get("recommended_anchors") or robot_square_map.RECOMMENDED_ANCHORS)

    print("Manual above poses: {}".format(counts.get("manual", 0)))
    print("Generated above poses: {}".format(counts.get("generated", 0)))
    print("Usable manual anchors: {}".format(len(anchors)))
    print("Missing recommended anchors: {}".format(", ".join(missing_recommended) if missing_recommended else "none"))
    print("Minimum 9-anchor fallback satisfied: {}".format("yes" if len(anchors) >= robot_square_map.MIN_MANUAL_ANCHORS_FOR_WRITE else "no"))

    for row in rows:
        square_name = row.get("square")
        source = row.get("source")
        if args.recommended_only and square_name not in recommended:
            continue
        if source is None:
            if args.show_missing:
                print("{}: source=missing validation=missing above_pose".format(square_name))
            continue
        if not _should_show_source(args, source):
            continue
        issues = row.get("issues") or []
        validation = "OK" if not issues else "FAILED ({})".format("; ".join(issues))
        print("{}: source={} validation={}".format(square_name, source, validation))
    return 0


if __name__ == "__main__":
    sys.exit(main())
