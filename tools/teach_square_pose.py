#!/usr/bin/env python3
from __future__ import absolute_import

import argparse
import os
import sys
from typing import Dict

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map
from chess_robot.robot.servo_bus import build_servo_bus, load_robot_config

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG_PATH = os.path.join(ROOT, "configs", "robot.yaml")
DEFAULT_TARGETS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "square_targets.yaml")
DEFAULT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_SERVO_MAP_PATH = os.path.join(ROOT, "data", "calibration", "robot", "servo_map.yaml")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record one manual square pose without moving the robot.")
    parser.add_argument("--square", required=True, help="Target square, for example e4.")
    parser.add_argument("--targets", default=DEFAULT_TARGETS_PATH, help="Square-target YAML path.")
    parser.add_argument("--pose-name", default="above_pose", choices=robot_square_map.ALLOWED_POSE_NAMES,
                        help="Pose entry to record. Defaults to above_pose.")
    parser.add_argument("--joint-limits", dest="joint_limits_path", default=DEFAULT_LIMITS_PATH,
                        help="Joint-limits YAML path.")
    parser.add_argument("--servo-map", dest="servo_map_path", default=DEFAULT_SERVO_MAP_PATH,
                        help="Servo-map YAML path.")
    parser.add_argument("--from-live-readback", action="store_true",
                        help="Read current joint ticks through the existing read-only servo backend.")
    parser.add_argument("--joints", default=None,
                        help="Comma-separated joint ticks, for example shoulder_pan=2048,...")
    parser.add_argument("--note", default=None, help="Optional note stored with the manual pose.")
    parser.add_argument("--write", action="store_true", help="Write the updated YAML file.")
    parser.add_argument("--force", action="store_true", help="Replace an existing manual pose of the selected pose type.")
    return parser


def _parse_joint_values(raw: str, joint_order) -> Dict[str, int]:
    if not raw:
        raise ValueError("--joints must not be empty.")
    parsed = {}
    for item in raw.split(","):
        chunk = item.strip()
        if not chunk:
            continue
        if "=" not in chunk:
            raise ValueError("Invalid joint assignment: {}".format(chunk))
        joint_name, value = chunk.split("=", 1)
        joint_name = joint_name.strip()
        value = value.strip()
        if joint_name in parsed:
            raise ValueError("Duplicate joint assignment for {}".format(joint_name))
        try:
            parsed[joint_name] = int(value)
        except ValueError:
            raise ValueError("Joint {} must have an integer tick value".format(joint_name))
    issues = []
    for joint_name in joint_order:
        if joint_name not in parsed:
            issues.append(joint_name)
    if issues:
        raise ValueError("Missing joints: {}".format(", ".join(issues)))
    unknown = [joint_name for joint_name in parsed if joint_name not in joint_order]
    if unknown:
        raise ValueError("Unknown joints: {}".format(", ".join(sorted(unknown))))
    return parsed


def _servo_ids_by_joint(servo_map, joint_order):
    joints = servo_map.get("joints") or {}
    mapping = {}
    for joint_name in joint_order:
        joint_info = joints.get(joint_name)
        if not isinstance(joint_info, dict):
            raise robot_square_map.SquareTargetError("servo_map entry missing for joint {}".format(joint_name))
        servo_id = joint_info.get("id")
        if isinstance(servo_id, bool) or not isinstance(servo_id, int):
            raise robot_square_map.SquareTargetError("servo_map id missing for joint {}".format(joint_name))
        mapping[joint_name] = servo_id
    return mapping


def _read_live_joint_positions(servo_map, joint_order):
    config = load_robot_config(DEFAULT_CONFIG_PATH)
    bus = build_servo_bus(
        config=config,
        config_path=DEFAULT_CONFIG_PATH,
        dry_run=False,
        backend_name="feetech",
        mock_ids=None,
    )
    joints = {}
    errors = []
    try:
        ids_by_joint = _servo_ids_by_joint(servo_map, joint_order)
        for joint_name in joint_order:
            servo_id = ids_by_joint[joint_name]
            ping_ok = bus.ping(servo_id)
            if not ping_ok:
                errors.append("servo {} ({}) did not respond to ping".format(servo_id, joint_name))
                continue
            position = bus.read_position(servo_id)
            if position is None:
                errors.append("servo {} ({}) present position unavailable".format(servo_id, joint_name))
                continue
            joints[joint_name] = int(position)
    finally:
        bus.close()
    if errors:
        raise robot_square_map.SquareTargetError("Live readback failed: {}".format("; ".join(errors)))
    return joints


def main() -> int:
    args = build_parser().parse_args()
    square_name = robot_square_map.normalise_square_name(args.square)
    try:
        pose_name = robot_square_map.validate_pose_name(args.pose_name)
    except robot_square_map.SquareTargetError as exc:
        print("ERROR: {}".format(exc))
        return 1
    targets = robot_square_map.load_square_targets(args.targets)
    joint_limits = robot_square_map.load_joint_limits(args.joint_limits_path)
    servo_map = robot_square_map.load_servo_map(args.servo_map_path)
    joint_order = list(targets.get("joint_order") or robot_square_map.DEFAULT_JOINT_ORDER)

    if bool(args.from_live_readback) == bool(args.joints):
        print("ERROR: Choose exactly one of --from-live-readback or --joints.")
        return 1

    try:
        if args.from_live_readback:
            joints = _read_live_joint_positions(servo_map, joint_order)
        else:
            joints = _parse_joint_values(args.joints, joint_order)
    except (ValueError, robot_square_map.SquareTargetError) as exc:
        print("ERROR: {}".format(exc))
        return 1

    issues = robot_square_map.validate_pose_joints(joints, joint_limits, joint_order)
    existing_square = targets.get("squares", {}).get(square_name, {})
    existing_pose = existing_square.get(pose_name) if isinstance(existing_square, dict) else None
    existing_source = existing_pose.get("source") if isinstance(existing_pose, dict) else None
    would_overwrite = isinstance(existing_pose, dict)

    if issues:
        print("Square: {}".format(square_name))
        print("Pose name: {}".format(pose_name))
        print("Source: manual")
        print("Validation: FAILED")
        print("Issues: {}".format("; ".join(issues)))
        print("File written: no")
        print("Would overwrite existing pose: {}".format("yes" if would_overwrite else "no"))
        print("Notes added: {}".format(args.note if args.note else "none"))
        return 1

    try:
        updated = robot_square_map.upsert_manual_pose(
            targets,
            square_name,
            pose_name,
            joints,
            notes=[args.note] if args.note else None,
            force=args.force,
        )
    except robot_square_map.SquareTargetError as exc:
        print("ERROR: {}".format(exc))
        return 1

    if args.write:
        robot_square_map.save_yaml_file(args.targets, updated)

    overwrite_label = "no"
    if would_overwrite:
        overwrite_label = "yes" if args.write else "would overwrite"
    print("Square: {}".format(square_name))
    print("Pose name: {}".format(pose_name))
    print("Source: manual")
    print("Joints: {}".format(", ".join(["{}={}".format(name, joints[name]) for name in joint_order])))
    print("Validation: OK")
    print("File written: {}".format("yes" if args.write else "no"))
    print("Would write target file: {}".format("yes" if args.write else "no"))
    print("Existing pose source: {}".format(existing_source or "none"))
    print("Existing pose overwritten: {}".format(overwrite_label))
    print("Notes added: {}".format(args.note if args.note else "none"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
