#!/usr/bin/env python3
"""Record a safe home pose from live servo positions.

This tool only reads servo state. It does not command motion, write goal
positions, or toggle torque.
"""

import argparse
import datetime
import json
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    import yaml
except ImportError:  # pragma: no cover - depends on runtime image
    yaml = None

from chess_robot.robot import safety
from chess_robot.robot.servo_bus import ServoBusError, build_servo_bus, load_robot_config

CANONICAL_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

DEFAULT_CONFIG_PATH = os.path.join(ROOT, "configs", "robot.yaml")
DEFAULT_SERVO_MAP_PATH = os.path.join(ROOT, "data", "calibration", "robot", "servo_map.yaml")
DEFAULT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_HOME_POSE_PATH = os.path.join(ROOT, "data", "calibration", "robot", "home_pose.yaml")


def _now_utc() -> str:
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def _require_yaml() -> None:
    if yaml is None:
        raise ServoBusError("PyYAML is required for calibration files.")


def _read_yaml(path: str) -> Dict[str, Any]:
    _require_yaml()
    if not os.path.exists(path):
        raise ServoBusError("Required calibration file not found: {}".format(path))
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ServoBusError("Calibration file must contain a YAML mapping: {}".format(path))
    return data


def _yaml_scalar(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return repr(value)
    return json.dumps(value)


def _write_home_pose(path: str, pose: Dict[str, Any]) -> None:
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)

    lines = [
        "created_at_utc: {}".format(_yaml_scalar(pose["created_at_utc"])),
        "notes: {}".format(_yaml_scalar(pose["notes"])),
        "source: {}".format(_yaml_scalar(pose["source"])),
        "joints:",
    ]
    joints = pose.get("joints") or {}
    for joint_name in CANONICAL_JOINTS:
        entry = joints[joint_name]
        lines.append("  {}:".format(joint_name))
        lines.append("    id: {}".format(_yaml_scalar(entry["id"])))
        lines.append("    position: {}".format(_yaml_scalar(entry["position"])))

    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def _as_int(value: Any) -> Optional[int]:
    if isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _load_servo_map(path: str) -> Dict[str, Any]:
    data = _read_yaml(path)
    joints = data.get("joints") or {}
    aliases = data.get("aliases") or {}
    if not isinstance(joints, dict):
        raise ServoBusError("servo_map.yaml 'joints' must be a mapping.")
    if not isinstance(aliases, dict):
        raise ServoBusError("servo_map.yaml 'aliases' must be a mapping.")

    result = {"joints": {}, "aliases": {}}
    for joint_name in CANONICAL_JOINTS:
        entry = joints.get(joint_name)
        if not isinstance(entry, dict):
            raise ServoBusError("Missing canonical joint {} in servo map.".format(joint_name))
        servo_id = entry.get("id", entry.get("servo_id"))
        if servo_id is None:
            raise ServoBusError("Joint {} is missing an id in servo_map.yaml.".format(joint_name))
        result["joints"][joint_name] = {
            "id": safety.validate_servo_id(servo_id),
            "calibrated": bool(entry.get("calibrated", False)),
        }
    for alias, canonical in aliases.items():
        if canonical not in CANONICAL_JOINTS:
            raise ServoBusError("Alias {} points to unknown joint {}.".format(alias, canonical))
        result["aliases"][str(alias)] = canonical
    return result


def _load_limits(path: str) -> Dict[str, Any]:
    data = _read_yaml(path)
    limits = data.get("limits") or data.get("joints") or {}
    if not isinstance(limits, dict):
        raise ServoBusError("joint_limits.yaml must contain a 'limits' mapping.")
    return {"limits": limits}


def _limit_entry(limits_data: Dict[str, Any], joint_name: str) -> Optional[Dict[str, Any]]:
    limits = limits_data.get("limits") or limits_data.get("joints") or {}
    entry = limits.get(joint_name)
    if not isinstance(entry, dict):
        return None
    return entry


def _validate_and_collect_pose(
    servo_map: Dict[str, Any],
    limits_data: Dict[str, Any],
    live_positions: Dict[str, Optional[int]],
) -> Tuple[List[str], Dict[str, Dict[str, int]]]:
    issues = []
    pose_joints = {}

    for joint_name in CANONICAL_JOINTS:
        servo_entry = (servo_map.get("joints") or {}).get(joint_name)
        if not isinstance(servo_entry, dict):
            issues.append("{} missing from servo_map.yaml".format(joint_name))
            continue

        limit_entry = _limit_entry(limits_data, joint_name)
        if not isinstance(limit_entry, dict):
            issues.append("{} missing from joint_limits.yaml".format(joint_name))
            continue

        if not bool(limit_entry.get("calibrated")):
            issues.append("{} joint limits are not calibrated".format(joint_name))

        min_value = _as_int(limit_entry.get("provisional_min"))
        max_value = _as_int(limit_entry.get("provisional_max"))
        if min_value is None or max_value is None:
            issues.append("{} missing provisional_min/provisional_max".format(joint_name))
            continue
        if min_value >= max_value:
            issues.append("{} provisional_min must be less than provisional_max".format(joint_name))
            continue

        live_position = live_positions.get(joint_name)
        if live_position is None:
            issues.append("{} live position unavailable".format(joint_name))
            continue
        if live_position < min_value or live_position > max_value:
            issues.append(
                "{} live position {} outside [{}, {}]".format(
                    joint_name,
                    live_position,
                    min_value,
                    max_value,
                )
            )
            continue

        pose_joints[joint_name] = {
            "id": servo_entry["id"],
            "position": live_position,
        }

    return issues, pose_joints


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record a safe home pose from live servo positions. This tool never commands motion."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH,
                        help="Path to robot YAML config. Default: configs/robot.yaml")
    parser.add_argument("--servo-map", dest="servo_map_path", default=DEFAULT_SERVO_MAP_PATH,
                        help="Path to data/calibration/robot/servo_map.yaml")
    parser.add_argument("--limits", dest="limits_path", default=DEFAULT_LIMITS_PATH,
                        help="Path to data/calibration/robot/joint_limits.yaml")
    parser.add_argument("--output", dest="output_path", default=DEFAULT_HOME_POSE_PATH,
                        help="Path to data/calibration/robot/home_pose.yaml")
    parser.add_argument("--backend", choices=("mock", "feetech"), default=None,
                        help="Backend override. Defaults to the backend in configs/robot.yaml.")
    parser.add_argument("--save", action="store_true",
                        help="Write the validated live pose to home_pose.yaml.")
    parser.add_argument("--notes", default="",
                        help="Optional notes to store with the recorded pose.")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    bus = None
    try:
        config = load_robot_config(args.config)
        servo_map = _load_servo_map(args.servo_map_path)
        limits_data = _load_limits(args.limits_path)
        mock_ids = [servo_map["joints"][joint_name]["id"] for joint_name in CANONICAL_JOINTS]
        dry_run = None
        if args.backend == "feetech":
            dry_run = False
        elif args.backend == "mock":
            dry_run = True
        bus = build_servo_bus(
            config=config,
            config_path=args.config,
            dry_run=dry_run,
            backend_name=args.backend,
            mock_ids=mock_ids,
        )
    except Exception as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        return 1

    live_positions = {}
    try:
        bus.logger.log(
            "home_pose_read_start",
            backend=bus.backend.name,
            dry_run=bus.dry_run,
            joints=list(CANONICAL_JOINTS),
        )
        for joint_name in CANONICAL_JOINTS:
            servo_id = servo_map["joints"][joint_name]["id"]
            live_positions[joint_name] = bus.read_position(servo_id)
        issues, pose_joints = _validate_and_collect_pose(servo_map, limits_data, live_positions)
        if issues:
            print("Home pose validation failed:", file=sys.stderr)
            for issue in issues:
                print("  - {}".format(issue), file=sys.stderr)
            return 1

        print("Home pose validated from {} backend.".format(bus.backend.name))
        for joint_name in CANONICAL_JOINTS:
            entry = pose_joints[joint_name]
            print(
                "  {}: {} within calibrated range".format(
                    joint_name,
                    entry["position"],
                )
            )

        if not args.save:
            print("Not saved. Re-run with --save to write {}.".format(args.output_path))
            return 0

        pose_data = {
            "created_at_utc": _now_utc(),
            "notes": args.notes,
            "source": "live_read",
            "joints": pose_joints,
        }
        _write_home_pose(args.output_path, pose_data)
        bus.logger.log(
            "home_pose_save",
            backend=bus.backend.name,
            dry_run=bus.dry_run,
            path=args.output_path,
            source="live_read",
            joint_count=len(pose_joints),
            positions={name: entry["position"] for name, entry in pose_joints.items()},
        )
        print("Saved home pose to {}.".format(args.output_path))
        print("Log: {}".format(bus.logger.path))
        return 0
    except Exception as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        return 1
    finally:
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
