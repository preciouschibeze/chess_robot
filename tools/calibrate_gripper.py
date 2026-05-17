#!/usr/bin/env python3
"""Record gripper calibration metadata without commanding motion.

This tool only reads the current gripper position when a backend is provided.
It does not write goal positions, open/close the gripper, or move any servo.
"""

from __future__ import print_function

import argparse
import datetime
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    import yaml
except ImportError:  # pragma: no cover - depends on runtime image
    yaml = None

from chess_robot.robot import safety
from chess_robot.robot.servo_bus import (  # noqa: E402
    ServoBusError,
    ServoEventLogger,
    build_servo_bus,
    load_robot_config,
)

PROFILE_PATH = os.path.join(ROOT, "data", "calibration", "gripper", "gripper_profile.yaml")
SERVO_LOG_PATH = os.path.join(ROOT, "data", "logs", "servo.log")
SERVO_MAP_PATH = os.path.join(ROOT, "data", "calibration", "robot", "servo_map.yaml")
JOINT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
JOINT_DIRECTIONS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_directions.yaml")

GRIPPER_JOINT = "gripper"
GRIPPER_SERVO_ID = 6
GRIPPER_MIN = 1033
GRIPPER_MAX = 1293
GRIPPER_NEUTRAL = 1164
GRIPPER_DIRECTION_SIGN = -1
GRIPPER_POSITIVE_DESCRIPTION = (
    "decreasing ticks closes the moving jaw against the fixed jaw"
)
GRIPPER_NOTES = [
    "Asymmetric gripper: one moving jaw, one fixed jaw.",
    "Grasp position is not full mechanical close.",
    "Open position must fit the largest chess piece.",
]

POSITION_FIELDS = (
    "open_position",
    "pre_grasp_position",
    "grasp_position",
    "release_position",
    "neutral_position",
)


def _now_utc():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def _require_yaml():
    if yaml is None:
        raise ServoBusError("PyYAML is required for gripper calibration files.")


def _read_yaml(path):
    _require_yaml()
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ServoBusError("{} must contain a YAML mapping.".format(path))
    return data


def _write_yaml(path, data):
    _require_yaml()
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, default_flow_style=False)


def _require_mapping(value, path, label):
    if not isinstance(value, dict):
        raise ServoBusError("{} must contain a {} mapping.".format(path, label))
    return value


def _to_int(value, label):
    if isinstance(value, bool):
        raise ServoBusError("{} must be an integer.".format(label))
    try:
        return int(value)
    except (TypeError, ValueError):
        raise ServoBusError("{} must be an integer.".format(label))


def _optional_int(value, label):
    if value is None:
        return None
    return _to_int(value, label)


def _load_servo_map(path):
    data = _read_yaml(path)
    joints = _require_mapping(data.get("joints") or {}, path, "'joints'")
    gripper_entry = _require_mapping(joints.get(GRIPPER_JOINT) or {}, path, "gripper joint")
    servo_id = _to_int(gripper_entry.get("id"), "servo_map.yaml gripper id")
    if servo_id != GRIPPER_SERVO_ID:
        raise ServoBusError(
            "servo_map.yaml gripper id must be {}.".format(GRIPPER_SERVO_ID)
        )
    return gripper_entry


def _load_joint_limits(path):
    data = _read_yaml(path)
    limits = _require_mapping(data.get("limits") or {}, path, "'limits'")
    gripper_entry = _require_mapping(limits.get(GRIPPER_JOINT) or {}, path, "gripper limits")
    if not bool(gripper_entry.get("calibrated")):
        raise ServoBusError("Gripper is not calibrated in joint_limits.yaml.")

    min_value = _to_int(gripper_entry.get("provisional_min"), "joint_limits.yaml gripper provisional_min")
    max_value = _to_int(gripper_entry.get("provisional_max"), "joint_limits.yaml gripper provisional_max")
    neutral = _to_int(gripper_entry.get("neutral"), "joint_limits.yaml gripper neutral")
    if min_value != GRIPPER_MIN or max_value != GRIPPER_MAX or neutral != GRIPPER_NEUTRAL:
        raise ServoBusError(
            "Gripper limits must remain at provisional_min={}, provisional_max={}, neutral={}.".format(
                GRIPPER_MIN, GRIPPER_MAX, GRIPPER_NEUTRAL
            )
        )
    return gripper_entry


def _load_joint_directions(path):
    data = _read_yaml(path)
    directions = _require_mapping(data.get("directions") or {}, path, "'directions'")
    gripper_entry = _require_mapping(directions.get(GRIPPER_JOINT) or {}, path, "gripper directions")
    sign = _to_int(gripper_entry.get("sign"), "joint_directions.yaml gripper sign")
    positive_description = str(gripper_entry.get("positive_description") or "").strip()
    if sign != GRIPPER_DIRECTION_SIGN:
        raise ServoBusError(
            "Gripper direction sign must be {}.".format(GRIPPER_DIRECTION_SIGN)
        )
    if positive_description != GRIPPER_POSITIVE_DESCRIPTION:
        raise ServoBusError(
            "Gripper positive_description does not match the known calibration."
        )
    return gripper_entry


def _normalize_notes(notes):
    if notes is None:
        return []
    if isinstance(notes, list):
        raw = notes
    else:
        raw = [notes]
    result = []
    for note in raw:
        text = str(note).strip()
        if text and text not in result:
            result.append(text)
    return result


def _merge_gripper_profile(existing, servo_entry, limits_entry, directions_entry):
    profile = {}
    if isinstance(existing, dict):
        profile.update(existing)

    profile["joint"] = profile.get("joint") or GRIPPER_JOINT
    profile["servo_id"] = _optional_int(
        profile.get("servo_id") if profile.get("servo_id") is not None else servo_entry.get("id"),
        "gripper profile servo_id",
    )
    profile["direction_sign"] = _optional_int(
        profile.get("direction_sign") if profile.get("direction_sign") is not None else directions_entry.get("sign"),
        "gripper profile direction_sign",
    )

    merged_limits = {}
    if isinstance(profile.get("limits"), dict):
        merged_limits.update(profile["limits"])
    merged_limits["min"] = _optional_int(
        merged_limits.get("min") if merged_limits.get("min") is not None else limits_entry.get("provisional_min"),
        "gripper profile limits.min",
    )
    merged_limits["max"] = _optional_int(
        merged_limits.get("max") if merged_limits.get("max") is not None else limits_entry.get("provisional_max"),
        "gripper profile limits.max",
    )
    profile["limits"] = merged_limits

    existing_notes = _normalize_notes(profile.get("notes"))
    merged_notes = []
    for note in existing_notes + GRIPPER_NOTES:
        if note not in merged_notes:
            merged_notes.append(note)
    profile["notes"] = merged_notes

    for field in POSITION_FIELDS:
        profile[field] = _optional_int(profile.get(field), "gripper profile {}".format(field))

    return profile


def _selected_record_field(args):
    selected = []
    if args.record_open:
        selected.append("open_position")
    if args.record_pre_grasp:
        selected.append("pre_grasp_position")
    if args.record_grasp:
        selected.append("grasp_position")
    if args.record_release:
        selected.append("release_position")
    if args.record_neutral:
        selected.append("neutral_position")
    if len(selected) > 1:
        raise ServoBusError("Only one --record-* flag may be used at a time.")
    if selected:
        return selected[0]
    return None


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Record gripper calibration positions without commanding motion."
    )
    parser.add_argument(
        "--config",
        default=None,
        help="Optional robot config path. Defaults to configs/robot.yaml.",
    )
    parser.add_argument(
        "--backend",
        default=None,
        help="Servo backend for live reads, e.g. feetech. If omitted, no hardware is read.",
    )
    parser.add_argument(
        "--record-open",
        action="store_true",
        help="Record the current live position as open_position.",
    )
    parser.add_argument(
        "--record-pre-grasp",
        action="store_true",
        help="Record the current live position as pre_grasp_position.",
    )
    parser.add_argument(
        "--record-grasp",
        action="store_true",
        help="Record the current live position as grasp_position.",
    )
    parser.add_argument(
        "--record-release",
        action="store_true",
        help="Record the current live position as release_position.",
    )
    parser.add_argument(
        "--record-neutral",
        action="store_true",
        help="Record the current live position as neutral_position.",
    )
    return parser


def _save_profile(profile_root, current_position, record_field, bus_logger, backend_name, servo_entry, limits_entry, directions_entry):
    gripper_profile = profile_root.get(GRIPPER_JOINT)
    if not isinstance(gripper_profile, dict):
        gripper_profile = {}

    merged_profile = _merge_gripper_profile(
        gripper_profile,
        servo_entry,
        limits_entry,
        directions_entry,
    )
    merged_profile[record_field] = current_position
    profile_root[GRIPPER_JOINT] = merged_profile
    _write_yaml(PROFILE_PATH, profile_root)

    if bus_logger is not None:
        bus_logger.log(
            "gripper_profile_saved",
            action_detail="record_live_position",
            backend=backend_name,
            current_position=current_position,
            joint=GRIPPER_JOINT,
            log_path=SERVO_LOG_PATH,
            profile_path=PROFILE_PATH,
            record_field=record_field,
            servo_id=GRIPPER_SERVO_ID,
            timestamp_utc=_now_utc(),
        )

    print("Saved {}={} to {}.".format(record_field, current_position, PROFILE_PATH))


def main():
    parser = _build_parser()
    args = parser.parse_args()

    record_field = _selected_record_field(args)
    config_path = args.config or os.path.join(ROOT, "configs", "robot.yaml")
    config = load_robot_config(config_path)

    servo_entry = _load_servo_map(SERVO_MAP_PATH)
    limits_entry = _load_joint_limits(JOINT_LIMITS_PATH)
    directions_entry = _load_joint_directions(JOINT_DIRECTIONS_PATH)

    current_position = None
    bus = None
    if args.backend:
        bus = build_servo_bus(config, config_path, dry_run=False, backend_name=args.backend)
        try:
            current_position = bus.read_position(GRIPPER_SERVO_ID)
        except Exception:
            bus.close()
            raise

    min_value = _to_int(limits_entry.get("provisional_min"), "joint_limits.yaml gripper provisional_min")
    max_value = _to_int(limits_entry.get("provisional_max"), "joint_limits.yaml gripper provisional_max")
    in_range = None if current_position is None else min_value <= current_position <= max_value

    print("Gripper profile check:")
    print("  joint: {}".format(GRIPPER_JOINT))
    print("  servo_id: {}".format(servo_entry.get("id")))
    print("  direction_sign: {}".format(directions_entry.get("sign")))
    print("  limits: [{}, {}]".format(min_value, max_value))
    if current_position is None:
        print("  live_position: unavailable")
    else:
        print("  live_position: {}".format(current_position))
        print("  live_position_in_range: {}".format("yes" if in_range else "no"))

    profile_root = _read_yaml(PROFILE_PATH)
    if record_field is None:
        print("No profile changes made.")
        if bus is not None:
            bus.close()
        if current_position is not None and not in_range:
            raise ServoBusError(
                "Refusing to save: current gripper position {} is outside [{}, {}].".format(
                    current_position, min_value, max_value
                )
            )
        return 0

    if current_position is None:
        if bus is not None:
            bus.close()
        raise ServoBusError(
            "Recording {} requires a backend that can read the live gripper position.".format(
                record_field
            )
        )

    if not in_range:
        if bus is not None:
            bus.close()
        raise ServoBusError(
            "Refusing to save: current gripper position {} is outside [{}, {}].".format(
                current_position, min_value, max_value
            )
        )

    _save_profile(
        profile_root,
        current_position,
        record_field,
        bus.logger if bus is not None else None,
        args.backend,
        servo_entry,
        limits_entry,
        directions_entry,
    )
    if bus is not None:
        bus.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ServoBusError, safety.SafetyError) as exc:
        sys.stderr.write("ERROR: {}\n".format(exc))
        sys.exit(1)
