#!/usr/bin/env python3
"""Manual SO-101 joint direction and software-limit recorder.

This tool is read-based. It records the current observed servo position as
manual calibration metadata and never writes goal positions or movement targets.
Only optional single-joint torque enable/disable is exposed, and only behind
--real plus an exact typed confirmation.
"""

from __future__ import print_function

import argparse
import datetime
import json
import os
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

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

DEFAULT_SERVO_IDS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

DEFAULT_ALIASES = {
    "base_yaw": "shoulder_pan",
    "shoulder_pitch": "shoulder_lift",
    "elbow_pitch": "elbow_flex",
    "wrist_pitch": "wrist_flex",
}

DEFAULT_MARGIN_TICKS = {
    "shoulder_pan": 20,
    "shoulder_lift": 20,
    "elbow_flex": 20,
    "wrist_flex": 20,
    "wrist_roll": 20,
    "gripper": 10,
}

MIN_LIMIT_SPAN_TICKS = 50


def _now_utc():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def _require_yaml():
    if yaml is None:
        raise ServoBusError("PyYAML is required for calibration files.")


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



def _yaml_scalar(value):
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    return json.dumps(str(value))


def _write_lines(path, lines):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def _write_limits_yaml(path, data):
    limits = data.get("limits") or {}
    lines = ["limits:"]
    for joint_name in CANONICAL_JOINTS:
        entry = limits.get(joint_name) or {}
        lines.append("  {}:".format(joint_name))
        for key in ("provisional_min", "provisional_max", "neutral", "margin_ticks", "calibrated", "notes"):
            lines.append("    {}: {}".format(key, _yaml_scalar(entry.get(key))))
    _write_lines(path, lines)


def _write_directions_yaml(path, data):
    directions = data.get("directions") or {}
    lines = ["directions:"]
    for joint_name in CANONICAL_JOINTS:
        entry = directions.get(joint_name) or {}
        lines.append("  {}:".format(joint_name))
        for key in ("sign", "positive_description", "notes"):
            lines.append("    {}: {}".format(key, _yaml_scalar(entry.get(key))))
    _write_lines(path, lines)

def _default_servo_map():
    return {
        "joints": {
            joint_name: {"id": DEFAULT_SERVO_IDS[joint_name], "calibrated": False}
            for joint_name in CANONICAL_JOINTS
        },
        "aliases": dict(DEFAULT_ALIASES),
    }


def _load_servo_map(path):
    data = _read_yaml(path)
    result = _default_servo_map()
    if not data:
        return result

    joints = data.get("joints") or {}
    if not isinstance(joints, dict):
        raise ServoBusError("servo_map.yaml 'joints' must be a mapping.")

    for joint_name in CANONICAL_JOINTS:
        entry = joints.get(joint_name)
        if entry is None:
            continue
        if not isinstance(entry, dict):
            raise ServoBusError("servo_map entry for {} must be a mapping.".format(joint_name))
        servo_id = entry.get("id", entry.get("servo_id"))
        if servo_id is None:
            raise ServoBusError("servo_map entry for {} has no servo ID.".format(joint_name))
        result["joints"][joint_name] = {
            "id": safety.validate_servo_id(servo_id),
            "calibrated": bool(entry.get("calibrated", False)),
        }

    aliases = data.get("aliases") or {}
    if aliases:
        if not isinstance(aliases, dict):
            raise ServoBusError("servo_map.yaml 'aliases' must be a mapping.")
        clean_aliases = {}
        for alias, canonical in aliases.items():
            if canonical not in CANONICAL_JOINTS:
                raise ServoBusError("Alias {} points to unknown joint {}.".format(alias, canonical))
            clean_aliases[str(alias)] = canonical
        result["aliases"] = clean_aliases
    return result


def _mapped_joints(servo_map):
    result = []
    joints = servo_map.get("joints") or {}
    for joint_name in CANONICAL_JOINTS:
        entry = joints.get(joint_name)
        if not isinstance(entry, dict):
            raise ServoBusError("Joint {} is not mapped in servo_map.yaml.".format(joint_name))
        servo_id = entry.get("id")
        if servo_id is None:
            raise ServoBusError("Joint {} has no mapped servo ID.".format(joint_name))
        result.append((joint_name, safety.validate_servo_id(servo_id)))
    return result


def _resolve_joint(servo_map, requested):
    if requested is None:
        raise ServoBusError("A mapped --joint is required for this operation.")
    if requested in CANONICAL_JOINTS:
        return requested
    aliases = servo_map.get("aliases") or {}
    if requested in aliases:
        return aliases[requested]
    raise ServoBusError("Unknown joint {!r}. Refusing to guess.".format(requested))


def _default_limits_data():
    limits = {}
    for joint_name in CANONICAL_JOINTS:
        limits[joint_name] = {
            "provisional_min": None,
            "provisional_max": None,
            "neutral": None,
            "margin_ticks": DEFAULT_MARGIN_TICKS[joint_name],
            "calibrated": False,
            "notes": "",
        }
    return {"limits": limits}


def _load_limits(path):
    data = _read_yaml(path)
    result = _default_limits_data()
    if not data:
        return result

    existing_limits = data.get("limits") or data.get("joints") or {}
    if not isinstance(existing_limits, dict):
        raise ServoBusError("joint_limits.yaml must contain a 'limits' mapping.")

    for joint_name in CANONICAL_JOINTS:
        entry = existing_limits.get(joint_name)
        if entry is None:
            continue
        if not isinstance(entry, dict):
            raise ServoBusError("Limit entry for {} must be a mapping.".format(joint_name))
        merged = dict(result["limits"][joint_name])
        for key in ("provisional_min", "provisional_max", "neutral", "margin_ticks", "calibrated", "notes"):
            if key in entry:
                merged[key] = entry[key]
        result["limits"][joint_name] = merged
    return result


def _default_directions_data():
    directions = {}
    for joint_name in CANONICAL_JOINTS:
        directions[joint_name] = {
            "sign": None,
            "positive_description": None,
            "notes": "",
        }
    return {"directions": directions}


def _load_directions(path):
    data = _read_yaml(path)
    result = _default_directions_data()
    if not data:
        return result

    existing = data.get("directions") or data.get("joints") or {}
    if not isinstance(existing, dict):
        raise ServoBusError("joint_directions.yaml must contain a 'directions' mapping.")

    for joint_name in CANONICAL_JOINTS:
        entry = existing.get(joint_name)
        if entry is None:
            continue
        if not isinstance(entry, dict):
            raise ServoBusError("Direction entry for {} must be a mapping.".format(joint_name))
        merged = dict(result["directions"][joint_name])
        for key in ("sign", "positive_description", "notes"):
            if key in entry:
                merged[key] = entry[key]
        result["directions"][joint_name] = merged
    return result


def _read_positions(bus, joints):
    positions = {}
    for joint_name, servo_id in joints:
        positions[joint_name] = bus.read_position(servo_id)
    return positions


def _save_snapshot(path, backend_name, dry_run, joints, positions, bus):
    data = {
        "snapshot_utc": _now_utc(),
        "backend": backend_name,
        "dry_run": bool(dry_run),
        "joints": {},
    }
    for joint_name, servo_id in joints:
        data["joints"][joint_name] = {
            "id": servo_id,
            "current_position": positions.get(joint_name),
        }
    _write_yaml(path, data)
    bus.logger.log("servo_snapshot_save", path=path, backend=backend_name, dry_run=bool(dry_run))


def _status_text(entry):
    bits = []
    if entry.get("provisional_min") is not None:
        bits.append("min")
    if entry.get("provisional_max") is not None:
        bits.append("max")
    if entry.get("neutral") is not None:
        bits.append("neutral")
    if entry.get("calibrated"):
        bits.append("calibrated")
    return ",".join(bits) if bits else "none"


def _print_table(joints, positions, limits_data, directions_data):
    print("{:<16} {:>8} {:>18} {:>9} {:>9} {:>9} {:>8} {:>11} {:>9}".format(
        "joint", "servo_id", "current_position", "min", "max", "neutral", "margin", "limit_state", "dir_sign"
    ))
    print("{:<16} {:>8} {:>18} {:>9} {:>9} {:>9} {:>8} {:>11} {:>9}".format(
        "-" * 16, "-" * 8, "-" * 18, "-" * 9, "-" * 9, "-" * 9, "-" * 8, "-" * 11, "-" * 9
    ))
    limits = limits_data.get("limits") or {}
    directions = directions_data.get("directions") or {}
    for joint_name, servo_id in joints:
        position = positions.get(joint_name)
        limit_entry = limits.get(joint_name) or {}
        direction_entry = directions.get(joint_name) or {}
        print("{:<16} {:>8} {:>18} {:>9} {:>9} {:>9} {:>8} {:>11} {:>9}".format(
            joint_name,
            servo_id,
            "unavailable" if position is None else position,
            "-" if limit_entry.get("provisional_min") is None else limit_entry.get("provisional_min"),
            "-" if limit_entry.get("provisional_max") is None else limit_entry.get("provisional_max"),
            "-" if limit_entry.get("neutral") is None else limit_entry.get("neutral"),
            limit_entry.get("margin_ticks"),
            _status_text(limit_entry),
            "-" if direction_entry.get("sign") is None else direction_entry.get("sign"),
        ))


def _as_optional_int(value, field_name):
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        raise ServoBusError("{} must be an integer or null.".format(field_name))


def _validate_limit_entry(joint_name, entry):
    min_value = _as_optional_int(entry.get("provisional_min"), "provisional_min")
    max_value = _as_optional_int(entry.get("provisional_max"), "provisional_max")
    neutral = _as_optional_int(entry.get("neutral"), "neutral")
    margin = _as_optional_int(entry.get("margin_ticks"), "margin_ticks")

    if margin is None or margin < 0:
        raise ServoBusError("margin_ticks for {} must be a non-negative integer.".format(joint_name))
    if min_value is not None and max_value is not None:
        if min_value >= max_value:
            raise ServoBusError("Refusing to save {}: provisional_min must be < provisional_max.".format(joint_name))
        span = max_value - min_value
        if span < MIN_LIMIT_SPAN_TICKS:
            raise ServoBusError(
                "Refusing to save {}: limit span {} ticks is suspiciously tiny (< {}).".format(
                    joint_name, span, MIN_LIMIT_SPAN_TICKS
                )
            )
    if neutral is not None and min_value is not None and max_value is not None:
        if neutral < min_value or neutral > max_value:
            raise ServoBusError("Refusing to save {}: neutral is outside provisional min/max.".format(joint_name))


def _record_limit(args, joint_name, servo_id, current_position, limits_data, bus):
    if current_position is None:
        raise ServoBusError("Cannot save a limit for {}: current position could not be read.".format(joint_name))
    current_position = int(current_position)
    entry = dict(limits_data["limits"][joint_name])
    changed_fields = []

    if args.record_min:
        entry["provisional_min"] = current_position
        changed_fields.append("provisional_min")
    if args.record_max:
        entry["provisional_max"] = current_position
        changed_fields.append("provisional_max")
    if args.record_neutral:
        entry["neutral"] = current_position
        changed_fields.append("neutral")
    if args.margin_ticks is not None:
        if args.margin_ticks < 0:
            raise ServoBusError("--margin-ticks must be non-negative.")
        entry["margin_ticks"] = int(args.margin_ticks)
        changed_fields.append("margin_ticks")
    if args.notes is not None:
        entry["notes"] = args.notes
        changed_fields.append("notes")

    _validate_limit_entry(joint_name, entry)
    limits_data["limits"][joint_name] = entry
    bus.logger.log(
        "joint_limit_save",
        joint=joint_name,
        servo_id=servo_id,
        current_position=current_position,
        changed_fields=changed_fields,
        entry=entry,
    )


def _prompt_if_missing(prompt, current_value):
    suffix = "" if current_value is None else str(current_value)
    value = input("{} [{}]: ".format(prompt, suffix)).strip()
    if value == "":
        return current_value
    return value


def _record_direction(args, joint_name, servo_id, directions_data, bus):
    entry = dict(directions_data["directions"][joint_name])

    sign = args.sign
    description = args.positive_description
    notes = args.notes

    if sign is None:
        sign_text = _prompt_if_missing("sign 1 or -1", entry.get("sign"))
        if sign_text is not None:
            try:
                sign = int(sign_text)
            except (TypeError, ValueError):
                raise ServoBusError("Direction sign must be 1 or -1.")
    if sign not in (1, -1):
        raise ServoBusError("Direction sign must be 1 or -1.")

    if description is None:
        description = _prompt_if_missing("positive_description", entry.get("positive_description"))
    if description is None or str(description).strip() == "":
        raise ServoBusError("positive_description is required when recording direction.")

    if notes is None:
        notes = _prompt_if_missing("notes", entry.get("notes", ""))
    if notes is None:
        notes = ""

    entry["sign"] = int(sign)
    entry["positive_description"] = str(description)
    entry["notes"] = str(notes)
    directions_data["directions"][joint_name] = entry
    bus.logger.log(
        "joint_direction_save",
        joint=joint_name,
        servo_id=servo_id,
        entry=entry,
    )


def _requested_record_count(args):
    flags = [args.record_min, args.record_max, args.record_neutral, args.record_direction]
    return sum(1 for flag in flags if flag)


def _torque_request(args):
    if args.torque_disable and args.torque_enable:
        raise ServoBusError("Specify only one torque command at a time.")
    if args.torque_disable:
        return False
    if args.torque_enable:
        return True
    return None


def _require_torque_confirmation(enabled, joint_name):
    expected = "TORQUE ENABLE {}".format(joint_name) if enabled else "TORQUE DISABLE {}".format(joint_name)
    if not enabled:
        print("Physically support the arm before disabling torque for {}.".format(joint_name))
    typed = input("Type {} to continue: ".format(expected)).strip()
    if typed != expected:
        raise ServoBusError("Torque command refused: typed confirmation did not match exactly.")


def _apply_single_joint_torque(args, bus, joint_name, servo_id, enabled):
    if not args.real:
        bus.logger.log(
            "servo_torque_request",
            enabled=bool(enabled),
            joint=joint_name,
            servo_id=servo_id,
            status="refused",
            reason="missing_real",
        )
        print("Torque writes require --real and exact typed confirmation. No torque write was attempted.")
        return

    _require_torque_confirmation(enabled, joint_name)
    bus.logger.log(
        "servo_torque_request",
        enabled=bool(enabled),
        joint=joint_name,
        servo_id=servo_id,
        status="attempting",
    )
    bus.torque_enable(servo_id, enabled)
    print("Torque {} for {} (servo {}).".format("enabled" if enabled else "disabled", joint_name, servo_id))


def _determine_dry_run(args, config):
    if args.real:
        return False
    if args.dry_run:
        return True
    if args.backend == "feetech":
        return False
    return bool((config.get("servo_bus") or {}).get("dry_run_default", True))


def build_parser():
    parser = argparse.ArgumentParser(
        description="Record manual SO-101 joint limits and direction signs from present-position reads."
    )
    parser.add_argument("--config", default=os.path.join(ROOT, "configs", "robot.yaml"))
    parser.add_argument("--backend", choices=("mock", "feetech"), default=None)
    parser.add_argument("--dry-run", action="store_true", help="Force mock/dry-run access. This is the default without --backend feetech.")
    parser.add_argument("--real", action="store_true", help="Allow only a selected single-joint torque write after exact confirmation.")
    parser.add_argument("--servo-map", default=os.path.join(ROOT, "data", "calibration", "robot", "servo_map.yaml"))
    parser.add_argument("--limits-path", default=os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml"))
    parser.add_argument("--directions-path", default=os.path.join(ROOT, "data", "calibration", "robot", "joint_directions.yaml"))
    parser.add_argument("--snapshot-path", default=os.path.join(ROOT, "data", "calibration", "robot", "servo_snapshot.yaml"))
    parser.add_argument("--joint", default=None, help="Canonical joint name or configured alias.")
    parser.add_argument("--record-min", action="store_true", help="Record current position as provisional_min.")
    parser.add_argument("--record-max", action="store_true", help="Record current position as provisional_max.")
    parser.add_argument("--record-neutral", action="store_true", help="Record current position as neutral/reference.")
    parser.add_argument("--record-direction", action="store_true", help="Record sign, positive_description, and notes.")
    parser.add_argument("--sign", type=int, choices=(1, -1), default=None, help="Direction sign for --record-direction.")
    parser.add_argument("--positive-description", default=None, help="Text describing the positive joint direction.")
    parser.add_argument("--notes", default=None, help="Notes to save with a limit or direction record.")
    parser.add_argument("--margin-ticks", type=int, default=None, help="Explicitly overwrite margin_ticks; never auto-applied to limits.")
    parser.add_argument("--torque-disable", action="store_true", help="Disable torque for --joint only; requires --real and confirmation.")
    parser.add_argument("--torque-enable", action="store_true", help="Enable torque for --joint only; requires --real and confirmation.")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    try:
        record_count = _requested_record_count(args)
        torque_enabled = _torque_request(args)
        if record_count > 1:
            raise ServoBusError("Specify only one record action at a time.")
        if torque_enabled is not None and record_count:
            raise ServoBusError("Do not combine torque commands with recording commands.")
        if (record_count or torque_enabled is not None or args.margin_ticks is not None) and args.joint is None:
            raise ServoBusError("A mapped --joint is required for recording, margin, or torque operations.")

        config = load_robot_config(args.config)
        servo_map = _load_servo_map(args.servo_map)
        joints = _mapped_joints(servo_map)
        safety.validate_servo_ids([servo_id for _, servo_id in joints])
        joint_name = _resolve_joint(servo_map, args.joint) if args.joint is not None else None
        joint_servo_id = None
        if joint_name is not None:
            for mapped_name, servo_id in joints:
                if mapped_name == joint_name:
                    joint_servo_id = servo_id
                    break
        if joint_name is not None and joint_servo_id is None:
            raise ServoBusError("Joint {} has no mapped servo ID.".format(joint_name))

        limits_data = _load_limits(args.limits_path)
        directions_data = _load_directions(args.directions_path)
        dry_run = _determine_dry_run(args, config)

        bus = build_servo_bus(
            config=config,
            config_path=args.config,
            dry_run=dry_run,
            backend_name=args.backend,
            mock_ids=[servo_id for _, servo_id in joints],
        )
        try:
            positions = _read_positions(bus, joints)
            _print_table(joints, positions, limits_data, directions_data)

            if not bus.dry_run:
                _save_snapshot(args.snapshot_path, bus.backend.name, bus.dry_run, joints, positions, bus)
                print("Snapshot saved: {}".format(args.snapshot_path))

            if record_count or args.margin_ticks is not None:
                _record_limit(args, joint_name, joint_servo_id, positions.get(joint_name), limits_data, bus)
                _write_limits_yaml(args.limits_path, limits_data)
                print("Joint limits saved: {}".format(args.limits_path))

            if args.record_direction:
                _record_direction(args, joint_name, joint_servo_id, directions_data, bus)
                _write_directions_yaml(args.directions_path, directions_data)
                print("Joint directions saved: {}".format(args.directions_path))
            else:
                _write_directions_yaml(args.directions_path, directions_data)

            if not (record_count or args.margin_ticks is not None):
                _write_limits_yaml(args.limits_path, limits_data)

            if torque_enabled is not None:
                _apply_single_joint_torque(args, bus, joint_name, joint_servo_id, torque_enabled)
        finally:
            bus.close()
    except (ServoBusError, safety.SafetyError) as exc:
        parser.exit(2, "error: {}\n".format(exc))


if __name__ == "__main__":
    main()
