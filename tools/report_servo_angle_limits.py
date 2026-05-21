#!/usr/bin/env python3
"""Read-only servo EEPROM/software angle-limit report.

This tool reads Feetech registers only. It never writes goal positions, torque,
operating mode, speed, acceleration, or EEPROM limits.
"""

import argparse
import os
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover - depends on runtime image
    yaml = None


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot import safety
from chess_robot.robot.servo_bus import (
    BackendUnavailable,
    ServoBusError,
    build_servo_bus,
    configured_joint_servo_ids,
    load_robot_config,
)


DEFAULT_JOINT_LIMITS_PATH = os.path.join("data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_GRIPPER_PROFILE_PATH = os.path.join("data", "calibration", "gripper", "gripper_profile.yaml")

REG_PRESENT_POSITION = (56, 2)
REG_GOAL_POSITION = (42, 2)
REG_OPERATING_MODE = (33, 1)
REG_TORQUE_ENABLE = (40, 1)
REG_MIN_ANGLE_LIMIT = (9, 2)
REG_MAX_ANGLE_LIMIT = (11, 2)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Read-only EEPROM/software angle-limit report for configured servos."
    )
    parser.add_argument(
        "--config",
        default=os.path.join(ROOT, "configs", "robot.yaml"),
        help="Path to robot YAML config. Default: configs/robot.yaml",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Use configured Feetech backend for read-only live hardware inspection.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Confirm intentional read-only real servo bus access when not using --live.",
    )
    parser.add_argument(
        "--goal-delta-warning",
        type=int,
        default=100,
        help="Warn when abs(goal-present) is greater than this many ticks. Default: 100.",
    )
    return parser


def _project_root(config_path):
    config_dir = os.path.dirname(os.path.abspath(config_path))
    if os.path.basename(config_dir) == "configs":
        return os.path.dirname(config_dir)
    return os.getcwd()


def _load_yaml_mapping(path):
    if yaml is None:
        raise ServoBusError("PyYAML is required to read calibration data.")
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ServoBusError("YAML file must contain a mapping: {}".format(path))
    return data


def _coerce_int(value):
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _resolve_limit_entry(entry):
    if not isinstance(entry, dict):
        return None
    minimum = None
    maximum = None
    for key in ("min", "provisional_min"):
        minimum = _coerce_int(entry.get(key))
        if minimum is not None:
            break
    for key in ("max", "provisional_max"):
        maximum = _coerce_int(entry.get(key))
        if maximum is not None:
            break
    if minimum is None or maximum is None:
        return None
    return {"min": minimum, "max": maximum}


def _load_software_limits(config_path):
    root = _project_root(config_path)
    joint_limits_path = os.path.join(root, DEFAULT_JOINT_LIMITS_PATH)
    gripper_profile_path = os.path.join(root, DEFAULT_GRIPPER_PROFILE_PATH)

    limits_data = _load_yaml_mapping(joint_limits_path)
    limits = limits_data.get("limits") or {}
    if not isinstance(limits, dict):
        limits = {}
    limits = dict(limits)

    gripper_profile = _load_yaml_mapping(gripper_profile_path).get("gripper") or {}
    if isinstance(gripper_profile, dict):
        profile_limits = gripper_profile.get("limits") or {}
        if isinstance(profile_limits, dict):
            merged = dict(limits.get("gripper") or {})
            if profile_limits.get("min") is not None:
                merged["min"] = profile_limits.get("min")
            if profile_limits.get("max") is not None:
                merged["max"] = profile_limits.get("max")
            if merged:
                limits["gripper"] = merged

    resolved = {}
    for joint, entry in limits.items():
        value = _resolve_limit_entry(entry)
        if value is not None:
            resolved[joint] = value
    return resolved


def _joint_maps(config):
    by_id = {}
    joints = config.get("joints") or {}
    if not isinstance(joints, dict):
        return by_id
    for joint_name, joint_config in joints.items():
        if isinstance(joint_config, dict) and joint_config.get("servo_id") is not None:
            try:
                by_id[int(joint_config.get("servo_id"))] = str(joint_name)
            except (TypeError, ValueError):
                continue
    return by_id


def _read_register(bus, servo_id, spec):
    address, length = spec
    try:
        return bus.read_register(servo_id, address, length), None
    except Exception as exc:
        return None, str(exc)


def _inside(value, minimum, maximum):
    if value is None or minimum is None or maximum is None:
        return None
    return minimum <= value <= maximum


def _overlaps(left_min, left_max, right_min, right_max):
    if None in (left_min, left_max, right_min, right_max):
        return None
    return max(left_min, right_min) <= min(left_max, right_max)


def _format_bool(value):
    if value is None:
        return "unknown"
    return "yes" if value else "NO"


def _format_range(limits):
    if not limits:
        return "unknown"
    return "{}..{}".format(limits.get("min"), limits.get("max"))


def _warnings(row, goal_delta_warning):
    warnings = []
    goal = row.get("goal_position")
    present = row.get("present_position")
    eeprom_min = row.get("eeprom_min")
    eeprom_max = row.get("eeprom_max")
    if goal is not None and eeprom_min is not None and eeprom_max is not None:
        if goal < eeprom_min or goal > eeprom_max:
            warnings.append("goal_outside_eeprom")
    if goal is not None and present is not None:
        delta = abs(int(goal) - int(present))
        if delta > int(goal_delta_warning):
            warnings.append("goal_present_delta={}".format(delta))
    if row.get("inside_eeprom") is False:
        warnings.append("present_outside_eeprom")
    if row.get("inside_software") is False:
        warnings.append("present_outside_software")
    if row.get("ranges_overlap") is False:
        warnings.append("software_eeprom_no_overlap")
    return ", ".join(warnings) if warnings else "-"


def _collect_rows(bus, servo_ids, joint_by_id, software_limits, goal_delta_warning):
    rows = []
    for servo_id in servo_ids:
        joint = joint_by_id.get(servo_id, "unmapped")
        present, present_error = _read_register(bus, servo_id, REG_PRESENT_POSITION)
        goal, goal_error = _read_register(bus, servo_id, REG_GOAL_POSITION)
        mode, mode_error = _read_register(bus, servo_id, REG_OPERATING_MODE)
        torque, torque_error = _read_register(bus, servo_id, REG_TORQUE_ENABLE)
        eeprom_min, min_error = _read_register(bus, servo_id, REG_MIN_ANGLE_LIMIT)
        eeprom_max, max_error = _read_register(bus, servo_id, REG_MAX_ANGLE_LIMIT)

        sw_limits = software_limits.get(joint)
        sw_min = sw_limits.get("min") if sw_limits else None
        sw_max = sw_limits.get("max") if sw_limits else None

        row = {
            "servo_id": servo_id,
            "joint": joint,
            "present_position": present,
            "goal_position": goal,
            "operating_mode": mode,
            "torque_state": torque,
            "eeprom_min": eeprom_min,
            "eeprom_max": eeprom_max,
            "software_min": sw_min,
            "software_max": sw_max,
            "inside_eeprom": _inside(present, eeprom_min, eeprom_max),
            "inside_software": _inside(present, sw_min, sw_max),
            "ranges_overlap": _overlaps(eeprom_min, eeprom_max, sw_min, sw_max),
            "read_errors": [error for error in (
                present_error, goal_error, mode_error, torque_error, min_error, max_error
            ) if error],
        }
        row["warnings"] = _warnings(row, goal_delta_warning)
        rows.append(row)
    return rows


def _stringify(value):
    if value is None:
        return "unknown"
    return str(value)


def _print_table(rows):
    columns = [
        ("id", "servo_id"),
        ("joint", "joint"),
        ("present", "present_position"),
        ("goal", "goal_position"),
        ("mode", "operating_mode"),
        ("torque", "torque_state"),
        ("eeprom_min", "eeprom_min"),
        ("eeprom_max", "eeprom_max"),
        ("sw_min", "software_min"),
        ("sw_max", "software_max"),
        ("present_in_eeprom", "inside_eeprom"),
        ("present_in_sw", "inside_software"),
        ("ranges_overlap", "ranges_overlap"),
        ("warnings", "warnings"),
    ]
    rendered = []
    for row in rows:
        rendered_row = []
        for _, key in columns:
            if key in ("inside_eeprom", "inside_software", "ranges_overlap"):
                rendered_row.append(_format_bool(row.get(key)))
            else:
                rendered_row.append(_stringify(row.get(key)))
        rendered.append(rendered_row)

    widths = []
    for index, column in enumerate(columns):
        header = column[0]
        width = len(header)
        for row in rendered:
            width = max(width, len(row[index]))
        widths.append(width)

    header_line = "  ".join(columns[index][0].ljust(widths[index]) for index in range(len(columns)))
    separator = "  ".join("-" * widths[index] for index in range(len(columns)))
    print(header_line)
    print(separator)
    for row in rendered:
        print("  ".join(row[index].ljust(widths[index]) for index in range(len(columns))))


def _print_comparison(rows):
    print("")
    print("Comparison summary:")
    for row in rows:
        eeprom = "{}..{}".format(_stringify(row.get("eeprom_min")), _stringify(row.get("eeprom_max")))
        software = "{}..{}".format(_stringify(row.get("software_min")), _stringify(row.get("software_max")))
        print(
            "ID {} {}: EEPROM={} software={} overlap={} present_in_eeprom={} present_in_software={} warnings={}".format(
                row.get("servo_id"),
                row.get("joint"),
                eeprom,
                software,
                _format_bool(row.get("ranges_overlap")),
                _format_bool(row.get("inside_eeprom")),
                _format_bool(row.get("inside_software")),
                row.get("warnings"),
            )
        )


def main():
    args = build_parser().parse_args()
    config = load_robot_config(args.config)

    dry_run = not bool(args.live)
    backend_name = "feetech" if args.live else None
    safety.require_read_only_hardware_confirmation(dry_run, bool(args.live or args.yes))

    servo_ids = configured_joint_servo_ids(config)
    if not servo_ids:
        raise SystemExit("No configured joint servo IDs found.")

    joint_by_id = _joint_maps(config)
    software_limits = _load_software_limits(args.config)

    try:
        bus = build_servo_bus(
            config=config,
            config_path=args.config,
            dry_run=dry_run,
            backend_name=backend_name,
        )
    except (BackendUnavailable, ServoBusError, OSError, ValueError) as exc:
        print("ERROR: Could not open servo backend: {}".format(exc))
        raise SystemExit(1)

    try:
        mode = "REAL READ-ONLY" if args.live else "DRY-RUN"
        print("{} servo angle-limit report using {} backend.".format(mode, bus.backend.name))
        print("No servo writes, torque writes, movement commands, or EEPROM writes are performed.")
        print("")
        rows = _collect_rows(
            bus=bus,
            servo_ids=servo_ids,
            joint_by_id=joint_by_id,
            software_limits=software_limits,
            goal_delta_warning=args.goal_delta_warning,
        )
        _print_table(rows)
        _print_comparison(rows)
        print("")
        print("Log: {}".format(bus.logger.path))
    finally:
        bus.close()


if __name__ == "__main__":
    main()
