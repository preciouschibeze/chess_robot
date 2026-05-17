#!/usr/bin/env python3
"""SO-101 servo mapping, snapshots, direction notes, and torque gates.

This tool reads present positions and records calibration metadata. It never
moves servos, changes IDs, writes goal positions, or infers joint directions.
"""

from __future__ import print_function

import argparse
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

DEFAULT_SERVO_MAP = {
    "joints": {
        "shoulder_pan": {"id": 1, "calibrated": False},
        "shoulder_lift": {"id": 2, "calibrated": False},
        "elbow_flex": {"id": 3, "calibrated": False},
        "wrist_flex": {"id": 4, "calibrated": False},
        "wrist_roll": {"id": 5, "calibrated": False},
        "gripper": {"id": 6, "calibrated": False},
    },
    "aliases": {
        "base_yaw": "shoulder_pan",
        "shoulder_pitch": "shoulder_lift",
        "elbow_pitch": "elbow_flex",
        "wrist_pitch": "wrist_flex",
    },
}


def _now_utc() -> str:
    import datetime

    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def _require_yaml() -> None:
    if yaml is None:
        raise ServoBusError("PyYAML is required for servo calibration files.")


def _read_yaml(path: str) -> Dict[str, Any]:
    _require_yaml()
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ServoBusError("{} must contain a YAML mapping.".format(path))
    return data


def _write_yaml(path: str, data: Dict[str, Any]) -> None:
    _require_yaml()
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, default_flow_style=False)


def _default_map_copy() -> Dict[str, Any]:
    return {
        "joints": {name: dict(values) for name, values in DEFAULT_SERVO_MAP["joints"].items()},
        "aliases": dict(DEFAULT_SERVO_MAP["aliases"]),
    }


def load_servo_map(path: str) -> Dict[str, Any]:
    data = _read_yaml(path)
    if not data:
        return _default_map_copy()

    result = _default_map_copy()
    joints = data.get("joints") or {}
    if not isinstance(joints, dict):
        raise ServoBusError("servo_map.yaml 'joints' must be a mapping.")

    for joint_name in CANONICAL_JOINTS:
        if joint_name not in joints:
            continue
        entry = joints[joint_name] or {}
        if not isinstance(entry, dict):
            raise ServoBusError("servo_map entry for {} must be a mapping.".format(joint_name))
        servo_id = entry.get("id", entry.get("servo_id", result["joints"][joint_name]["id"]))
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


def save_servo_map(path: str, servo_map: Dict[str, Any], bus) -> None:
    _write_yaml(path, servo_map)
    bus.logger.log("servo_map_save", path=path, joints=list(servo_map.get("joints", {}).keys()))


def mapped_joints(servo_map: Dict[str, Any]) -> List[Tuple[str, Dict[str, Any]]]:
    joints = servo_map.get("joints") or {}
    result = []
    for joint_name in CANONICAL_JOINTS:
        entry = joints.get(joint_name)
        if not isinstance(entry, dict):
            raise ServoBusError("Missing canonical joint {} in servo map.".format(joint_name))
        result.append((joint_name, entry))
    return result


def resolve_joint_name(servo_map: Dict[str, Any], requested: str) -> str:
    if requested in CANONICAL_JOINTS:
        return requested
    aliases = servo_map.get("aliases") or {}
    if requested in aliases:
        return aliases[requested]
    raise ServoBusError("Unknown mapped joint {!r}.".format(requested))


def read_positions(bus, joints: List[Tuple[str, Dict[str, Any]]]) -> Dict[str, Optional[int]]:
    positions = {}
    for joint_name, entry in joints:
        positions[joint_name] = bus.read_position(entry["id"])
    return positions


def print_position_table(joints: List[Tuple[str, Dict[str, Any]]], positions: Dict[str, Optional[int]]) -> None:
    print("{:<16} {:>8} {:>18} {:>12}".format("joint", "servo_id", "current_position", "calibrated"))
    print("{:<16} {:>8} {:>18} {:>12}".format("-" * 16, "-" * 8, "-" * 18, "-" * 12))
    for joint_name, entry in joints:
        position = positions.get(joint_name)
        position_text = "unavailable" if position is None else str(position)
        print("{:<16} {:>8} {:>18} {:>12}".format(
            joint_name,
            entry["id"],
            position_text,
            str(bool(entry.get("calibrated", False))).lower(),
        ))


def save_snapshot(path: str, backend: str, dry_run: bool,
                  joints: List[Tuple[str, Dict[str, Any]]],
                  positions: Dict[str, Optional[int]], bus) -> Dict[str, Any]:
    data = {
        "snapshot_utc": _now_utc(),
        "backend": backend,
        "dry_run": bool(dry_run),
        "joints": {},
    }
    for joint_name, entry in joints:
        data["joints"][joint_name] = {
            "id": entry["id"],
            "calibrated": bool(entry.get("calibrated", False)),
            "current_position": positions.get(joint_name),
        }
    _write_yaml(path, data)
    bus.logger.log("servo_snapshot_save", path=path, backend=backend, dry_run=bool(dry_run))
    return data


def load_directions(path: str) -> Dict[str, Any]:
    data = _read_yaml(path)
    if not data:
        data = {"joints": {}}
    if "joints" not in data or not isinstance(data.get("joints"), dict):
        raise ServoBusError("joint_directions.yaml must contain a 'joints' mapping.")
    return data


def _direction_entry(servo_id: int) -> Dict[str, Any]:
    return {
        "id": servo_id,
        "positive_description": None,
        "sign": None,
        "notes": "",
        "recorded_at_utc": None,
    }


def _prompt_direction(joint_name: str, servo_id: int, existing: Dict[str, Any]) -> Dict[str, Any]:
    print("\nRecording direction metadata for {} (servo {}). No servo movement will be commanded.".format(
        joint_name, servo_id
    ))
    current_desc = existing.get("positive_description")
    current_sign = existing.get("sign")
    current_notes = existing.get("notes", "")
    desc = input("positive_description [{}]: ".format("" if current_desc is None else current_desc)).strip()
    sign_text = input("sign 1 or -1 [{}]: ".format("" if current_sign is None else current_sign)).strip()
    notes = input("notes [{}]: ".format(current_notes)).strip()

    entry = dict(existing)
    entry["id"] = servo_id
    if desc:
        entry["positive_description"] = desc
    if sign_text:
        if sign_text not in ("1", "-1"):
            raise ServoBusError("Direction sign must be 1 or -1.")
        entry["sign"] = int(sign_text)
    if notes:
        entry["notes"] = notes
    entry["recorded_at_utc"] = _now_utc()
    return entry


def record_directions(path: str, args, servo_map: Dict[str, Any],
                      joints: List[Tuple[str, Dict[str, Any]]], bus) -> Dict[str, Any]:
    data = load_directions(path)
    existing_joints = data["joints"]

    for joint_name, entry in joints:
        if joint_name not in existing_joints or not isinstance(existing_joints.get(joint_name), dict):
            existing_joints[joint_name] = _direction_entry(entry["id"])
        else:
            existing_joints[joint_name]["id"] = entry["id"]

    if args.direction_joint:
        joint_name = resolve_joint_name(servo_map, args.direction_joint)
        entry = dict(existing_joints[joint_name])
        if args.positive_description is not None:
            entry["positive_description"] = args.positive_description
        if args.sign is not None:
            if args.sign not in (1, -1):
                raise ServoBusError("--sign must be 1 or -1.")
            entry["sign"] = args.sign
        if args.notes is not None:
            entry["notes"] = args.notes
        entry["recorded_at_utc"] = _now_utc()
        existing_joints[joint_name] = entry
    else:
        for joint_name, entry in joints:
            existing_joints[joint_name] = _prompt_direction(joint_name, entry["id"], existing_joints[joint_name])

    _write_yaml(path, data)
    bus.logger.log("servo_directions_save", path=path, joints=list(existing_joints.keys()))
    return data


def torque_request(args) -> Optional[Tuple[bool, Optional[str]]]:
    requests = []
    if args.torque_disable_all or args.torque_enable_all:
        raise ServoBusError("All-servo torque commands are disabled. Use one mapped joint at a time.")
    if args.torque_disable:
        requests.append((False, args.torque_disable))
    if args.torque_enable:
        requests.append((True, args.torque_enable))
    if len(requests) > 1:
        raise ServoBusError("Specify only one torque command at a time.")
    return requests[0] if requests else None


def require_torque_confirmation(enabled: bool, joint_name: Optional[str]) -> None:
    if joint_name is None:
        expected = "TORQUE ENABLE ALL" if enabled else "TORQUE DISABLE ALL"
        prompt = "Type {} to continue: ".format(expected)
    else:
        expected = "TORQUE ENABLE {}".format(joint_name) if enabled else "TORQUE DISABLE {}".format(joint_name)
        prompt = "Type {} to continue: ".format(expected)
    if not enabled:
        print("Physically support the arm before disabling torque.")
    typed = input(prompt).strip()
    if typed != expected:
        raise ServoBusError("Torque command refused: typed confirmation did not match exactly.")


def apply_torque(bus, servo_map: Dict[str, Any], joints: List[Tuple[str, Dict[str, Any]]],
                 enabled: bool, requested_joint: Optional[str], real: bool) -> None:
    if not real:
        bus.logger.log(
            "servo_torque_request",
            enabled=bool(enabled),
            joint=requested_joint,
            status="refused",
            reason="missing_real",
        )
        print("Torque writes require --real and exact typed confirmation. No torque write was attempted.")
        return

    if requested_joint is None:
        require_torque_confirmation(enabled, None)
        targets = [(joint_name, entry["id"]) for joint_name, entry in joints]
    else:
        joint_name = resolve_joint_name(servo_map, requested_joint)
        require_torque_confirmation(enabled, joint_name)
        entry = servo_map["joints"][joint_name]
        targets = [(joint_name, entry["id"])]

    for joint_name, servo_id in targets:
        bus.logger.log(
            "servo_torque_request",
            enabled=bool(enabled),
            joint=joint_name,
            servo_id=servo_id,
            status="attempting",
        )
        bus.torque_enable(servo_id, enabled)
        print("Torque {} for {} (servo {}).".format("enabled" if enabled else "disabled", joint_name, servo_id))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read SO-101 servo positions, save snapshots, record direction metadata, and gate torque writes."
    )
    parser.add_argument("--config", default=os.path.join(ROOT, "configs", "robot.yaml"),
                        help="Path to robot YAML config. Default: configs/robot.yaml")
    parser.add_argument("--backend", choices=("mock", "feetech"), default=None,
                        help="Backend override. --backend feetech performs read-only real position reads unless --dry-run is set.")
    parser.add_argument("--dry-run", dest="dry_run", action="store_true", default=None,
                        help="Use mock backend and avoid hardware access.")
    parser.add_argument("--no-dry-run", dest="dry_run", action="store_false",
                        help="Allow real read-only backend access for snapshot reads.")
    parser.add_argument("--real", action="store_true",
                        help="Allow a torque write only after exact typed confirmation.")
    parser.add_argument("--yes", action="store_true",
                        help="Accepted for read-only compatibility only. It never authorizes torque writes.")
    parser.add_argument("--servo-map", default=os.path.join(ROOT, "data", "calibration", "robot", "servo_map.yaml"))
    parser.add_argument("--snapshot-path", default=os.path.join(ROOT, "data", "calibration", "robot", "servo_snapshot.yaml"))
    parser.add_argument("--directions-path", default=os.path.join(ROOT, "data", "calibration", "robot", "joint_directions.yaml"))
    parser.add_argument("--record-directions", action="store_true",
                        help="Record positive_description, sign, and notes without moving servos.")
    parser.add_argument("--direction-joint", default=None,
                        help="Canonical joint or alias to update when using --record-directions non-interactively.")
    parser.add_argument("--positive-description", default=None)
    parser.add_argument("--sign", type=int, choices=(1, -1), default=None)
    parser.add_argument("--notes", default=None)
    parser.add_argument("--torque-disable-all", action="store_true", help="Disabled: all-servo torque commands are refused.")
    parser.add_argument("--torque-enable-all", action="store_true", help="Disabled: all-servo torque commands are refused.")
    parser.add_argument("--torque-disable", default=None, metavar="JOINT")
    parser.add_argument("--torque-enable", default=None, metavar="JOINT")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    try:
        config = load_robot_config(args.config)
        servo_map = load_servo_map(args.servo_map)
        joints = mapped_joints(servo_map)
        servo_ids = [entry["id"] for _, entry in joints]
        safety.validate_servo_ids(servo_ids)

        if args.real:
            dry_run = False
        elif args.dry_run is not None:
            dry_run = bool(args.dry_run)
        elif args.backend == "feetech":
            dry_run = False
        else:
            dry_run = bool((config.get("servo_bus") or {}).get("dry_run_default", True))

        bus = build_servo_bus(
            config=config,
            config_path=args.config,
            dry_run=dry_run,
            backend_name=args.backend,
            mock_ids=servo_ids,
        )
        try:
            save_servo_map(args.servo_map, servo_map, bus)
            positions = read_positions(bus, joints)
            print_position_table(joints, positions)
            snapshot = save_snapshot(args.snapshot_path, bus.backend.name, bus.dry_run, joints, positions, bus)
            print("Snapshot saved: {}".format(args.snapshot_path))

            if args.record_directions:
                record_directions(args.directions_path, args, servo_map, joints, bus)
                print("Direction metadata saved: {}".format(args.directions_path))

            request = torque_request(args)
            if request is not None:
                enabled, requested_joint = request
                apply_torque(bus, servo_map, joints, enabled, requested_joint, args.real)
        finally:
            bus.close()
    except (ServoBusError, safety.SafetyError) as exc:
        parser.exit(2, "error: {}\n".format(exc))


if __name__ == "__main__":
    main()
