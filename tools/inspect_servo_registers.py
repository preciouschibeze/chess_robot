#!/usr/bin/env python3
"""Read-only Feetech register inspection for servo safety audits.

This tool never writes registers and never commands movement. Use --live for
explicit read-only access to the configured Feetech bus.
"""

import argparse
import os
import sys
from typing import Any, Dict, List, Optional

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


REGISTER_SPECS = [
    {"key": "present_position", "label": "present position", "address": 56, "length": 2},
    {"key": "goal_position", "label": "goal position", "address": 42, "length": 2},
    {"key": "operating_mode", "label": "operating mode", "address": 33, "length": 1},
    {"key": "acceleration", "label": "acceleration", "address": 41, "length": 1},
    {"key": "moving_speed", "label": "speed/moving-speed", "address": 46, "length": 2},
    {"key": "torque_enable", "label": "torque state", "address": 40, "length": 1},
    {"key": "present_voltage", "label": "voltage", "address": 62, "length": 1},
    {"key": "present_temperature", "label": "temperature", "address": 63, "length": 1},
    {"key": "present_load", "label": "load", "address": 60, "length": 2},
    {"key": "present_speed", "label": "present speed", "address": 58, "length": 2},
    {"key": "min_angle_limit", "label": "min angle limit", "address": 9, "length": 2},
    {"key": "max_angle_limit", "label": "max angle limit", "address": 11, "length": 2},
    {"key": "lock", "label": "lock/eeprom state", "address": 55, "length": 1},
    {"key": "moving", "label": "moving flag", "address": 66, "length": 1},
    {"key": "hardware_error_status", "label": "hardware error status", "address": 65, "length": 1},
]


def build_parser():
    parser = argparse.ArgumentParser(
        description="Inspect read-only Feetech servo registers. This tool never writes registers."
    )
    parser.add_argument("--config", default=os.path.join(ROOT, "configs", "robot.yaml"),
                        help="Path to robot YAML config. Default: configs/robot.yaml")
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--id", dest="servo_id", type=int, help="Servo ID to inspect, e.g. 6")
    target.add_argument("--all", action="store_true", help="Inspect all configured joint servo IDs")
    parser.add_argument("--live", action="store_true",
                        help="Use configured Feetech backend for read-only live hardware inspection.")
    parser.add_argument("--yes", action="store_true",
                        help="Confirm intentional read-only real servo bus access when not using --live.")
    return parser


def _build_joint_map(config):
    joint_map = {}
    joints = config.get("joints") or {}
    if not isinstance(joints, dict):
        return joint_map
    for joint_name, joint_config in joints.items():
        if isinstance(joint_config, dict) and joint_config.get("servo_id") is not None:
            try:
                joint_map[int(joint_config.get("servo_id"))] = joint_name
            except (TypeError, ValueError):
                continue
    return joint_map


def _backend_details(config, backend_name, dry_run):
    servo_config = config.get("servo_bus") or {}
    effective_backend = backend_name or servo_config.get("backend") or "mock"
    if dry_run:
        effective_backend = "mock"
    details = {
        "backend": effective_backend,
        "transport": "n/a",
        "serial_port": "n/a",
        "baudrate": "n/a",
        "config_default_backend": servo_config.get("backend") or "mock",
        "config_dry_run_default": bool(servo_config.get("dry_run_default", True)),
    }
    if effective_backend == "feetech":
        feetech_config = servo_config.get("feetech") or {}
        details["transport"] = feetech_config.get("transport") or "raw_serial"
        details["serial_port"] = feetech_config.get("port") or "n/a"
        details["baudrate"] = feetech_config.get("baudrate") or "n/a"
    return details


def _decode_mode(value):
    names = {
        0: "position/servo",
        1: "velocity/motor",
        2: "pwm",
        3: "step/multi-turn",
    }
    if value is None:
        return None
    return names.get(int(value), "unknown")


def _format_value(key, value):
    if value is None:
        return "unavailable"
    if key == "operating_mode":
        return "{} ({})".format(value, _decode_mode(value))
    if key == "torque_enable":
        return "{} ({})".format(value, "enabled" if int(value) else "disabled")
    if key == "present_voltage":
        return "{} ({:.1f} V)".format(value, float(value) / 10.0)
    if key == "moving":
        return "{} ({})".format(value, "moving" if int(value) else "stopped")
    if key == "lock":
        return "{} ({})".format(value, "locked" if int(value) else "unlocked")
    return str(value)


def _read_register(bus, servo_id, spec):
    try:
        value = bus.read_register(servo_id, spec["address"], spec["length"])
    except Exception as exc:
        return None, str(exc)
    if value is None:
        return None, "no valid status response or register unreadable"
    return value, None


def _inspect_servo(bus, servo_id, joint_name):
    result = {
        "servo_id": servo_id,
        "joint": joint_name,
        "ping_ok": None,
        "ping_failure_reason": None,
        "registers": {},
    }
    try:
        result["ping_ok"] = bus.ping(servo_id)
        if not result["ping_ok"]:
            result["ping_failure_reason"] = "servo did not respond to model-number read"
    except Exception as exc:
        result["ping_ok"] = False
        result["ping_failure_reason"] = str(exc)

    for spec in REGISTER_SPECS:
        value, failure_reason = _read_register(bus, servo_id, spec)
        result["registers"][spec["key"]] = {
            "label": spec["label"],
            "address": spec["address"],
            "length": spec["length"],
            "value": value,
            "failure_reason": failure_reason,
        }
    return result


def _print_header(details):
    print("backend: {}".format(details.get("backend")))
    print("transport: {}".format(details.get("transport")))
    print("serial port: {}".format(details.get("serial_port")))
    print("baudrate: {}".format(details.get("baudrate")))
    print("config default backend: {}".format(details.get("config_default_backend")))
    print("config dry_run_default: {}".format(details.get("config_dry_run_default")))


def _print_result(result):
    print("")
    print("servo ID: {}".format(result.get("servo_id")))
    print("joint name: {}".format(result.get("joint")))
    print("ping: {}".format("ok" if result.get("ping_ok") else "failed"))
    if result.get("ping_failure_reason"):
        print("ping failure reason: {}".format(result.get("ping_failure_reason")))
    registers = result.get("registers") or {}
    for spec in REGISTER_SPECS:
        entry = registers.get(spec["key"]) or {}
        failure_reason = entry.get("failure_reason")
        value_text = _format_value(spec["key"], entry.get("value"))
        suffix = ""
        if failure_reason:
            suffix = " failure_reason={}".format(failure_reason)
        print("{}: {} (addr={}, len={}){}".format(
            spec["label"], value_text, spec["address"], spec["length"], suffix
        ))


def main():
    parser = build_parser()
    args = parser.parse_args()
    config = load_robot_config(args.config)

    dry_run = not bool(args.live)
    backend_name = "feetech" if args.live else None
    safety.require_read_only_hardware_confirmation(dry_run, bool(args.live or args.yes))
    details = _backend_details(config, backend_name, dry_run)

    joint_map = _build_joint_map(config)
    if args.all:
        servo_ids = configured_joint_servo_ids(config)
    else:
        servo_ids = [safety.validate_servo_id(args.servo_id)]
    servo_ids = safety.validate_servo_ids(servo_ids)
    if not servo_ids:
        parser.error("No servo IDs are configured.")

    try:
        bus = build_servo_bus(
            config=config,
            config_path=args.config,
            dry_run=dry_run,
            backend_name=backend_name,
        )
    except (BackendUnavailable, ServoBusError, OSError, ValueError) as exc:
        print("ERROR: Could not open servo backend.")
        _print_header(details)
        print("reason: {}".format(exc))
        raise SystemExit(1)

    try:
        _print_header(details)
        for servo_id in servo_ids:
            result = _inspect_servo(bus, servo_id, joint_map.get(servo_id, "unmapped"))
            _print_result(result)
        print("")
        print("Log: {}".format(bus.logger.path))
    finally:
        bus.close()


if __name__ == "__main__":
    main()
