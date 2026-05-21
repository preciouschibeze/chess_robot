#!/usr/bin/env python3
"""Read servo positions without commanding movement.

Dry-run/mock remains the default. Use --live for explicit read-only hardware
access through the configured Feetech backend.
"""

import argparse
import os
import sys
from typing import List, Optional

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot import safety
from chess_robot.robot.servo_bus import (
    BackendUnavailable,
    ServoBusError,
    build_servo_bus,
    configured_joint_servo_ids,
    configured_mock_ids,
    load_robot_config,
)


def _parse_id_list(value: Optional[str]) -> Optional[List[int]]:
    if value is None or value == "":
        return None
    return safety.validate_servo_ids(part.strip() for part in value.split(",") if part.strip())


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read current servo positions only. This tool never writes registers or moves servos."
    )
    parser.add_argument("--config", default=os.path.join(ROOT, "configs", "robot.yaml"),
                        help="Path to robot YAML config. Default: configs/robot.yaml")
    parser.add_argument("--backend", choices=("mock", "feetech"), default=None,
                        help="Backend override. Use feetech for explicit live read-only hardware access.")
    parser.add_argument("--live", action="store_true",
                        help="Use the configured Feetech backend for read-only live hardware position reads.")
    parser.add_argument("--dry-run", dest="dry_run", action="store_true", default=None,
                        help="Use mock backend and avoid hardware access. This is the default from config.")
    parser.add_argument("--no-dry-run", dest="dry_run", action="store_false",
                        help="Allow read-only real bus access when combined with --yes or implied by --live.")
    parser.add_argument("--yes", action="store_true",
                        help="Confirm intentional read-only real servo bus access for --no-dry-run.")
    parser.add_argument("--ids", default=None,
                        help="Comma-separated servo IDs to read, e.g. 1,2,3. Defaults to configured joint/mock IDs.")
    parser.add_argument("--mock-ids", default=None,
                        help="Comma-separated mock IDs available during dry-run.")
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


def _resolve_runtime_mode(args, config):
    servo_config = config.get("servo_bus") or {}
    live_requested = bool(args.live)
    backend_name = args.backend
    if live_requested and backend_name is None:
        backend_name = "feetech"

    if args.dry_run is None:
        if live_requested:
            dry_run = False
        else:
            dry_run = bool(servo_config.get("dry_run_default", True))
    else:
        dry_run = bool(args.dry_run)

    if live_requested and dry_run:
        raise safety.SafetyError("--live cannot be combined with --dry-run.")

    confirmed = bool(args.yes or live_requested)
    safety.require_read_only_hardware_confirmation(bool(dry_run), confirmed)
    return dry_run, backend_name


def _describe_backend(config, backend_name, dry_run):
    servo_config = config.get("servo_bus") or {}
    effective_backend = backend_name or servo_config.get("backend") or "mock"
    if dry_run and effective_backend != "feetech":
        effective_backend = "mock"

    details = {
        "backend": effective_backend,
        "serial_port": None,
        "baudrate": None,
    }
    if effective_backend == "feetech":
        feetech_config = servo_config.get("feetech") or {}
        details["serial_port"] = feetech_config.get("port")
        details["baudrate"] = feetech_config.get("baudrate")
    return details


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    config = load_robot_config(args.config)
    dry_run, backend_name = _resolve_runtime_mode(args, config)
    backend_details = _describe_backend(config, backend_name, dry_run)

    explicit_ids = _parse_id_list(args.ids)
    mock_ids = _parse_id_list(args.mock_ids)
    if explicit_ids is None:
        servo_ids = configured_joint_servo_ids(config) or configured_mock_ids(config) or (mock_ids or [])
    else:
        servo_ids = explicit_ids
    servo_ids = safety.validate_servo_ids(servo_ids)
    if not servo_ids:
        parser.error("No servo IDs provided or configured. Use --ids 1,2,3 for a read.")

    try:
        bus = build_servo_bus(
            config=config,
            config_path=args.config,
            dry_run=bool(dry_run),
            backend_name=backend_name,
            mock_ids=mock_ids,
        )
    except (BackendUnavailable, ServoBusError, OSError, ValueError) as exc:
        print("ERROR: Could not open servo backend.")
        print("Requested backend: {}".format(backend_details.get("backend")))
        print("Serial port: {}".format(backend_details.get("serial_port") or "n/a"))
        print("Baudrate: {}".format(backend_details.get("baudrate") or "n/a"))
        print("Reason: {}".format(exc))
        raise SystemExit(1)

    joint_map = _build_joint_map(config)
    results = {}
    bus.logger.log(
        "servo_position_read_start",
        backend=bus.backend.name,
        dry_run=bus.dry_run,
        ids=servo_ids,
        serial_port=backend_details.get("serial_port"),
        baudrate=backend_details.get("baudrate"),
    )
    try:
        for servo_id in servo_ids:
            joint_name = joint_map.get(servo_id, "unmapped")
            ping_ok = None
            position = None
            failure_reason = None
            try:
                ping_ok = bus.ping(servo_id)
                if not ping_ok:
                    failure_reason = "servo_not_responding_to_ping"
                else:
                    position = bus.read_position(servo_id)
                    if position is None:
                        failure_reason = "present_position_unavailable"
            except Exception as exc:
                failure_reason = str(exc)
            results[servo_id] = {
                "joint": joint_name,
                "ping_ok": ping_ok,
                "position": position,
                "failure_reason": failure_reason,
            }
    finally:
        bus.logger.log(
            "servo_position_read_complete",
            backend=bus.backend.name,
            dry_run=bus.dry_run,
            serial_port=backend_details.get("serial_port"),
            baudrate=backend_details.get("baudrate"),
            results=results,
        )
        bus.close()

    mode = "DRY-RUN" if bus.dry_run else "REAL READ-ONLY"
    print("{} position read complete using {} backend.".format(mode, bus.backend.name))
    print("Backend used: {}".format(bus.backend.name))
    print("Serial port: {}".format(backend_details.get("serial_port") or "n/a"))
    print("Baudrate: {}".format(backend_details.get("baudrate") or "n/a"))
    for servo_id in servo_ids:
        result = results.get(servo_id) or {}
        if result.get("position") is None:
            print(
                "ID {} joint={} position=unavailable reason={}".format(
                    servo_id,
                    result.get("joint", "unmapped"),
                    result.get("failure_reason") or "unknown",
                )
            )
        else:
            print(
                "ID {} joint={} position={}".format(
                    servo_id,
                    result.get("joint", "unmapped"),
                    result.get("position"),
                )
            )
    print("Log: {}".format(bus.logger.path))


if __name__ == "__main__":
    main()
