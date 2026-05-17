#!/usr/bin/env python3
"""Read servo positions without commanding movement.

Dry-run is the default. Read-only real bus access requires --no-dry-run --yes and a
configured read-only backend in configs/robot.yaml.
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
                        help="Backend override. Dry-run uses mock regardless of configured backend.")
    parser.add_argument("--dry-run", dest="dry_run", action="store_true", default=None,
                        help="Use mock backend and avoid hardware access. This is the default from config.")
    parser.add_argument("--no-dry-run", dest="dry_run", action="store_false",
                        help="Allow read-only real bus access when combined with --yes.")
    parser.add_argument("--yes", action="store_true",
                        help="Confirm intentional read-only real servo bus access for --no-dry-run.")
    parser.add_argument("--ids", default=None,
                        help="Comma-separated servo IDs to read, e.g. 1,2,3. Defaults to configured joint/mock IDs.")
    parser.add_argument("--mock-ids", default=None,
                        help="Comma-separated mock IDs available during dry-run.")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    config = load_robot_config(args.config)
    servo_config = config.get("servo_bus") or {}
    dry_run = servo_config.get("dry_run_default", True) if args.dry_run is None else args.dry_run
    safety.require_read_only_hardware_confirmation(bool(dry_run), args.yes)

    explicit_ids = _parse_id_list(args.ids)
    mock_ids = _parse_id_list(args.mock_ids)
    if explicit_ids is None:
        servo_ids = configured_joint_servo_ids(config) or configured_mock_ids(config) or (mock_ids or [])
    else:
        servo_ids = explicit_ids
    servo_ids = safety.validate_servo_ids(servo_ids)
    if not servo_ids:
        parser.error("No servo IDs provided or configured. Use --ids 1,2,3 for a read.")

    bus = build_servo_bus(
        config=config,
        config_path=args.config,
        dry_run=bool(dry_run),
        backend_name=args.backend,
        mock_ids=mock_ids,
    )

    positions = {}
    bus.logger.log(
        "servo_position_read_start",
        backend=bus.backend.name,
        dry_run=bus.dry_run,
        ids=servo_ids,
    )
    try:
        for servo_id in servo_ids:
            positions[servo_id] = bus.read_position(servo_id)
    finally:
        bus.logger.log(
            "servo_position_read_complete",
            backend=bus.backend.name,
            dry_run=bus.dry_run,
            positions=positions,
        )
        bus.close()

    mode = "DRY-RUN" if bus.dry_run else "REAL READ-ONLY"
    print("{} position read complete using {} backend.".format(mode, bus.backend.name))
    for servo_id in servo_ids:
        value = positions.get(servo_id)
        print("ID {}: {}".format(servo_id, "unavailable" if value is None else value))
    print("Log: {}".format(bus.logger.path))


if __name__ == "__main__":
    main()
