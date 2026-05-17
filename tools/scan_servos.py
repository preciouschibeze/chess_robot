#!/usr/bin/env python3
"""Safely scan servo IDs by pinging only.

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
from chess_robot.robot.servo_bus import build_servo_bus, load_robot_config


def _parse_id_list(value: Optional[str]) -> Optional[List[int]]:
    if value is None or value == "":
        return None
    return safety.validate_servo_ids(part.strip() for part in value.split(",") if part.strip())


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Ping servo IDs for discovery. This tool never writes registers or moves servos."
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
                        help="Comma-separated IDs to ping instead of scanning a range, e.g. 1,2,3.")
    parser.add_argument("--start-id", type=int, default=None,
                        help="First ID in inclusive scan range.")
    parser.add_argument("--end-id", type=int, default=None,
                        help="Last ID in inclusive scan range.")
    parser.add_argument("--mock-ids", default=None,
                        help="Comma-separated mock IDs to report as present during dry-run.")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    config = load_robot_config(args.config)
    servo_config = config.get("servo_bus") or {}
    scan_config = servo_config.get("scan") or {}
    dry_run = servo_config.get("dry_run_default", True) if args.dry_run is None else args.dry_run
    safety.require_read_only_hardware_confirmation(bool(dry_run), args.yes)

    explicit_ids = _parse_id_list(args.ids)
    mock_ids = _parse_id_list(args.mock_ids)
    if explicit_ids is None:
        start_id = args.start_id if args.start_id is not None else scan_config.get("default_start_id", 1)
        end_id = args.end_id if args.end_id is not None else scan_config.get("default_end_id", 253)
        servo_ids = safety.validate_id_range(start_id, end_id)
    else:
        servo_ids = explicit_ids

    bus = build_servo_bus(
        config=config,
        config_path=args.config,
        dry_run=bool(dry_run),
        backend_name=args.backend,
        mock_ids=mock_ids,
    )

    found = []
    bus.logger.log(
        "servo_scan_start",
        backend=bus.backend.name,
        dry_run=bus.dry_run,
        ids=servo_ids,
    )
    try:
        for servo_id in servo_ids:
            if bus.ping(servo_id):
                found.append(servo_id)
    finally:
        bus.logger.log(
            "servo_scan_complete",
            backend=bus.backend.name,
            dry_run=bus.dry_run,
            scanned_count=len(servo_ids),
            found_ids=found,
        )
        bus.close()

    mode = "DRY-RUN" if bus.dry_run else "REAL READ-ONLY"
    print("{} servo scan complete using {} backend.".format(mode, bus.backend.name))
    print("Scanned {} ID(s). Found: {}".format(len(servo_ids), found if found else "none"))
    print("Log: {}".format(bus.logger.path))


if __name__ == "__main__":
    main()
