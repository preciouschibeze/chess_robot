#!/usr/bin/env python3
"""Calibrate Gripper placeholder tool."""

import argparse


MESSAGE = 'Placeholder only. Gripper calibration is not implemented yet. Dry-run only; no hardware was accessed.'


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=MESSAGE)
    parser.add_argument(
        "--config",
        default=None,
        help="Optional config path reserved for future implementation.",
    )
    return parser


def main() -> None:
    build_parser().parse_args()
    print(MESSAGE)


if __name__ == "__main__":
    main()
