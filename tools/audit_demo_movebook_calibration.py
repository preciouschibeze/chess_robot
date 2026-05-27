#!/usr/bin/env python3
"""Audit movebook robot-reply square calibration coverage without hardware access."""

from __future__ import absolute_import

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map
from chess_robot.chess_logic.demo_movebook import DemoMovebook, DemoMovebookError

DEFAULT_MOVEBOOK_PATH = os.path.join(ROOT, "configs", "demo_movebook.yaml")
DEFAULT_SQUARE_TARGETS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "square_targets.yaml")


class AuditError(RuntimeError):
    """Raised when audit inputs are invalid."""


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--movebook",
        default=DEFAULT_MOVEBOOK_PATH,
        help="Demo movebook YAML path.",
    )
    parser.add_argument(
        "--square-targets",
        default=DEFAULT_SQUARE_TARGETS_PATH,
        help="Square-target calibration YAML path.",
    )
    return parser


def _row_status(source_above_ok, source_pick_ok, dest_above_ok, dest_place_ok):
    if source_above_ok and source_pick_ok and dest_above_ok and dest_place_ok:
        return "ok"
    return "missing"


def _bool_text(value):
    return "yes" if bool(value) else "no"


def _format_table(rows):
    headers = [
        "move",
        "source",
        "destination",
        "source above",
        "source pick",
        "dest above",
        "dest place",
        "status",
    ]
    widths = [len(value) for value in headers]

    for row in rows:
        values = [
            row["move"],
            row["source"],
            row["destination"],
            _bool_text(row["source_above"]),
            _bool_text(row["source_pick"]),
            _bool_text(row["dest_above"]),
            _bool_text(row["dest_place"]),
            row["status"],
        ]
        for i, value in enumerate(values):
            if len(value) > widths[i]:
                widths[i] = len(value)

    def fmt_line(values):
        padded = []
        for i, value in enumerate(values):
            padded.append(value.ljust(widths[i]))
        return " | ".join(padded)

    table_lines = [fmt_line(headers), "-+-".join(["-" * width for width in widths])]
    for row in rows:
        table_lines.append(
            fmt_line([
                row["move"],
                row["source"],
                row["destination"],
                _bool_text(row["source_above"]),
                _bool_text(row["source_pick"]),
                _bool_text(row["dest_above"]),
                _bool_text(row["dest_place"]),
                row["status"],
            ])
        )
    return "\n".join(table_lines)


def audit_movebook_calibration(movebook_path, square_targets_path):
    movebook = DemoMovebook.from_path(movebook_path)
    square_targets = robot_square_map.load_square_targets(square_targets_path)
    squares = square_targets.get("squares") or {}

    rows = []
    missing = []

    for human_move, robot_move in sorted(movebook.to_dict().items()):
        if not DemoMovebook.is_uci_like(robot_move):
            raise AuditError("Malformed robot move in movebook: {} -> {}".format(human_move, robot_move))

        source = robot_move[:2]
        destination = robot_move[2:4]

        source_info = squares.get(source)
        if not isinstance(source_info, dict):
            source_info = {}
        destination_info = squares.get(destination)
        if not isinstance(destination_info, dict):
            destination_info = {}

        source_above_ok = source_info.get("above_pose") is not None
        source_pick_ok = source_info.get("pick_pose") is not None
        dest_above_ok = destination_info.get("above_pose") is not None
        dest_place_ok = destination_info.get("place_pose") is not None

        if not source_above_ok:
            missing.append("{}.above_pose".format(source))
        if not source_pick_ok:
            missing.append("{}.pick_pose".format(source))
        if not dest_above_ok:
            missing.append("{}.above_pose".format(destination))
        if not dest_place_ok:
            missing.append("{}.place_pose".format(destination))

        rows.append({
            "move": robot_move,
            "source": source,
            "destination": destination,
            "source_above": source_above_ok,
            "source_pick": source_pick_ok,
            "dest_above": dest_above_ok,
            "dest_place": dest_place_ok,
            "status": _row_status(source_above_ok, source_pick_ok, dest_above_ok, dest_place_ok),
        })

    unique_missing = sorted(set(missing))
    return {
        "rows": rows,
        "missing": unique_missing,
        "ok": len(unique_missing) == 0,
    }


def run_audit(movebook_path, square_targets_path, stream=None):
    if stream is None:
        stream = sys.stdout

    try:
        report = audit_movebook_calibration(movebook_path, square_targets_path)
    except (DemoMovebookError, robot_square_map.SquareTargetError, ValueError, IOError, OSError, AuditError) as exc:
        stream.write("Audit failed: {}\n".format(exc))
        return 2, None

    stream.write(_format_table(report["rows"]))
    stream.write("\n")

    if report["missing"]:
        stream.write("\nMissing required calibration:\n")
        for entry in report["missing"]:
            stream.write("- {}\n".format(entry))
        return 1, report

    stream.write("\nAll required movebook calibration poses are present.\n")
    return 0, report


def main():
    args = build_parser().parse_args()
    code, _ = run_audit(args.movebook, args.square_targets)
    return code


if __name__ == "__main__":
    sys.exit(main())
