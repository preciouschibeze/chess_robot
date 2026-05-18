#!/usr/bin/env python
"""Plan a dry-run symbolic robot action sequence for a chess move."""

from __future__ import absolute_import

import argparse
import io
import json
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from chess_robot.planning.task_planner import plan_chess_move


def _load_fen(args):
    if args.fen is not None:
        if args.fen.strip().lower() == "startpos":
            return "startpos"
        return args.fen.strip()

    with io.open(args.fen_file, "r", encoding="utf-8") as handle:
        text = handle.read().strip()
    return text


def main():
    parser = argparse.ArgumentParser(description="Plan dry-run robot actions for one UCI chess move.")
    parser.add_argument("--fen", default=None, help="FEN string or startpos")
    parser.add_argument("--fen-file", default="data/game/current_fen.txt", help="Path to FEN text file")
    parser.add_argument("--move", required=True, help="UCI move such as d7d5")
    parser.add_argument("--output", default="data/debug/latest_move_plan.json", help="Output JSON file path")
    parser.add_argument("--capture-zone", default="capture_zone", help="Capture zone symbolic name")
    args = parser.parse_args()

    fen_input = _load_fen(args)
    if fen_input.lower() == "startpos":
        fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
    else:
        fen = fen_input

    plan = plan_chess_move(fen=fen, move_uci=args.move, capture_zone_name=args.capture_zone)

    output_dir = os.path.dirname(args.output)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    with io.open(args.output, "w", encoding="utf-8") as handle:
        json.dump(plan.to_dict(), handle, indent=2, sort_keys=False)
        handle.write("\n")

    print("FEN before: %s" % plan.fen_before)
    print("move UCI: %s" % plan.move_uci)
    print("SAN: %s" % (plan.move_san or ""))
    print("move_type: %s" % plan.move_type)
    print("supported: %s" % str(plan.supported).lower())
    print("actions: %d" % len(plan.actions))
    print("output: %s" % args.output)

    if (not plan.supported) or plan.move_type == "illegal":
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
