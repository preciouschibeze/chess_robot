#!/usr/bin/env python3
"""Compare occupancy snapshots and emit transition diagnostics plus a grid visual."""

import argparse
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from chess_robot.vision.state_transition import compare_occupancy_snapshots  # noqa: E402

DEFAULT_LATEST_RESULT = "data/debug/latest_transition_result.json"
DEFAULT_LATEST_GRID = "data/debug/latest_transition_grid.png"


def build_parser():
    parser = argparse.ArgumentParser(
        description="Compare previous/current occupancy JSON snapshots and report changed squares."
    )
    parser.add_argument("--previous", required=True, help="Path to previous occupancy JSON snapshot.")
    parser.add_argument("--current", required=True, help="Path to current occupancy JSON snapshot.")
    parser.add_argument(
        "--output-dir",
        default="data/debug",
        help="Directory for transition output files. Default: data/debug",
    )
    return parser


def load_json(path):
    with open(path, "r") as handle:
        return json.load(handle)


def save_json(path, payload):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _square_to_grid(square_name):
    file_name = square_name[0]
    rank = int(square_name[1])
    col = "hgfedcba".index(file_name)
    row = rank - 1
    return row, col


def _draw_cell(canvas, row, col, text_top, text_mid, bg):
    cell = 80
    x0 = col * cell
    y0 = row * cell
    x1 = x0 + cell
    y1 = y0 + cell
    cv2.rectangle(canvas, (x0, y0), (x1, y1), bg, -1)
    cv2.rectangle(canvas, (x0, y0), (x1, y1), (30, 30, 30), 1)
    cv2.putText(canvas, text_top, (x0 + 6, y0 + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (10, 10, 10), 1, cv2.LINE_AA)
    if text_mid:
        cv2.putText(canvas, text_mid, (x0 + 6, y0 + 52), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 0, 0), 2, cv2.LINE_AA)


def render_transition_grid(previous, current, transition, output_path):
    squares_prev = previous.get("squares", {}) if isinstance(previous, dict) else {}
    squares_cur = current.get("squares", {}) if isinstance(current, dict) else {}
    changed_by_square = {}
    for entry in transition.get("changed_squares", []):
        if isinstance(entry, dict) and isinstance(entry.get("square"), str):
            changed_by_square[entry["square"]] = entry

    canvas = np.full((8 * 80, 8 * 80, 3), 240, dtype=np.uint8)

    for rank in range(1, 9):
        for file_name in "abcdefgh":
            square = "{}{}".format(file_name, rank)
            row, col = _square_to_grid(square)

            prev_entry = squares_prev.get(square, {})
            cur_entry = squares_cur.get(square, {})
            prev_state = prev_entry.get("state") if isinstance(prev_entry, dict) else prev_entry
            cur_state = cur_entry.get("state") if isinstance(cur_entry, dict) else cur_entry
            ch = changed_by_square.get(square)

            text_mid = ""
            bg = (245, 245, 245)
            if ch:
                change_type = ch.get("change_type")
                if change_type == "added":
                    bg = (190, 235, 190)
                    text_mid = "+ {}".format(square)
                elif change_type == "removed":
                    bg = (205, 205, 245)
                    text_mid = "- {}".format(square)
                elif change_type == "uncertain":
                    bg = (185, 230, 245)
                    text_mid = "? {}".format(square)
                else:
                    bg = (225, 225, 225)
                    text_mid = "{}>{}".format((prev_state or "?")[0], (cur_state or "?")[0])
            else:
                if cur_state == "occupied":
                    text_mid = "occ"
                elif cur_state == "empty":
                    text_mid = "."

            _draw_cell(canvas, row, col, square, text_mid, bg)

    directory = os.path.dirname(output_path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    cv2.imwrite(output_path, canvas)


def main():
    args = build_parser().parse_args()
    previous = load_json(args.previous)
    current = load_json(args.current)

    result = compare_occupancy_snapshots(previous, current)

    os.makedirs(args.output_dir, exist_ok=True)
    transition_path = os.path.join(args.output_dir, "transition_result.json")
    transition_grid_path = os.path.join(args.output_dir, "transition_grid.png")

    save_json(transition_path, result)
    save_json(DEFAULT_LATEST_RESULT, result)
    render_transition_grid(previous, current, result, transition_grid_path)
    render_transition_grid(previous, current, result, DEFAULT_LATEST_GRID)

    summary = result.get("summary", {})
    print("status: {}".format(summary.get("status")))
    print("transition_type: {}".format(summary.get("transition_type")))
    print("changed_count: {}".format(summary.get("changed_count")))
    print("added_squares: {}".format(result.get("added_squares", [])))
    print("removed_squares: {}".format(result.get("removed_squares", [])))
    print("uncertain_squares: {}".format(result.get("uncertain_squares", [])))
    print("transition_result: {}".format(transition_path))
    print("transition_grid: {}".format(transition_grid_path))
    print("latest_transition_result: {}".format(DEFAULT_LATEST_RESULT))
    print("latest_transition_grid: {}".format(DEFAULT_LATEST_GRID))


if __name__ == "__main__":
    main()
