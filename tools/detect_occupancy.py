#!/usr/bin/env python3
"""Extract central square occupancy crops and save diagnostics."""

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from chess_robot.vision.occupancy import (  # noqa: E402
    DEFAULT_OUTPUT_DIR,
    DEFAULT_PROFILE_PATH,
    DEFAULT_UNCERTAIN_MARGIN,
    DEFAULT_UNCERTAIN_THRESHOLD,
    analyse_image,
    save_debug_outputs,
    save_result_json,
)
from chess_robot.vision.state_transition import normalise_occupancy_snapshot  # noqa: E402

DEFAULT_LATEST_SNAPSHOT = "data/debug/latest_occupancy_snapshot.json"


def build_parser():
    parser = argparse.ArgumentParser(
        description="Extract masked central square crops and conservative occupancy evidence."
    )
    parser.add_argument(
        "--image",
        required=True,
        help="Input board image path, for example data/snapshots/latest_undistorted.png.",
    )
    parser.add_argument(
        "--profile",
        default=DEFAULT_PROFILE_PATH,
        help="Board profile YAML path. Default: {}".format(DEFAULT_PROFILE_PATH),
    )
    parser.add_argument(
        "--empty-reference",
        default=None,
        help="Optional empty-board reference image for conservative occupied/empty scoring.",
    )
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for debug PNG outputs. Default: {}".format(DEFAULT_OUTPUT_DIR),
    )
    parser.add_argument(
        "--json-output",
        default=None,
        help="Diagnostics JSON path. Default: <output-dir>/occupancy_diagnostics.json.",
    )
    parser.add_argument(
        "--occupied-threshold",
        type=float,
        default=0.12,
        help="Normalized occupied threshold. Default: 0.12",
    )
    parser.add_argument(
        "--uncertain-threshold",
        type=float,
        default=DEFAULT_UNCERTAIN_THRESHOLD,
        help="Lower normalized uncertain threshold. Default: {}".format(DEFAULT_UNCERTAIN_THRESHOLD),
    )
    parser.add_argument(
        "--uncertain-margin",
        type=float,
        default=DEFAULT_UNCERTAIN_MARGIN,
        help=(
            "Deprecated compatibility option. If set, uses "
            "occupied_threshold - uncertain_margin as the uncertain threshold."
        ),
    )
    return parser


def _save_snapshot_outputs(result, output_dir):
    snapshot = normalise_occupancy_snapshot(result)
    snapshot_path = os.path.join(output_dir, "occupancy_snapshot.json")
    save_result_json(snapshot, snapshot_path)
    save_result_json(snapshot, DEFAULT_LATEST_SNAPSHOT)
    return snapshot_path, DEFAULT_LATEST_SNAPSHOT


def main():
    args = build_parser().parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    json_output = args.json_output
    if not json_output:
        json_output = os.path.join(args.output_dir, "occupancy_diagnostics.json")

    result = analyse_image(
        args.image,
        profile_path=args.profile,
        empty_reference_path=args.empty_reference,
        occupied_threshold=args.occupied_threshold,
        uncertain_threshold=args.uncertain_threshold,
        uncertain_margin=args.uncertain_margin,
    )
    save_result_json(result, json_output)
    snapshot_path, latest_snapshot_path = _save_snapshot_outputs(result, args.output_dir)
    outputs = save_debug_outputs(args.image, args.profile, result, args.output_dir)

    print("occupancy diagnostics: {}".format(json_output))
    print("occupancy snapshot: {}".format(snapshot_path))
    print("latest occupancy snapshot: {}".format(latest_snapshot_path))
    print("occupancy crop overlay: {}".format(outputs["occupancy_crop_overlay"]))
    print("square crops contact sheet: {}".format(outputs["square_crops_contact_sheet"]))
    print("occupancy grid: {}".format(outputs["occupancy_grid"]))


if __name__ == "__main__":
    main()
