import copy
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from chess_robot.vision.state_transition import (  # noqa: E402
    compare_occupancy_snapshots,
    grid_to_square,
    normalise_occupancy_snapshot,
)
from tools.detect_changed_squares import render_transition_grid  # noqa: E402


def make_snapshot(overrides=None, as_dict_entries=False, orientation="robot_black_side"):
    if overrides is None:
        overrides = {}
    squares = {}
    for rank in range(1, 9):
        for file_name in "abcdefgh":
            square = "{}{}".format(file_name, rank)
            value = overrides.get(square, "empty")
            if as_dict_entries:
                squares[square] = {
                    "state": value,
                    "score": 0.0 if value == "empty" else 0.2,
                    "confidence": 1.0,
                    "row": rank - 1,
                    "col": "hgfedcba".index(file_name),
                }
            else:
                squares[square] = value
    return {
        "board_orientation": orientation,
        "squares": squares,
    }


def test_normalise_occupancy_snapshot_simple_format():
    snapshot = make_snapshot({"e4": "occupied"}, as_dict_entries=True)
    normalized = normalise_occupancy_snapshot(snapshot)

    assert normalized["type"] == "occupancy_snapshot"
    assert normalized["board_orientation"] == "robot_black_side"
    assert normalized["squares"]["e4"]["state"] == "occupied"
    assert normalized["squares"]["e4"]["row"] == 3
    assert normalized["squares"]["e4"]["col"] == 3


def test_normalise_occupancy_snapshot_diagnostics_like_format():
    raw = {
        "board_orientation": "robot_black_side",
        "image_path": "data/snapshots/latest_undistorted.png",
        "occupied_threshold": 0.12,
        "uncertain_threshold": 0.06,
        "squares": make_snapshot({"d5": "occupied"}, as_dict_entries=True)["squares"],
    }
    raw["squares"]["d5"].pop("row")
    raw["squares"]["d5"].pop("col")

    normalized = normalise_occupancy_snapshot(raw)
    assert normalized["squares"]["d5"]["state"] == "occupied"
    assert normalized["squares"]["d5"]["score"] == raw["squares"]["d5"]["score"]
    assert normalized["source"]["image_path"] == "data/snapshots/latest_undistorted.png"


def test_empty_board_one_piece_added():
    previous = make_snapshot()
    current = make_snapshot({"e4": "occupied"})

    result = compare_occupancy_snapshots(previous, current)

    assert result["summary"]["transition_type"] == "add"
    assert result["summary"]["status"] == "clean"
    assert result["added_squares"] == ["e4"]
    assert result["removed_squares"] == []


def test_one_piece_removed():
    previous = make_snapshot({"e4": "occupied"})
    current = make_snapshot()

    result = compare_occupancy_snapshots(previous, current)

    assert result["summary"]["transition_type"] == "remove"
    assert result["summary"]["status"] == "clean"
    assert result["removed_squares"] == ["e4"]
    assert result["added_squares"] == []


def test_no_change():
    previous = make_snapshot({"e4": "occupied"})
    current = make_snapshot({"e4": "occupied"})
    result = compare_occupancy_snapshots(previous, current)

    assert result["summary"]["transition_type"] == "no_change"
    assert result["summary"]["status"] == "clean"
    assert result["summary"]["changed_count"] == 0


def test_uncertain_square_change():
    previous = make_snapshot({"e4": "empty"})
    current = make_snapshot({"e4": "uncertain"})

    result = compare_occupancy_snapshots(previous, current)

    assert result["summary"]["transition_type"] == "uncertain"
    assert result["summary"]["status"] == "uncertain"
    assert result["uncertain_squares"] == ["e4"]


def test_multi_change():
    previous = make_snapshot({"e2": "occupied", "g1": "occupied"})
    current = make_snapshot({"e4": "occupied", "f3": "occupied"})

    result = compare_occupancy_snapshots(previous, current)

    assert result["summary"]["transition_type"] == "multi_change"
    assert result["summary"]["status"] == "uncertain"
    assert result["summary"]["changed_count"] == 4


def test_robot_black_mapping():
    assert grid_to_square(0, 0) == "h1"
    assert grid_to_square(0, 7) == "a1"
    assert grid_to_square(7, 0) == "h8"
    assert grid_to_square(7, 7) == "a8"


def test_invalid_incomplete_snapshot():
    previous = make_snapshot()
    current = make_snapshot()
    current_bad = copy.deepcopy(current)
    del current_bad["squares"]["a1"]

    result = compare_occupancy_snapshots(previous, current_bad)

    assert result["summary"]["transition_type"] == "invalid"
    assert result["summary"]["status"] == "invalid"


def test_render_transition_grid_png(tmp_path):
    previous = make_snapshot({"e2": "occupied"}, as_dict_entries=True)
    current = make_snapshot({"e4": "occupied"}, as_dict_entries=True)
    result = compare_occupancy_snapshots(previous, current)

    out_path = str(tmp_path / "transition_grid.png")
    render_transition_grid(previous, current, result, out_path)

    assert os.path.exists(out_path)
    assert os.path.getsize(out_path) > 0


def test_cli_smoke_synthetic(tmp_path):
    prev_path = str(tmp_path / "prev.json")
    cur_path = str(tmp_path / "cur.json")
    out_dir = str(tmp_path / "out")

    with open(prev_path, "w") as h:
        json.dump(make_snapshot(as_dict_entries=True), h)
    with open(cur_path, "w") as h:
        json.dump(make_snapshot({"e4": "occupied"}, as_dict_entries=True), h)

    cmd = "python tools/detect_changed_squares.py --previous {} --current {} --output-dir {}".format(
        prev_path, cur_path, out_dir
    )
    rc = os.system(cmd)
    assert rc == 0
    assert os.path.exists(os.path.join(out_dir, "transition_result.json"))
    assert os.path.exists(os.path.join(out_dir, "transition_grid.png"))
