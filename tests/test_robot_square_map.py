from __future__ import absolute_import

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map


def test_grid_to_square_robot_black_mapping():
    assert robot_square_map.grid_to_square(0, 0) == "h1"
    assert robot_square_map.grid_to_square(0, 7) == "a1"
    assert robot_square_map.grid_to_square(7, 0) == "h8"
    assert robot_square_map.grid_to_square(7, 7) == "a8"


def test_square_to_grid_is_inverse_for_all_squares():
    for row in range(8):
        for col in range(8):
            square_name = robot_square_map.grid_to_square(row, col)
            assert robot_square_map.square_to_grid(square_name) == (row, col)


def test_invalid_square_labels_are_rejected():
    for value in ("", "aa", "i1", "a9", "11", None):
        with pytest.raises(ValueError):
            robot_square_map.square_to_grid(value)


def test_yaml_round_trip_preserves_existing_entries(tmpdir):
    temp_path = str(tmpdir.join("square_targets.yaml"))
    document = robot_square_map.default_square_targets()
    document["metadata"]["operator"] = "test"
    document["squares"]["e4"] = {
        "above_pose": robot_square_map.build_manual_above_pose({
            "shoulder_pan": 2000,
            "shoulder_lift": 2100,
            "elbow_flex": 2200,
            "wrist_flex": 2300,
            "wrist_roll": 1200,
            "gripper": 1600,
        }, note="manual"),
        "pick_pose": {"source": "manual"},
    }
    robot_square_map.save_yaml_file(temp_path, document)
    loaded = robot_square_map.load_square_targets(temp_path)
    assert loaded["metadata"]["operator"] == "test"
    assert loaded["squares"]["e4"]["above_pose"]["source"] == "manual"
    assert loaded["squares"]["e4"]["pick_pose"]["source"] == "manual"
