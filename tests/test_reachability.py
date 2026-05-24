from __future__ import absolute_import

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot.reachability import REPORT_FIELDNAMES
from chess_robot.robot.reachability import analyse_target_reachability
from chess_robot.robot.reachability import build_report_row
from chess_robot.robot.reachability import classify_distance
from chess_robot.robot.reachability import generate_square_centers
from chess_robot.robot.reachability import generate_targets
from chess_robot.robot.reachability import grid_to_square
from chess_robot.robot.urdf_model import EXPECTED_ARM_JOINT_NAMES


def _scene_geometry():
    return {
        "chessboard": {
            "xyz_m": np.asarray((0.003, 0.220, 0.026), dtype=float),
            "xy_center_m": np.asarray((0.003, 0.220), dtype=float),
            "size_xy_m": np.asarray((0.252, 0.252), dtype=float),
            "height_m": 0.026,
            "base_z_m": 0.0,
            "top_z_m": 0.026,
        },
        "capture_zone": {
            "xyz_m": np.asarray((0.203, 0.220, 0.0), dtype=float),
            "xy_center_m": np.asarray((0.203, 0.220), dtype=float),
            "size_xy_m": np.asarray((0.040, 0.140), dtype=float),
            "height_m": 0.050,
            "base_z_m": 0.0,
            "top_z_m": 0.050,
        },
    }


def _workspace_samples():
    return {
        "joint_names": list(EXPECTED_ARM_JOINT_NAMES),
        "positions": np.asarray(
            (
                (0.1, 0.2, 0.3, 0.4, 0.5),
                (0.6, 0.7, 0.8, 0.9, 1.0),
            ),
            dtype=float,
        ),
        "tcp_points_world_m": np.asarray(
            (
                (0.00, 0.00, 0.00),
                (0.06, 0.00, 0.00),
            ),
            dtype=float,
        ),
    }


def test_generate_square_centers_returns_64_centres():
    centers = generate_square_centers(_scene_geometry())
    assert len(centers) == 64


def test_black_side_square_mapping_matches_required_corners():
    assert grid_to_square(0, 0) == "h1"
    assert grid_to_square(0, 7) == "a1"
    assert grid_to_square(7, 0) == "h8"
    assert grid_to_square(7, 7) == "a8"


def test_square_centres_lie_inside_board_bounds():
    scene_geometry = _scene_geometry()
    centers = generate_square_centers(scene_geometry)
    board = scene_geometry["chessboard"]
    x_min = board["xy_center_m"][0] - (board["size_xy_m"][0] / 2.0)
    x_max = board["xy_center_m"][0] + (board["size_xy_m"][0] / 2.0)
    y_min = board["xy_center_m"][1] - (board["size_xy_m"][1] / 2.0)
    y_max = board["xy_center_m"][1] + (board["size_xy_m"][1] / 2.0)
    for center in centers:
        assert x_min < center["x_m"] < x_max
        assert y_min < center["y_m"] < y_max


def test_above_targets_have_higher_z_than_surface_targets():
    targets = generate_targets(_scene_geometry(), above_board_offset_m=0.080, pick_offset_m=0.030)
    surface_by_square = {}
    above_by_square = {}
    for target in targets:
        if target["target_type"] == "square_surface":
            surface_by_square[target["square"]] = target
        elif target["target_type"] == "square_above":
            above_by_square[target["square"]] = target
    for square, surface_target in surface_by_square.items():
        assert above_by_square[square]["z_m"] > surface_target["z_m"]


def test_classify_distance_matches_thresholds():
    assert classify_distance(0.010, 0.020, 0.050) == "reachable"
    assert classify_distance(0.030, 0.020, 0.050) == "marginal"
    assert classify_distance(0.060, 0.020, 0.050) == "unreachable"


def test_report_rows_contain_required_fields():
    target = {
        "target_name": "e4_surface",
        "target_type": "square_surface",
        "square": "e4",
        "x_m": 0.01,
        "y_m": 0.02,
        "z_m": 0.03,
    }
    row = build_report_row(
        target,
        nearest_xyz_m=np.asarray((0.0, 0.0, 0.0), dtype=float),
        nearest_distance_m=0.01,
        status="reachable",
        nearest_joint_map=dict((joint_name, 0.25) for joint_name in EXPECTED_ARM_JOINT_NAMES),
    )
    assert sorted(row.keys()) == sorted(REPORT_FIELDNAMES)


def test_report_rows_do_not_include_gripper_joint_output():
    rows = analyse_target_reachability(
        [
            {"target_name": "a1_surface", "target_type": "square_surface", "square": "a1", "x_m": 0.01, "y_m": 0.0, "z_m": 0.0},
            {"target_name": "a2_surface", "target_type": "square_surface", "square": "a2", "x_m": 0.03, "y_m": 0.0, "z_m": 0.0},
            {"target_name": "a3_surface", "target_type": "square_surface", "square": "a3", "x_m": 0.13, "y_m": 0.0, "z_m": 0.0},
        ],
        _workspace_samples(),
        reachable_threshold_m=0.020,
        marginal_threshold_m=0.050,
    )
    assert rows[0]["status"] == "reachable"
    assert rows[1]["status"] == "marginal"
    assert rows[2]["status"] == "unreachable"
    assert "nearest_gripper_rad" not in rows[0]
