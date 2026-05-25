from __future__ import absolute_import

import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot.approach_orientation import best_candidate_axis
from chess_robot.robot.approach_orientation import inspect_candidate_axes


def _transform_with_rotation(rotation):
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = np.asarray(rotation, dtype=float)
    return transform


def test_candidate_axis_tilt_is_zero_when_local_axis_maps_to_world_down():
    ranked = inspect_candidate_axes(np.eye(4, dtype=float))
    minus_z = [item for item in ranked if item["axis_name"] == "minus_z"][0]
    assert math.isclose(float(minus_z["tilt_deg"]), 0.0, abs_tol=1.0e-9)


def test_candidate_axis_tilt_is_ninety_for_horizontal_axis():
    ranked = inspect_candidate_axes(np.eye(4, dtype=float))
    plus_x = [item for item in ranked if item["axis_name"] == "plus_x"][0]
    assert math.isclose(float(plus_x["tilt_deg"]), 90.0, abs_tol=1.0e-9)


def test_best_candidate_axis_selection_works_for_rotated_tcp_frame():
    rotation_y_ninety = np.asarray([
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ], dtype=float)
    best = best_candidate_axis(_transform_with_rotation(rotation_y_ninety))
    assert best["axis_name"] == "plus_x"
    assert math.isclose(float(best["tilt_deg"]), 0.0, abs_tol=1.0e-9)
