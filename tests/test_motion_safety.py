from __future__ import absolute_import

import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot.motion_safety import approach_tilt_deg
from chess_robot.robot.motion_safety import validate_sampled_tcp_path


def test_path_validation_passes_for_vertical_motion_below_low_zone():
    summary = validate_sampled_tcp_path(
        [
            [0.10, 0.20, 0.04],
            [0.10, 0.20, 0.08],
        ],
        low_zone_z=0.09,
        xy_motion_epsilon_m=0.005,
    )
    assert summary["xy_changing"] is False
    assert summary["passed"] is True


def test_path_validation_fails_when_xy_changes_and_sample_dips_low():
    summary = validate_sampled_tcp_path(
        [
            [0.10, 0.20, 0.10],
            [0.12, 0.21, 0.05],
            [0.14, 0.22, 0.10],
        ],
        low_zone_z=0.09,
        xy_motion_epsilon_m=0.005,
    )
    assert summary["xy_changing"] is True
    assert summary["passed"] is False
    assert "below low zone" in summary["failure_reason"]


def test_path_validation_passes_when_xy_changes_above_low_zone():
    summary = validate_sampled_tcp_path(
        [
            [0.10, 0.20, 0.10],
            [0.12, 0.21, 0.095],
            [0.14, 0.22, 0.10],
        ],
        low_zone_z=0.09,
        xy_motion_epsilon_m=0.005,
    )
    assert summary["xy_changing"] is True
    assert summary["passed"] is True


def test_approach_tilt_is_zero_for_world_down_axis():
    assert approach_tilt_deg(np.asarray([0.0, 0.0, -1.0])) == 0.0


def test_approach_tilt_is_ninety_for_horizontal_axis():
    assert math.isclose(approach_tilt_deg(np.asarray([1.0, 0.0, 0.0])), 90.0, abs_tol=1.0e-9)
