from __future__ import absolute_import

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
TOOLS_ROOT = os.path.join(ROOT, "tools")
if TOOLS_ROOT not in sys.path:
    sys.path.insert(0, TOOLS_ROOT)

import analyse_square_ik

from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_joint_limits
from chess_robot.robot.joint_calibration import tick_to_angle_rad
from chess_robot.robot.joint_limits import LEGACY_HARD_LIMITS_WARNING
from chess_robot.robot.joint_limits import convert_joint_preferences_to_urdf_radians
from chess_robot.robot.joint_limits import convert_joint_safety_limits_to_angle_limits
from chess_robot.robot.joint_limits import load_joint_preferences
from chess_robot.robot.joint_limits import load_joint_safety_limits
from chess_robot.robot.joint_limits import resolve_hard_limit_profile

JOINT_CALIBRATION_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
LEGACY_JOINT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
JOINT_SAFETY_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")
JOINT_PREFERENCES_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_preferences.yaml")


def test_joint_safety_limit_loader_accepts_new_schema():
    joint_safety_limits = load_joint_safety_limits(JOINT_SAFETY_LIMITS_PATH)
    shoulder_pan = joint_safety_limits["joints"]["shoulder_pan"]
    assert joint_safety_limits["profile_kind"] == "safety"
    assert joint_safety_limits["calibrated"] is False
    assert shoulder_pan["min_tick"] == 876
    assert shoulder_pan["max_tick"] == 3078
    assert shoulder_pan["status"] == "copied_from_previous_joint_limits_until_recalibrated"


def test_joint_preferences_loader_accepts_preferred_tick_and_range():
    joint_preferences = load_joint_preferences(JOINT_PREFERENCES_PATH)
    wrist_roll = joint_preferences["joints"]["wrist_roll"]
    wrist_flex = joint_preferences["joints"]["wrist_flex"]
    assert wrist_roll["preferred_tick"] == 1093
    assert abs(wrist_roll["weight"] - 0.8) < 1.0e-9
    assert wrist_flex["preferred_range_ticks"] == [1145, 2935]


def test_legacy_joint_limits_yaml_still_loads():
    legacy_joint_limits = load_joint_limits(LEGACY_JOINT_LIMITS_PATH)
    assert legacy_joint_limits["shoulder_pan"]["provisional_min"] == 876
    assert legacy_joint_limits["wrist_roll"]["provisional_max"] == 2350


def test_joint_safety_limits_convert_to_urdf_radians():
    calibration = load_joint_calibration(JOINT_CALIBRATION_PATH)
    joint_safety_limits = load_joint_safety_limits(JOINT_SAFETY_LIMITS_PATH)
    converted = convert_joint_safety_limits_to_angle_limits(joint_safety_limits, calibration)
    expected_lower = tick_to_angle_rad("base_yaw", 876, calibration)
    expected_upper = tick_to_angle_rad("base_yaw", 3078, calibration)
    shoulder_pan = converted["shoulder_pan"]
    assert abs(shoulder_pan["lower_rad"] - min(expected_lower, expected_upper)) < 1.0e-9
    assert abs(shoulder_pan["upper_rad"] - max(expected_lower, expected_upper)) < 1.0e-9


def test_joint_preferences_convert_to_urdf_radians():
    calibration = load_joint_calibration(JOINT_CALIBRATION_PATH)
    joint_preferences = load_joint_preferences(JOINT_PREFERENCES_PATH)
    converted = convert_joint_preferences_to_urdf_radians(joint_preferences, calibration)
    expected_preferred = tick_to_angle_rad("wrist_roll", 1093, calibration)
    assert abs(converted["wrist_roll"]["preferred_rad"] - expected_preferred) < 1.0e-9
    preferred_range = converted["wrist_flex"]["preferred_range_rad"]
    assert isinstance(preferred_range, list)
    assert len(preferred_range) == 2
    assert preferred_range[0] <= preferred_range[1]


def test_analyse_square_ik_accepts_joint_safety_limits_flag_without_breaking_joint_limits():
    parser = analyse_square_ik.build_parser()
    safety_args = parser.parse_args(
        [
            "--output-prefix",
            os.path.join(ROOT, "data", "debug", "joint_limit_profile_test"),
            "--joint-safety-limits",
            JOINT_SAFETY_LIMITS_PATH,
            "--joint-preferences",
            JOINT_PREFERENCES_PATH,
        ]
    )
    legacy_args = parser.parse_args(
        [
            "--output-prefix",
            os.path.join(ROOT, "data", "debug", "joint_limit_profile_test"),
            "--joint-limits",
            LEGACY_JOINT_LIMITS_PATH,
        ]
    )
    assert safety_args.joint_safety_limits == JOINT_SAFETY_LIMITS_PATH
    assert safety_args.joint_preferences == JOINT_PREFERENCES_PATH
    assert legacy_args.joint_limits == LEGACY_JOINT_LIMITS_PATH


def test_hard_limit_resolution_warns_on_legacy_fallback():
    legacy_joint_limits = load_joint_limits(LEGACY_JOINT_LIMITS_PATH)
    resolved_profile, profile_kind, warnings = resolve_hard_limit_profile(joint_limits=legacy_joint_limits)
    assert resolved_profile == legacy_joint_limits
    assert profile_kind == "legacy"
    assert warnings == [LEGACY_HARD_LIMITS_WARNING]
