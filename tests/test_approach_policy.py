from __future__ import absolute_import

import importlib.util
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
TOOLS_ROOT = os.path.join(ROOT, "tools")
if TOOLS_ROOT not in sys.path:
    sys.path.insert(0, TOOLS_ROOT)

from chess_robot.robot.approach_policy import load_approach_policy
from chess_robot.robot.approach_policy import resolve_approach_policy


POLICY_PATH = os.path.join(ROOT, "data", "calibration", "robot", "approach_policy.yaml")
SCENE_PATH = os.path.join(ROOT, "data", "calibration", "robot", "scene_geometry.yaml")
URDF_PATH = os.path.join(ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
JOINT_CALIBRATION_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
JOINT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
JOINT_SAFETY_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")
HOME_POSE_PATH = os.path.join(ROOT, "data", "calibration", "robot", "home_pose.yaml")
TOOL_FRAMES_PATH = os.path.join(ROOT, "data", "calibration", "gripper", "tool_frames.yaml")

CLI_SPEC = importlib.util.spec_from_file_location(
    "safe_square_transfer_cli",
    os.path.join(TOOLS_ROOT, "test_safe_square_transfer.py"),
)
safe_square_transfer_cli = importlib.util.module_from_spec(CLI_SPEC)
CLI_SPEC.loader.exec_module(safe_square_transfer_cli)


def _parse(square, extra=None):
    values = [
        "--urdf", URDF_PATH,
        "--scene", SCENE_PATH,
        "--joint-calibration", JOINT_CALIBRATION_PATH,
        "--joint-limits", JOINT_LIMITS_PATH,
        "--joint-safety-limits", JOINT_SAFETY_LIMITS_PATH,
        "--home-pose", HOME_POSE_PATH,
        "--tool-frames", TOOL_FRAMES_PATH,
        "--tcp-frame", "gripper_frame",
        "--square", square,
        "--workspace-seed-samples", "0",
        "--output", os.path.join("/tmp", "approach_policy_%s.json" % square),
        "--csv-log", os.path.join("/tmp", "approach_policy_%s.csv" % square),
    ]
    values.extend(list(extra or []))
    return safe_square_transfer_cli.build_parser().parse_args(values)


def test_default_policy_loads_correctly():
    policy_document = load_approach_policy(POLICY_PATH)
    assert policy_document["version"] == 1
    assert policy_document["default"]["prefer_vertical_approach"] is True
    assert policy_document["default"]["enforce_approach_angle"] is False
    assert policy_document["default"]["approach_axis_name"] == "plus_z"
    assert policy_document["default"]["approach_weight"] == 0.05
    assert policy_document["default"]["max_approach_tilt_deg"] == 20.0
    assert policy_document["default"]["max_edge_approach_tilt_deg"] == 30.0
    assert policy_document["default"]["normal_above_offset_m"] == 0.08
    assert policy_document["default"]["high_above_offset_m"] == 0.12
    assert policy_document["default"]["transit_clearance_m"] == 0.12
    assert policy_document["default"]["board_clearance_m"] == 0.06
    assert policy_document["default"]["return_route_squares"] == []
    assert policy_document["default"]["route_above_offset_m"] == 0.12
    assert policy_document["default"]["lock_wrist_roll_home"] is True


@pytest.mark.parametrize(
    "square_name,expected_route",
    [("a1", ["a2", "c3", "e4"]), ("b1", ["b2", "c3", "e4"]), ("h1", ["h2", "f3", "e4"])],
)
def test_square_override_applies_for_routed_far_rank_squares(square_name, expected_route):
    policy_info = resolve_approach_policy(load_approach_policy(POLICY_PATH), square_name)
    assert policy_info["policy_override_applied"] is True
    assert policy_info["resolved_policy"]["approach_axis_name"] == "plus_z"
    assert policy_info["resolved_policy"]["approach_weight"] == 0.02
    assert policy_info["resolved_policy"]["enforce_approach_angle"] is False
    assert policy_info["resolved_policy"]["return_route_squares"] == expected_route
    assert policy_info["resolved_policy"]["route_above_offset_m"] == 0.14


def test_non_overridden_square_uses_default_policy_values():
    policy_info = resolve_approach_policy(load_approach_policy(POLICY_PATH), "e4")
    assert policy_info["policy_override_applied"] is False
    assert policy_info["resolved_policy"]["approach_weight"] == 0.05
    assert policy_info["resolved_policy"]["normal_above_offset_m"] == 0.08
    assert policy_info["resolved_policy"]["return_route_squares"] == []
    assert policy_info["resolved_policy"]["route_above_offset_m"] == 0.12


def test_cli_approach_weight_override_wins_over_policy_default():
    args = _parse("a1", ["--approach-policy", POLICY_PATH, "--approach-weight", "0.07"])
    assert args.approach_weight == 0.07
    assert args.resolved_policy["approach_weight"] == 0.07
    assert args.policy_override_applied is True


def test_cli_normal_above_offset_override_wins_over_policy_default():
    args = _parse("e4", ["--approach-policy", POLICY_PATH, "--normal-above-offset-m", "0.095"])
    assert args.normal_above_offset_m == 0.095
    assert args.resolved_policy["normal_above_offset_m"] == 0.095
    assert args.policy_override_applied is False


def test_cli_return_route_override_wins_over_policy_default():
    args = _parse("a1", ["--approach-policy", POLICY_PATH, "--return-route-squares", "h2,f3,e4"])
    assert args.return_route_squares == ["h2", "f3", "e4"]
    assert args.resolved_policy["return_route_squares"] == ["h2", "f3", "e4"]


def test_cli_route_above_offset_override_wins_over_policy_default():
    args = _parse("a1", ["--approach-policy", POLICY_PATH, "--route-above-offset-m", "0.155"])
    assert args.route_above_offset_m == 0.155
    assert args.resolved_policy["route_above_offset_m"] == 0.155


def test_cli_high_above_override_updates_default_route_offset_when_route_omitted():
    args = _parse("e4", ["--approach-policy", POLICY_PATH, "--high-above-offset-m", "0.131"])
    assert args.high_above_offset_m == 0.131
    assert args.route_above_offset_m == 0.131
    assert args.resolved_policy["route_above_offset_m"] == 0.131


def test_missing_policy_file_gives_clear_error(capsys):
    missing_path = os.path.join(ROOT, "data", "calibration", "robot", "missing_approach_policy.yaml")
    with pytest.raises(SystemExit):
        _parse("e4", ["--approach-policy", missing_path])
    captured = capsys.readouterr()
    assert "Approach policy file was not found" in captured.err
    assert missing_path in captured.err


def test_policy_keeps_wrist_roll_locked_by_default():
    args = _parse("e4", ["--approach-policy", POLICY_PATH])
    assert args.lock_wrist_roll_home is True
    assert args.resolved_policy["lock_wrist_roll_home"] is True
