from __future__ import absolute_import

import json
import os
import tempfile

import yaml

from chess_robot.robot.motion_primitives import resolve_move_plan


def _joint_pose(seed):
    return {
        "base_yaw": seed + 1,
        "shoulder_pitch": seed + 2,
        "elbow_pitch": seed + 3,
        "wrist_pitch": seed + 4,
        "wrist_roll": seed + 5,
        "gripper": seed + 6,
    }


def _write_yaml(tmpdir, name, data):
    path = os.path.join(tmpdir, name)
    with open(path, "w") as handle:
        yaml.safe_dump(data, handle)
    return path


def _write_json(tmpdir, name, data):
    path = os.path.join(tmpdir, name)
    with open(path, "w") as handle:
        json.dump(data, handle)
    return path


def _quiet_plan():
    return {
        "move_uci": "d7d5",
        "move_type": "quiet",
        "actions": [
            {"name": "move_home", "params": {}},
            {"name": "move_above_square", "params": {"square": "d7"}},
            {"name": "descend_to_pick", "params": {"square": "d7"}},
            {"name": "close_gripper", "params": {}},
            {"name": "lift_from_square", "params": {"square": "d7"}},
            {"name": "move_above_square", "params": {"square": "d5"}},
            {"name": "descend_to_place", "params": {"square": "d5"}},
            {"name": "open_gripper", "params": {}},
            {"name": "lift_from_square", "params": {"square": "d5"}},
            {"name": "move_home", "params": {}},
        ],
    }


def _capture_plan():
    return {
        "move_uci": "e4d5",
        "move_type": "capture",
        "actions": [
            {"name": "move_to_capture_zone", "params": {}},
        ],
    }


def _complete_square_targets(include_d5_place=True, include_capture_zone=False):
    squares = {
        "d7": {
            "above_pose": _joint_pose(100),
            "pick_pose": _joint_pose(110),
            "place_pose": _joint_pose(120),
        },
        "d5": {
            "above_pose": _joint_pose(200),
            "pick_pose": _joint_pose(210),
        },
    }
    if include_d5_place:
        squares["d5"]["place_pose"] = _joint_pose(220)

    data = {"squares": squares}
    if include_capture_zone:
        data["zones"] = {
            "capture_zone": {
                "above_pose": _joint_pose(300),
                "place_pose": _joint_pose(310),
            }
        }
    return data


def _home_pose():
    return {"home_pose": _joint_pose(10)}


def _gripper_profile(include_grasp=True, include_release=True, include_open=True):
    gripper = {}
    if include_open:
        gripper["open_position"] = 1250
    if include_grasp:
        gripper["grasp_position"] = 1122
    if include_release:
        gripper["release_position"] = 1185
    return {"gripper": gripper}


def _resolve_with_tmp(plan, square_targets, home_pose, gripper_profile):
    tmpdir = tempfile.mkdtemp(prefix="test_motion_primitives_")
    plan_path = _write_json(tmpdir, "plan.json", plan)
    squares_path = _write_yaml(tmpdir, "square_targets.yaml", square_targets)
    home_path = _write_yaml(tmpdir, "home_pose.yaml", home_pose)
    gripper_path = _write_yaml(tmpdir, "gripper_profile.yaml", gripper_profile)
    with open(plan_path, "r") as handle:
        plan_dict = json.load(handle)
    return resolve_move_plan(plan_dict, squares_path, home_path, gripper_path)


def test_quiet_d7d5_complete_calibration_ready_true():
    result = _resolve_with_tmp(
        _quiet_plan(),
        _complete_square_targets(include_d5_place=True),
        _home_pose(),
        _gripper_profile(),
    )
    assert result.supported is True
    assert result.ready_for_execution is True
    assert result.missing_calibration == []
    assert len(result.steps) == 10


def test_quiet_d7d5_missing_d5_pick_place_reports_missing():
    square_targets = _complete_square_targets(include_d5_place=False)
    del square_targets["squares"]["d5"]["pick_pose"]

    result = _resolve_with_tmp(
        _quiet_plan(),
        square_targets,
        _home_pose(),
        _gripper_profile(),
    )
    assert result.supported is True
    assert result.ready_for_execution is False
    assert "squares.d5.pick_pose" in result.missing_calibration
    assert "squares.d5.place_pose" in result.missing_calibration


def test_missing_home_pose_reported():
    result = _resolve_with_tmp(
        _quiet_plan(),
        _complete_square_targets(include_d5_place=True),
        {},
        _gripper_profile(),
    )
    assert result.ready_for_execution is False
    assert "home_pose" in result.missing_calibration


def test_missing_gripper_positions_reported():
    result = _resolve_with_tmp(
        _quiet_plan(),
        _complete_square_targets(include_d5_place=True),
        _home_pose(),
        _gripper_profile(include_grasp=False, include_release=False, include_open=False),
    )
    assert result.ready_for_execution is False
    assert "gripper.grasp_position" in result.missing_calibration
    assert "gripper.release_position" in result.missing_calibration
    assert "gripper.open_position" in result.missing_calibration


def test_descend_to_place_falls_back_to_pick_pose_with_note():
    result = _resolve_with_tmp(
        _quiet_plan(),
        _complete_square_targets(include_d5_place=False),
        _home_pose(),
        _gripper_profile(),
    )
    assert result.ready_for_execution is True
    descend_steps = [s for s in result.steps if s.params.get("target") == "d5.pick_pose"]
    assert len(descend_steps) == 1
    assert "fell back to pick_pose" in " ".join(descend_steps[0].notes)


def test_unknown_action_marks_unsupported():
    plan = _quiet_plan()
    plan["actions"].append({"name": "teleport_piece", "params": {}})

    result = _resolve_with_tmp(
        plan,
        _complete_square_targets(include_d5_place=True),
        _home_pose(),
        _gripper_profile(),
    )
    assert result.supported is False
    assert result.ready_for_execution is False


def test_capture_without_zone_not_ready_and_reports_missing():
    result = _resolve_with_tmp(
        _capture_plan(),
        _complete_square_targets(include_capture_zone=False),
        _home_pose(),
        _gripper_profile(),
    )
    assert result.supported is False
    assert result.ready_for_execution is False
    assert "zones.capture_zone.above_pose" in result.missing_calibration
    assert "zones.capture_zone.place_pose" in result.missing_calibration


def test_capture_with_zone_calibration_resolves():
    result = _resolve_with_tmp(
        _capture_plan(),
        _complete_square_targets(include_capture_zone=True),
        _home_pose(),
        _gripper_profile(),
    )
    assert result.supported is True
    assert result.ready_for_execution is True
    assert result.missing_calibration == []
