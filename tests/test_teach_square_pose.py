from __future__ import absolute_import

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map
import tools.teach_square_pose as teach_square_pose


JOINTS_RAW = (
    "shoulder_pan=2000,shoulder_lift=2000,elbow_flex=2000,"
    "wrist_flex=2000,wrist_roll=1000,gripper=1600"
)
JOINTS = {
    "shoulder_pan": 2000,
    "shoulder_lift": 2000,
    "elbow_flex": 2000,
    "wrist_flex": 2000,
    "wrist_roll": 1000,
    "gripper": 1600,
}


def _joint_limits(minimum=900, maximum=3000):
    return {
        "limits": {
            name: {"provisional_min": minimum, "provisional_max": maximum}
            for name in robot_square_map.DEFAULT_JOINT_ORDER
        }
    }


def _servo_map():
    return {
        "joints": {
            name: {"id": index + 1}
            for index, name in enumerate(robot_square_map.DEFAULT_JOINT_ORDER)
        }
    }


def _write_yaml(tmpdir, name, data):
    path = str(tmpdir.join(name))
    robot_square_map.save_yaml_file(path, data)
    return path


def _paths(tmpdir, targets=None, limits=None):
    targets_path = _write_yaml(
        tmpdir,
        "square_targets.yaml",
        targets if targets is not None else robot_square_map.default_square_targets(),
    )
    limits_path = _write_yaml(tmpdir, "joint_limits.yaml", limits if limits is not None else _joint_limits())
    servo_map_path = _write_yaml(tmpdir, "servo_map.yaml", _servo_map())
    return targets_path, limits_path, servo_map_path


def _run_main(monkeypatch, tmpdir, extra_args, targets=None, limits=None):
    targets_path, limits_path, servo_map_path = _paths(tmpdir, targets=targets, limits=limits)
    argv = [
        "teach_square_pose.py",
        "--square", "c3",
        "--targets", targets_path,
        "--joint-limits", limits_path,
        "--servo-map", servo_map_path,
    ] + list(extra_args)
    monkeypatch.setattr(sys, "argv", argv)
    return teach_square_pose.main(), targets_path


def _load(path):
    return robot_square_map.load_square_targets(path)


def test_pose_name_defaults_to_above_pose(monkeypatch, tmpdir, capsys):
    code, _ = _run_main(monkeypatch, tmpdir, ["--joints", JOINTS_RAW])
    assert code == 0
    assert "Pose name: above_pose" in capsys.readouterr().out


def test_allowed_pose_names_are_defined():
    assert robot_square_map.ALLOWED_POSE_NAMES == ["above_pose", "pick_pose", "place_pose"]
    for pose_name in robot_square_map.ALLOWED_POSE_NAMES:
        assert robot_square_map.validate_pose_name(pose_name) == pose_name


def test_invalid_pose_name_is_rejected():
    with pytest.raises(robot_square_map.SquareTargetError):
        robot_square_map.validate_pose_name("drop_pose")


def test_dry_run_pick_pose_does_not_write_file(monkeypatch, tmpdir):
    code, targets_path = _run_main(
        monkeypatch,
        tmpdir,
        ["--pose-name", "pick_pose", "--joints", JOINTS_RAW, "--note", "open-loop baseline pick depth"],
    )
    assert code == 0
    assert "pick_pose" not in _load(targets_path)["squares"].get("c3", {})


def test_write_pick_pose_creates_entry(monkeypatch, tmpdir):
    code, targets_path = _run_main(
        monkeypatch,
        tmpdir,
        ["--pose-name", "pick_pose", "--joints", JOINTS_RAW, "--note", "open-loop baseline pick depth", "--write"],
    )
    assert code == 0
    pose = _load(targets_path)["squares"]["c3"]["pick_pose"]
    assert pose["source"] == "manual"
    assert pose["confidence"] == "taught"
    assert pose["joints"] == JOINTS
    assert pose["notes"] == ["open-loop baseline pick depth"]
    assert pose.get("recorded_at")


def test_write_place_pose_creates_entry(monkeypatch, tmpdir):
    code, targets_path = _run_main(
        monkeypatch,
        tmpdir,
        ["--pose-name", "place_pose", "--joints", JOINTS_RAW, "--note", "open-loop baseline place depth", "--write"],
    )
    assert code == 0
    assert _load(targets_path)["squares"]["c3"]["place_pose"]["notes"] == ["open-loop baseline place depth"]


def test_existing_manual_pick_pose_refuses_overwrite_without_force(monkeypatch, tmpdir):
    document = robot_square_map.default_square_targets()
    document["squares"]["c3"] = {"pick_pose": robot_square_map.build_manual_pose_entry(JOINTS)}
    code, targets_path = _run_main(
        monkeypatch,
        tmpdir,
        ["--pose-name", "pick_pose", "--joints", JOINTS_RAW, "--write"],
        targets=document,
    )
    assert code == 1
    assert _load(targets_path)["squares"]["c3"]["pick_pose"]["joints"] == JOINTS


def test_existing_manual_pick_pose_overwrites_with_force(monkeypatch, tmpdir):
    document = robot_square_map.default_square_targets()
    old_joints = dict(JOINTS)
    old_joints["gripper"] = 1700
    document["squares"]["c3"] = {"pick_pose": robot_square_map.build_manual_pose_entry(old_joints)}
    code, targets_path = _run_main(
        monkeypatch,
        tmpdir,
        ["--pose-name", "pick_pose", "--joints", JOINTS_RAW, "--write", "--force"],
        targets=document,
    )
    assert code == 0
    assert _load(targets_path)["squares"]["c3"]["pick_pose"]["joints"] == JOINTS


def test_above_pose_backwards_compatibility(monkeypatch, tmpdir):
    code, targets_path = _run_main(monkeypatch, tmpdir, ["--joints", JOINTS_RAW, "--write"])
    assert code == 0
    assert _load(targets_path)["squares"]["c3"]["above_pose"]["joints"] == JOINTS


def test_validation_rejects_missing_joints():
    with pytest.raises(ValueError):
        teach_square_pose._parse_joint_values("shoulder_pan=2000", robot_square_map.DEFAULT_JOINT_ORDER)


def test_validation_rejects_unknown_joints():
    with pytest.raises(ValueError):
        teach_square_pose._parse_joint_values(JOINTS_RAW + ",extra=1", robot_square_map.DEFAULT_JOINT_ORDER)


def test_validation_rejects_out_of_limit_joint_values(monkeypatch, tmpdir):
    code, _ = _run_main(
        monkeypatch,
        tmpdir,
        ["--pose-name", "pick_pose", "--joints", JOINTS_RAW],
        limits=_joint_limits(minimum=2100, maximum=3000),
    )
    assert code == 1


def test_writing_pick_pose_preserves_existing_generated_above_pose(monkeypatch, tmpdir):
    document = robot_square_map.default_square_targets()
    generated_above = {
        "source": "generated",
        "confidence": "interpolated",
        "joints": dict(JOINTS),
        "generated_at": "2026-01-01T00:00:00Z",
        "notes": ["keep me"],
    }
    document["squares"]["c3"] = {"above_pose": generated_above, "metadata": {"operator": "test"}}
    code, targets_path = _run_main(
        monkeypatch,
        tmpdir,
        ["--pose-name", "pick_pose", "--joints", JOINTS_RAW, "--write"],
        targets=document,
    )
    assert code == 0
    square = _load(targets_path)["squares"]["c3"]
    assert square["above_pose"] == generated_above
    assert square["metadata"] == {"operator": "test"}
    assert square["pick_pose"]["source"] == "manual"


def test_write_above_pose_replaces_generated_above_pose_without_force(monkeypatch, tmpdir):
    document = robot_square_map.default_square_targets()
    generated_above = {
        "source": "generated",
        "confidence": "interpolated",
        "joints": dict(JOINTS),
        "generated_at": "2026-01-01T00:00:00Z",
        "notes": ["generated"],
    }
    document["squares"]["c3"] = {"above_pose": generated_above}
    code, targets_path = _run_main(monkeypatch, tmpdir, ["--joints", JOINTS_RAW, "--write"], targets=document)
    assert code == 0
    pose = _load(targets_path)["squares"]["c3"]["above_pose"]
    assert pose["source"] == "manual"
    assert pose["confidence"] == "taught"
    assert pose["joints"] == JOINTS
