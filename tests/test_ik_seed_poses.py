from __future__ import absolute_import

import importlib.util
import os
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
TOOLS_ROOT = os.path.join(ROOT, "tools")
if TOOLS_ROOT not in sys.path:
    sys.path.insert(0, TOOLS_ROOT)

from chess_robot.robot.ik_seed_poses import IKSeedPoseError
from chess_robot.robot.ik_seed_poses import default_ik_seed_poses_document
from chess_robot.robot.ik_seed_poses import load_ik_seed_poses
from chess_robot.robot.ik_seed_poses import prepare_square_ik_seed
from chess_robot.robot.ik_seed_poses import resolve_square_ik_seed
from chess_robot.robot.ik_seed_poses import upsert_square_seed_entry
from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_limits import load_joint_safety_limits


JOINT_CALIBRATION_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
JOINT_SAFETY_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")

SAVE_SPEC = importlib.util.spec_from_file_location(
    "save_current_ik_seed_pose_cli",
    os.path.join(TOOLS_ROOT, "save_current_ik_seed_pose.py"),
)
save_current_ik_seed_pose_cli = importlib.util.module_from_spec(SAVE_SPEC)
SAVE_SPEC.loader.exec_module(save_current_ik_seed_pose_cli)


def _write_seed_file(path, document):
    with open(path, "w") as handle:
        yaml.safe_dump(document, handle, default_flow_style=False)


def test_load_empty_seed_file(tmpdir):
    path = os.path.join(str(tmpdir), "ik_seed_poses.yaml")
    _write_seed_file(path, default_ik_seed_poses_document())
    document = load_ik_seed_poses(path)
    resolved = resolve_square_ik_seed(document, "a1")
    assert resolved["seed_applied"] is False
    assert resolved["seed_ticks"] == {}


def test_load_square_seed_ticks(tmpdir):
    path = os.path.join(str(tmpdir), "ik_seed_poses.yaml")
    document = default_ik_seed_poses_document()
    document["ik_seed_poses"]["squares"]["a1"]["seed_ticks"] = {
        "shoulder_pan": 1074,
        "shoulder_lift": 1776,
    }
    _write_seed_file(path, document)
    resolved = resolve_square_ik_seed(load_ik_seed_poses(path), "a1")
    assert resolved["seed_applied"] is True
    assert resolved["seed_ticks"]["shoulder_pan"] == 1074
    assert resolved["seed_ticks"]["shoulder_lift"] == 1776


def test_unknown_joint_in_seed_aborts(tmpdir):
    path = os.path.join(str(tmpdir), "ik_seed_poses.yaml")
    document = default_ik_seed_poses_document()
    document["ik_seed_poses"]["squares"]["a1"]["seed_ticks"] = {
        "banana": 1234,
    }
    _write_seed_file(path, document)
    calibration = load_joint_calibration(JOINT_CALIBRATION_PATH)
    joint_safety_limits = load_joint_safety_limits(JOINT_SAFETY_LIMITS_PATH)
    with pytest.raises(IKSeedPoseError) as exc:
        prepare_square_ik_seed(
            load_ik_seed_poses(path),
            "a1",
            calibration,
            joint_safety_limits,
            {},
            {},
        )
    assert "unknown joints" in str(exc.value)


def test_out_of_limit_seed_aborts(tmpdir):
    path = os.path.join(str(tmpdir), "ik_seed_poses.yaml")
    document = default_ik_seed_poses_document()
    document["ik_seed_poses"]["squares"]["a1"]["seed_ticks"] = {
        "shoulder_pan": 4000,
    }
    _write_seed_file(path, document)
    calibration = load_joint_calibration(JOINT_CALIBRATION_PATH)
    joint_safety_limits = load_joint_safety_limits(JOINT_SAFETY_LIMITS_PATH)
    with pytest.raises(IKSeedPoseError) as exc:
        prepare_square_ik_seed(
            load_ik_seed_poses(path),
            "a1",
            calibration,
            joint_safety_limits,
            {},
            {},
        )
    assert "outside joint safety limits" in str(exc.value)


def test_locked_wrist_roll_overrides_seed_wrist_roll(tmpdir):
    path = os.path.join(str(tmpdir), "ik_seed_poses.yaml")
    document = default_ik_seed_poses_document()
    document["ik_seed_poses"]["squares"]["a1"]["seed_ticks"] = {
        "shoulder_pan": 1074,
        "wrist_roll": 2500,
    }
    _write_seed_file(path, document)

    calibration = load_joint_calibration(JOINT_CALIBRATION_PATH)
    joint_safety_limits = load_joint_safety_limits(JOINT_SAFETY_LIMITS_PATH)
    locked_ticks = {"wrist_roll": 1091}
    locked_rad = convert_pose_ticks_to_urdf_radians(locked_ticks, calibration)
    prepared = prepare_square_ik_seed(
        load_ik_seed_poses(path),
        "a1",
        calibration,
        joint_safety_limits,
        locked_rad,
        locked_ticks,
    )
    assert prepared["seed_ticks_used"]["wrist_roll"] == 1091
    assert "wrist_roll" in prepared["seed_joints_used"]


def test_save_helper_updates_one_square_without_deleting_others(tmpdir):
    path = os.path.join(str(tmpdir), "ik_seed_poses.yaml")
    document = default_ik_seed_poses_document()
    document["ik_seed_poses"]["squares"]["b1"]["seed_ticks"] = {
        "shoulder_pan": 1074,
    }
    _write_seed_file(path, document)

    loaded = load_ik_seed_poses(path)
    serializable = save_current_ik_seed_pose_cli.build_serializable_document(loaded)
    updated = upsert_square_seed_entry(
        serializable,
        "a1",
        {
            "shoulder_pan": 1074,
            "shoulder_lift": 1776,
            "elbow_flex": 2321,
            "wrist_flex": 2619,
            "wrist_roll": 1091,
        },
        notes="Manually taught extended high-clearance posture for a1.",
    )
    save_current_ik_seed_pose_cli.save_ik_seed_poses(path, updated)

    saved = load_ik_seed_poses(path)
    assert saved["squares"]["a1"]["seed_ticks"]["shoulder_pan"] == 1074
    assert saved["squares"]["a1"]["notes"] == "Manually taught extended high-clearance posture for a1."
    assert saved["squares"]["b1"]["seed_ticks"]["shoulder_pan"] == 1074
