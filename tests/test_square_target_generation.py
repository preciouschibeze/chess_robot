from __future__ import absolute_import

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map


JOINT_ORDER = list(robot_square_map.DEFAULT_JOINT_ORDER)


def _joint_limits(minimum=0, maximum=4095):
    limits = {}
    for joint_name in JOINT_ORDER:
        limits[joint_name] = {
            "provisional_min": minimum,
            "provisional_max": maximum,
        }
    return limits


def _manual_pose(base_value):
    joints = {}
    for index, joint_name in enumerate(JOINT_ORDER):
        joints[joint_name] = base_value + (index * 10)
    return {
        "source": "manual",
        "confidence": "taught",
        "joints": joints,
        "recorded_at": "2026-05-20T00:00:00Z",
        "notes": [],
    }


def _document_with_manual_anchors(anchor_squares, base_value=2000):
    document = robot_square_map.default_square_targets()
    for offset, square_name in enumerate(anchor_squares):
        document["squares"][square_name] = {
            "above_pose": _manual_pose(base_value + offset),
        }
    return document


def test_generated_poses_never_overwrite_manual_poses():
    anchors = ["a8", "c8", "f8", "h8", "a6", "c6", "f6", "a3", "c3"]
    document = _document_with_manual_anchors(anchors)
    result = robot_square_map.generate_square_targets(document, _joint_limits())
    assert result["data"]["squares"]["a8"]["above_pose"]["source"] == "manual"
    assert result["data"]["squares"]["h1"]["above_pose"]["source"] == "generated"


def test_fewer_than_nine_manual_anchors_refuses_write_ready():
    anchors = ["a8", "c8", "f8", "h8", "a6", "c6", "f6", "a3"]
    document = _document_with_manual_anchors(anchors)
    result = robot_square_map.generate_square_targets(document, _joint_limits())
    assert result["manual_anchor_count"] == 8
    assert result["write_ready"] is False


def test_missing_recommended_anchors_are_reported():
    anchors = ["a8", "c8", "f8", "h8", "a6", "c6", "f6", "a3", "c3"]
    document = _document_with_manual_anchors(anchors)
    result = robot_square_map.generate_square_targets(document, _joint_limits())
    assert "h6" in result["missing_recommended_anchors"]
    assert any("fewer than 16 recommended anchors" in warning for warning in result["warnings"])


def test_joint_limit_violation_is_detected():
    anchors = ["a8", "c8", "f8", "h8", "a6", "c6", "f6", "a3", "c3"]
    document = _document_with_manual_anchors(anchors, base_value=5000)
    result = robot_square_map.generate_square_targets(document, _joint_limits(maximum=2500))
    assert result["generated_validation_errors"]
    assert result["write_ready"] is False


def test_generated_pose_contains_source_and_interpolation_metadata():
    anchors = ["a8", "c8", "f8", "h8", "a6", "c6", "f6", "a3", "c3"]
    document = _document_with_manual_anchors(anchors)
    result = robot_square_map.generate_square_targets(document, _joint_limits())
    pose = result["data"]["squares"]["h1"]["above_pose"]
    assert pose["source"] == "generated"
    assert pose["interpolation"]["method"] == "joint_space_idw"
    assert pose["interpolation"]["anchors_used"]


def test_manual_pose_contains_source_manual():
    pose = robot_square_map.build_manual_above_pose({
        "shoulder_pan": 2000,
        "shoulder_lift": 2100,
        "elbow_flex": 2200,
        "wrist_flex": 2300,
        "wrist_roll": 1200,
        "gripper": 1600,
    })
    assert pose["source"] == "manual"
