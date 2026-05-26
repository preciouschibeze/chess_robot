from __future__ import absolute_import

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import tools.teleop_joints_keyboard as teleop


def _records():
    return {
        "base_yaw": {
            "user_joint": "base_yaw",
            "urdf_joint": "shoulder_pan",
            "servo_id": 1,
            "limits": {"min": 900, "max": 1100},
            "calibrated": True,
            "joggable": True,
            "reason": None,
        },
        "wrist_roll": {
            "user_joint": "wrist_roll",
            "urdf_joint": "wrist_roll",
            "servo_id": 5,
            "limits": {"min": 900, "max": 1100},
            "calibrated": True,
            "joggable": True,
            "reason": None,
        },
        "gripper": {
            "user_joint": "gripper",
            "urdf_joint": "gripper",
            "servo_id": 6,
            "limits": None,
            "calibrated": False,
            "joggable": False,
            "reason": "missing joint calibration",
        },
    }


def test_command_parser_accepts_number_select_and_delta():
    command = teleop.parse_operator_command(
        "5",
        active_joint="base_yaw",
        current_step=10,
        allowed=["base_yaw", "wrist_roll"],
        max_step_ticks=100,
    )
    assert command == {"action": "select", "joint": "wrist_roll"}

    command = teleop.parse_operator_command(
        "+5",
        active_joint="wrist_roll",
        current_step=5,
        allowed=["base_yaw", "wrist_roll"],
        max_step_ticks=100,
    )
    assert command == {"action": "jog", "joint": "wrist_roll", "delta": 5}


def test_command_parser_rejects_delta_above_current_step():
    with pytest.raises(teleop.CommandRejected):
        teleop.parse_operator_command(
            "+11",
            active_joint="base_yaw",
            current_step=10,
            allowed=["base_yaw"],
            max_step_ticks=100,
        )


def test_step_size_clamping():
    assert teleop.clamp_step_ticks(0, 100) == 1
    assert teleop.clamp_step_ticks(25, 100) == 25
    assert teleop.clamp_step_ticks(250, 100) == 100

    command = teleop.parse_operator_command(
        "step 250",
        active_joint="base_yaw",
        current_step=10,
        allowed=["base_yaw"],
        max_step_ticks=100,
    )
    assert command == {"action": "set_step", "step": 100}


def test_gripper_blocked_unless_allowed():
    with pytest.raises(teleop.CommandRejected):
        teleop.parse_operator_command(
            "6",
            active_joint="base_yaw",
            current_step=10,
            allowed=["base_yaw"],
            max_step_ticks=100,
        )

    command = teleop.parse_operator_command(
        "6",
        active_joint="base_yaw",
        current_step=10,
        allowed=["base_yaw", "gripper"],
        max_step_ticks=100,
    )
    assert command == {"action": "select", "joint": "gripper"}


def test_safety_rejection_for_target_outside_limits():
    validation = teleop.validate_jog_request(
        records=_records(),
        current_ticks={"base_yaw": 1095},
        active_joint="base_yaw",
        delta=10,
        max_delta=10,
    )
    assert validation["ok"] is False
    assert "outside limits" in validation["reason"]


def test_gripper_jog_rejected_when_missing_calibration():
    validation = teleop.validate_jog_request(
        records=_records(),
        current_ticks={"gripper": 1000},
        active_joint="gripper",
        delta=5,
        max_delta=10,
    )
    assert validation["ok"] is False
    assert "missing calibration" in validation["reason"]


def test_pose_snapshot_json_structure():
    snapshot = teleop.build_pose_snapshot(
        timestamp="2026-05-26T00:00:00Z",
        joint_ticks={"base_yaw": 1000},
        joint_angles_rad={"base_yaw": 0.1},
        tcp_frame="gripper_frame",
        notes=["test note"],
        transform=[[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
        tool_frame={"tcp_frame": "gripper_frame"},
        mode="dry_run",
    )
    assert snapshot["timestamp"] == "2026-05-26T00:00:00Z"
    assert snapshot["joint_ticks"] == {"base_yaw": 1000}
    assert snapshot["joint_angles_rad"] == {"base_yaw": 0.1}
    assert snapshot["tcp_frame"] == "gripper_frame"
    assert snapshot["T_base_gripper"][3][3] == 1.0
    assert snapshot["notes"] == ["test note"]
    assert snapshot["tool_frame"] == {"tcp_frame": "gripper_frame"}
