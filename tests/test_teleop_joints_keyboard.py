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
        pose_id=1,
        session_id="session-1",
        timestamp_iso="2026-05-26T00:00:00.123Z",
        timestamp_unix=1780000000.123,
        joint_ticks={"base_yaw": 1000},
        joint_angles_rad={"base_yaw": 0.1},
        tcp_frame="gripper_frame",
        notes=["test note"],
        transform=[[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
        tool_frame={"tcp_frame": "gripper_frame"},
        mode="dry_run",
        source="keyboard_teleop_hotkey",
    )
    assert snapshot["pose_id"] == 1
    assert snapshot["session_id"] == "session-1"
    assert snapshot["timestamp_iso"] == "2026-05-26T00:00:00.123Z"
    assert snapshot["timestamp"] == "2026-05-26T00:00:00.123Z"
    assert snapshot["timestamp_unix"] == 1780000000.123
    assert snapshot["joint_ticks"] == {"base_yaw": 1000}
    assert snapshot["joint_angles_rad"] == {"base_yaw": 0.1}
    assert snapshot["tcp_frame"] == "gripper_frame"
    assert snapshot["T_base_gripper"][3][3] == 1.0
    assert snapshot["notes"] == ["test note"]
    assert snapshot["tool_frame"] == {"tcp_frame": "gripper_frame"}
    assert snapshot["source"] == "keyboard_teleop_hotkey"



def test_hotkey_mapping_required_joints():
    assert teleop.parse_hotkey("a", allow_gripper=False) == {
        "action": "jog", "joint": "base_yaw", "direction": -1, "key": "a"
    }
    assert teleop.parse_hotkey("d", allow_gripper=False)["direction"] == 1
    assert teleop.parse_hotkey("z", allow_gripper=False)["joint"] == "shoulder_pitch"
    assert teleop.parse_hotkey("x", allow_gripper=False)["direction"] == 1
    assert teleop.parse_hotkey("c", allow_gripper=False)["joint"] == "elbow_pitch"
    assert teleop.parse_hotkey("v", allow_gripper=False)["direction"] == 1
    assert teleop.parse_hotkey("b", allow_gripper=False)["joint"] == "wrist_pitch"
    assert teleop.parse_hotkey("n", allow_gripper=False)["direction"] == 1
    assert teleop.parse_hotkey("j", allow_gripper=False)["joint"] == "wrist_roll"
    assert teleop.parse_hotkey("k", allow_gripper=False)["direction"] == 1
    assert teleop.parse_hotkey("s", allow_gripper=False) == {"action": "save", "notes": ""}
    assert teleop.parse_hotkey(" ", allow_gripper=False) == {"action": "read"}
    assert teleop.parse_hotkey("q", allow_gripper=False) == {"action": "quit"}


def test_save_hotkey_does_not_request_exit():
    command = teleop.parse_hotkey("s", allow_gripper=False)
    assert command["action"] == "save"
    assert command["action"] != "quit"


def test_pose_id_increments():
    session = teleop.TeleopSession("hotkey", session_id="fixed-session")
    assert session.next_pose_id() == 1
    assert session.next_pose_id() == 2
    assert session.next_pose_id() == 3


def test_saved_filename_uniqueness(tmpdir):
    path1, _ts1, _unix1 = teleop.next_pose_output_path(str(tmpdir), 1)
    with open(path1, "w") as handle:
        handle.write("{}\n")
    path2, _ts2, _unix2 = teleop.next_pose_output_path(str(tmpdir), 1)
    assert path1 != path2
    assert os.path.basename(path1).startswith("pose_0001_")
    assert os.path.basename(path2).startswith("pose_0001_")
    assert path1.endswith(".json")
    assert path2.endswith(".json")


def test_rate_limit_logic():
    assert teleop.is_rate_limited(10.0, None, 0.15) is False
    assert teleop.is_rate_limited(10.10, 10.0, 0.15) is True
    assert teleop.is_rate_limited(10.20, 10.0, 0.15) is False
    assert teleop.action_is_rate_limited("jog") is True
    assert teleop.action_is_rate_limited("quit") is False


def test_gripper_hotkeys_ignored_unless_allow_gripper():
    blocked = teleop.parse_hotkey("u", allow_gripper=False)
    assert blocked["action"] == "ignored"
    assert "--allow-gripper" in blocked["reason"]

    allowed = teleop.parse_hotkey("i", allow_gripper=True)
    assert allowed == {"action": "jog", "joint": "gripper", "direction": 1, "key": "i"}


def test_line_mode_still_parses_existing_commands():
    command = teleop.parse_operator_command(
        "select wrist_roll",
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



def test_hotkey_save_pose_log_uses_non_conflicting_pose_path(tmpdir):
    log_path = str(tmpdir.join("keyboard_teleop.log"))
    pose_path = str(tmpdir.join("pose_0001_20260526_000000_000.json"))
    teleop.append_log(
        log_path,
        "save_pose",
        pose_path=pose_path,
        pose_id=1,
        session_id="session-1",
        source="keyboard_teleop_hotkey",
        dry_run=True,
    )
    content = tmpdir.join("keyboard_teleop.log").read()
    assert '"event": "save_pose"' in content
    assert '"pose_path":' in content
    assert pose_path in content


def test_save_hotkey_saves_and_continues_action():
    command = teleop.parse_hotkey("s", allow_gripper=False)
    assert command == {"action": "save", "notes": ""}
    assert command["action"] != "quit"
