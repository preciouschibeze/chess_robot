from __future__ import absolute_import

import importlib.util
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
TOOLS_ROOT = os.path.join(ROOT, "tools")
if TOOLS_ROOT not in sys.path:
    sys.path.insert(0, TOOLS_ROOT)

from chess_robot.robot import safe_transfer
from chess_robot.robot.ik_validation import ARM_JOINTS
from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_pose_ticks


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


def _args(tmpdir, extra=None):
    output = os.path.join(str(tmpdir), "safe_transfer.json")
    csv_log = os.path.join(str(tmpdir), "safe_transfer.csv")
    values = [
        "--urdf", URDF_PATH,
        "--scene", SCENE_PATH,
        "--joint-calibration", JOINT_CALIBRATION_PATH,
        "--joint-limits", JOINT_LIMITS_PATH,
        "--joint-safety-limits", JOINT_SAFETY_LIMITS_PATH,
        "--home-pose", HOME_POSE_PATH,
        "--tool-frames", TOOL_FRAMES_PATH,
        "--tcp-frame", "gripper_frame",
        "--square", "e4",
        "--workspace-seed-samples", "0",
        "--output", output,
        "--csv-log", csv_log,
    ]
    values.extend(list(extra or []))
    return safe_square_transfer_cli.build_parser().parse_args(values)


def _home_ticks():
    return dict((joint, int(value)) for joint, value in load_pose_ticks(HOME_POSE_PATH).items())


def _home_arm_ticks():
    ticks = _home_ticks()
    return dict((joint, int(ticks[joint])) for joint in ARM_JOINTS)


def _radians_from_ticks(ticks):
    calibration = load_joint_calibration(JOINT_CALIBRATION_PATH)
    return convert_pose_ticks_to_urdf_radians(ticks, calibration)


class FakeIKResult(object):
    def __init__(self, target_robot, joint_positions_rad=None, success=True):
        self.success = bool(success)
        self.status = "success" if success else "max_iters"
        self.final_xyz_robot = np.asarray(target_robot, dtype=float)
        self.error_m = 0.001 if success else 0.100
        self.iterations = 3
        self.joint_positions_rad = dict(joint_positions_rad or _radians_from_ticks(_home_ticks()))


def _home_solver(*args, **kwargs):
    del kwargs
    return FakeIKResult(args[1])


def _shifted_solver(*args, **kwargs):
    del kwargs
    ticks = _home_ticks()
    ticks["shoulder_pan"] = int(ticks["shoulder_pan"]) + 150
    return FakeIKResult(args[1], _radians_from_ticks(ticks))


def _passing_path(*args, **kwargs):
    del kwargs
    low_zone = float(args[7])
    samples_count = int(args[9])
    return {
        "xy_delta_m": 0.010,
        "min_z_m": low_zone + 0.050,
        "low_zone_z_m": low_zone,
        "passed": True,
        "failure_reason": None,
        "samples_count": samples_count,
        "xy_changing": True,
        "current_tcp_world_xyz_m": [0.0, 0.0, low_zone + 0.050],
        "target_tcp_world_xyz_m": [0.01, 0.0, low_zone + 0.050],
    }


class FakeBus(object):
    def __init__(self, positions=None, update_on_write=True):
        base = _home_arm_ticks()
        self.positions = dict((index + 1, int(base[joint])) for index, joint in enumerate(ARM_JOINTS))
        if positions:
            self.positions.update(dict(positions))
        self.update_on_write = bool(update_on_write)
        self.writes = []
        self.closed = False

    def read_position(self, servo_id):
        return self.positions.get(int(servo_id))

    def read_register(self, servo_id, address, length):
        del servo_id, length
        if int(address) == 9:
            return 0
        if int(address) == 11:
            return 4095
        return None

    def write_goal_position(self, servo_id, goal_position):
        self.writes.append((int(servo_id), int(goal_position)))
        if self.update_on_write:
            self.positions[int(servo_id)] = int(goal_position)

    def close(self):
        self.closed = True


def _bus_factory(bus):
    def factory(args):
        del args
        config = {
            "joints": {
                "shoulder_pan": {"servo_id": 1},
                "shoulder_lift": {"servo_id": 2},
                "elbow_flex": {"servo_id": 3},
                "wrist_flex": {"servo_id": 4},
                "wrist_roll": {"servo_id": 5},
                "gripper": {"servo_id": 6},
            }
        }
        return bus, config
    return factory


def test_staged_plan_contains_forward_segments():
    args = _args("/tmp")
    plan = safe_transfer.build_staged_plan(
        [0.0, 0.0, 0.050],
        [0.2, 0.1, 0.026],
        [0.0, 0.0, 0.050],
        0.026,
        args,
    )
    assert [item["segment_name"] for item in plan] == ["current_lift", "target_high_above", "target_normal_above"]


def test_return_home_plan_adds_return_segments():
    args = _args("/tmp", ["--return-home"])
    plan = safe_transfer.build_staged_plan(
        [0.0, 0.0, 0.050],
        [0.2, 0.1, 0.026],
        [0.0, 0.0, 0.050],
        0.026,
        args,
    )
    assert [item["segment_name"] for item in plan][-3:] == ["target_high_above_return", "home_high", "home_pose"]


def test_target_offsets_are_used_in_plan():
    args = _args("/tmp", ["--normal-above-offset-m", "0.081", "--high-above-offset-m", "0.123"])
    plan = safe_transfer.build_staged_plan(
        [0.0, 0.0, 0.050],
        [0.2, 0.1, 0.026],
        [0.0, 0.0, 0.050],
        0.026,
        args,
    )
    assert plan[1]["target_world_xyz_m"][2] == 0.149
    assert plan[2]["target_world_xyz_m"][2] == 0.107


def test_dry_run_never_commands_hardware(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)

    def forbidden_bus_factory(args):
        del args
        raise AssertionError("dry-run must not open hardware")

    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir),
        bus_factory=forbidden_bus_factory,
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert log["mode"] == "dry_run"
    assert log["command_sent_any"] is False
    assert [segment["command_sent"] for segment in log["segments"]] == [False, False, False]


def test_execute_without_confirm_aborts_before_hardware(tmpdir):
    def forbidden_bus_factory(args):
        del args
        raise AssertionError("confirmation failure must not open hardware")

    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--execute"]),
        bus_factory=forbidden_bus_factory,
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert "requires --confirm" in log["abort_reason"]
    assert log["segments"] == []
    assert log["command_sent_any"] is False


def test_segment_execution_stops_after_failed_validation(tmpdir, monkeypatch):
    calls = {"count": 0}

    def second_segment_fails(*args, **kwargs):
        calls["count"] += 1
        summary = _passing_path(*args, **kwargs)
        if calls["count"] == 2:
            summary["passed"] = False
            summary["failure_reason"] = "synthetic unsafe path"
        return summary

    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", second_segment_fails)
    bus = FakeBus(update_on_write=True)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--execute", "--confirm", safe_transfer.CONFIRM_TEXT]),
        bus_factory=_bus_factory(bus),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
        sleep_fn=lambda seconds: None,
    )
    assert log["abort_reason"] == "synthetic unsafe path"
    assert [segment["segment_name"] for segment in log["segments"]] == ["current_lift", "target_high_above"]
    assert log["segments"][0]["command_sent"] is True
    assert log["segments"][1]["command_sent"] is False


def test_segment_execution_stops_after_failed_readback_tolerance(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    bus = FakeBus(update_on_write=False)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--execute", "--confirm", safe_transfer.CONFIRM_TEXT, "--readback-tolerance-ticks", "10"]),
        bus_factory=_bus_factory(bus),
        ik_solver=_shifted_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
        sleep_fn=lambda seconds: None,
    )
    assert "outside target" in log["abort_reason"]
    assert [segment["segment_name"] for segment in log["segments"]] == ["current_lift"]
    assert log["segments"][0]["command_sent"] is True


def test_wrist_roll_is_locked_by_default(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert "wrist_roll" in log["locked_joints"]


def test_no_lock_wrist_roll_disables_default_lock(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--no-lock-wrist-roll"]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert "wrist_roll" not in log["locked_joints"]


def test_path_validation_is_called_for_every_segment(tmpdir, monkeypatch):
    calls = {"count": 0}

    def counting_path(*args, **kwargs):
        calls["count"] += 1
        return _passing_path(*args, **kwargs)

    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", counting_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert len(log["segments"]) == 3
    assert calls["count"] == 3


def test_gripper_is_excluded_from_all_target_ticks(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    for segment in log["segments"]:
        assert "gripper" not in segment["target_ticks"]


def test_output_json_contains_required_segment_fields(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    args = _args(tmpdir)
    log = safe_transfer.run_safe_square_transfer(
        args,
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    with open(args.output, "r") as handle:
        saved = json.load(handle)
    assert saved["segments"]
    required = [
        "segment_index",
        "segment_name",
        "target_world_xyz_m",
        "target_robot_xyz_m",
        "ik_success",
        "ik_status",
        "ik_error_m",
        "ik_iterations",
        "final_tcp_world_xyz_m",
        "final_tcp_robot_xyz_m",
        "target_ticks",
        "current_ticks_before",
        "final_ticks_after",
        "motion_deltas_ticks",
        "readback_errors_ticks",
        "safety_checks",
        "path_validation",
        "approach_tilt_deg",
        "command_sent",
        "abort_reason",
    ]
    for key in required:
        assert key in saved["segments"][0]
    assert log["segments"][0]["ik_success"] is True
