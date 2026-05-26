from __future__ import absolute_import

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
TOOLS_ROOT = os.path.join(ROOT, "tools")
if TOOLS_ROOT not in sys.path:
    sys.path.insert(0, TOOLS_ROOT)

from chess_robot.robot import safe_recovery
from chess_robot.robot import safe_transfer
from chess_robot.robot.ik_validation import ARM_JOINTS
from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_pose_ticks
import recover_home as recover_home_cli


SCENE_PATH = os.path.join(ROOT, "data", "calibration", "robot", "scene_geometry.yaml")
URDF_PATH = os.path.join(ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
JOINT_CALIBRATION_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
JOINT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
JOINT_SAFETY_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")
HOME_POSE_PATH = os.path.join(ROOT, "data", "calibration", "robot", "home_pose.yaml")
TOOL_FRAMES_PATH = os.path.join(ROOT, "data", "calibration", "gripper", "tool_frames.yaml")
APPROACH_POLICY_PATH = os.path.join(ROOT, "data", "calibration", "robot", "approach_policy.yaml")


def _args(tmpdir, extra=None):
    output = os.path.join(str(tmpdir), "recover_home.json")
    values = [
        "--urdf", URDF_PATH,
        "--scene", SCENE_PATH,
        "--joint-calibration", JOINT_CALIBRATION_PATH,
        "--joint-limits", JOINT_LIMITS_PATH,
        "--joint-safety-limits", JOINT_SAFETY_LIMITS_PATH,
        "--home-pose", HOME_POSE_PATH,
        "--tool-frames", TOOL_FRAMES_PATH,
        "--tcp-frame", "gripper_frame",
        "--approach-policy", APPROACH_POLICY_PATH,
        "--output", output,
    ]
    values.extend(list(extra or []))
    return recover_home_cli.build_parser().parse_args(values)


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


def test_recovery_planner_contains_expected_minimum_sequence(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    log = safe_recovery.run_safe_recovery(
        _args(tmpdir, ["--recovery-route-squares", "e4"]),
        bus_factory=_bus_factory(FakeBus()),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert [segment["segment_name"] for segment in log["segments"]] == [
        "current_safe_lift",
        "recovery_route_high_e4",
        "home_high",
        "home_pose",
    ]


def test_recovery_route_override_wins_over_policy(tmpdir):
    args = _args(tmpdir, ["--recovery-route-squares", "a2,c3"])
    assert safe_recovery.resolve_recovery_route_squares(args, default_route_squares=["e4"]) == ["a2", "c3"]


def test_recovery_execute_requires_confirmation(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    bus = FakeBus(update_on_write=True)
    log = safe_recovery.run_safe_recovery(
        _args(tmpdir, ["--execute"]),
        bus_factory=_bus_factory(bus),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
        sleep_fn=lambda seconds: None,
    )
    assert "requires --confirm" in log["abort_reason"]
    assert log["command_sent_any"] is False
    assert log["segments"] == []


def test_recovery_dry_run_sends_no_commands(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    bus = FakeBus(update_on_write=True)
    log = safe_recovery.run_safe_recovery(
        _args(tmpdir, ["--recovery-route-squares", "e4"]),
        bus_factory=_bus_factory(bus),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert log["mode"] == "dry_run"
    assert log["command_sent_any"] is False
    assert [segment["command_sent"] for segment in log["segments"]] == [False, False, False, False]


def test_recovery_readback_failure_marks_unavailable(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    bus = FakeBus(update_on_write=True)
    bus.positions.pop(1, None)
    log = safe_recovery.run_safe_recovery(
        _args(tmpdir),
        bus_factory=_bus_factory(bus),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert log["recovery_available"] is False
    assert log["abort_reason"] == safe_recovery.RECOVERY_READBACK_FAILURE_MESSAGE


def test_recovery_validates_every_segment(tmpdir, monkeypatch):
    calls = {"count": 0}

    def counting_path(*args, **kwargs):
        calls["count"] += 1
        return _passing_path(*args, **kwargs)

    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", counting_path)
    log = safe_recovery.run_safe_recovery(
        _args(tmpdir, ["--recovery-route-squares", "e4"]),
        bus_factory=_bus_factory(FakeBus()),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert [segment["segment_name"] for segment in log["segments"]] == [
        "current_safe_lift",
        "recovery_route_high_e4",
        "home_high",
        "home_pose",
    ]
    assert calls["count"] == 4


def test_recovery_aborts_when_path_validation_fails(tmpdir, monkeypatch):
    calls = {"count": 0}

    def third_segment_fails(*args, **kwargs):
        calls["count"] += 1
        summary = _passing_path(*args, **kwargs)
        if calls["count"] == 3:
            summary["passed"] = False
            summary["failure_reason"] = "synthetic recovery path failure"
        return summary

    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", third_segment_fails)
    log = safe_recovery.run_safe_recovery(
        _args(tmpdir, ["--recovery-route-squares", "e4"]),
        bus_factory=_bus_factory(FakeBus()),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert log["aborted"] is True
    assert log["abort_reason"] == "synthetic recovery path failure"
    assert [segment["segment_name"] for segment in log["segments"]] == [
        "current_safe_lift",
        "recovery_route_high_e4",
        "home_high",
    ]
