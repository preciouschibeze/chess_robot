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
from chess_robot.robot.ik_seed_poses import default_ik_seed_poses_document
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
APPROACH_POLICY_PATH = os.path.join(ROOT, "data", "calibration", "robot", "approach_policy.yaml")

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
        "--stop-at", "normal_above",
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


def _route_targets(route_squares, route_z):
    targets = []
    for square_name in route_squares:
        targets.append({
            "square": str(square_name).lower(),
            "target_world_xyz_m": [0.0, 0.0, float(route_z)],
        })
    return targets


def _write_seed_file(path, document):
    import yaml

    with open(path, "w") as handle:
        yaml.safe_dump(document, handle, default_flow_style=False)


def _seed_path(tmpdir, square, seed_ticks):
    path = os.path.join(str(tmpdir), "%s_ik_seed_poses.yaml" % square)
    document = default_ik_seed_poses_document()
    document["ik_seed_poses"]["squares"].setdefault(square, {"notes": None, "seed_ticks": {}})
    document["ik_seed_poses"]["squares"][square]["seed_ticks"] = dict(seed_ticks)
    _write_seed_file(path, document)
    return path


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


def test_cli_default_stop_at_is_high_above():
    args = safe_square_transfer_cli.build_parser().parse_args([
        "--square", "e4",
        "--output", "/tmp/safe_transfer_default.json",
    ])
    assert args.stop_at == "high_above"


def test_stop_at_high_above_plan_stops_before_normal_above():
    args = _args("/tmp", ["--stop-at", "high_above"])
    plan = safe_transfer.build_staged_plan(
        [0.0, 0.0, 0.050],
        [0.2, 0.1, 0.026],
        [0.0, 0.0, 0.050],
        0.026,
        args,
    )
    assert [item["segment_name"] for item in plan] == ["current_lift", "target_high_above"]


def test_stop_at_return_home_plan_creates_full_return_sequence():
    args = _args("/tmp", ["--stop-at", "return_home"])
    plan = safe_transfer.build_staged_plan(
        [0.0, 0.0, 0.050],
        [0.2, 0.1, 0.026],
        [0.0, 0.0, 0.050],
        0.026,
        args,
    )
    assert [item["segment_name"] for item in plan] == [
        "current_lift",
        "target_high_above",
        "target_normal_above",
        "target_high_above_return",
        "home_high",
        "home_pose",
    ]


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


def test_return_home_plan_marks_reverse_replay_sources():
    args = _args("/tmp", ["--return-home"])
    plan = safe_transfer.build_staged_plan(
        [0.0, 0.0, 0.050],
        [0.2, 0.1, 0.026],
        [0.0, 0.0, 0.050],
        0.026,
        args,
    )
    by_name = dict((item["segment_name"], item) for item in plan)
    assert by_name["target_high_above_return"]["replay_source_segment"] == "target_high_above"
    assert by_name["home_high"]["replay_source_segment"] == "current_lift"
    assert by_name["home_pose"]["is_return_segment"] is True


def test_return_strategy_defaults_to_achieved_reverse_replay():
    args = _args("/tmp")
    assert args.return_strategy == safe_transfer.RETURN_STRATEGY_ACHIEVED_REVERSE_REPLAY


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


def test_return_home_plan_inserts_route_segments_before_home_high():
    args = _args("/tmp", ["--return-home", "--return-route-squares", "a2,c3,e4", "--route-above-offset-m", "0.140"])
    plan = safe_transfer.build_staged_plan(
        [0.0, 0.0, 0.050],
        [0.2, 0.1, 0.026],
        [0.0, 0.0, 0.050],
        0.026,
        args,
        return_route_targets=_route_targets(args.return_route_squares, 0.166),
    )
    assert [item["segment_name"] for item in plan] == [
        "current_lift",
        "target_high_above",
        "target_normal_above",
        "target_high_above_return",
        "route_high_a2",
        "route_high_c3",
        "route_high_e4",
        "home_high",
        "home_pose",
    ]


def test_route_segments_are_high_above_only():
    args = _args("/tmp", ["--return-home", "--return-route-squares", "a2,c3", "--route-above-offset-m", "0.140"])
    plan = safe_transfer.build_staged_plan(
        [0.0, 0.0, 0.050],
        [0.2, 0.1, 0.026],
        [0.0, 0.0, 0.050],
        0.026,
        args,
        return_route_targets=_route_targets(args.return_route_squares, 0.166),
    )
    route_segments = [item for item in plan if item["segment_name"].startswith("route_high_")]
    assert [item["route_square"] for item in route_segments] == ["a2", "c3"]
    assert [item["route_waypoint"] for item in route_segments] == [True, True]
    assert [item["target_world_xyz_m"][2] for item in route_segments] == [0.166, 0.166]
    assert not [item for item in plan if "route_normal" in item["segment_name"]]


def test_return_route_squares_are_ignored_for_high_above_without_return_home():
    args = _args("/tmp", ["--stop-at", "high_above", "--return-route-squares", "a2,c3", "--route-above-offset-m", "0.140"])
    plan = safe_transfer.build_staged_plan(
        [0.0, 0.0, 0.050],
        [0.2, 0.1, 0.026],
        [0.0, 0.0, 0.050],
        0.026,
        args,
        return_route_targets=_route_targets(["a2", "c3"], 0.166),
    )
    assert [item["segment_name"] for item in plan] == ["current_lift", "target_high_above"]


def test_old_settle_time_behavior_remains_for_all_segments(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--return-home", "--settle-time-s", "1.25"]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert [segment["settle_time_s"] for segment in log["segments"]] == [1.25, 1.25, 1.25, 1.25, 1.25, 1.25]


def test_split_settle_times_apply_to_intermediate_and_final_segments(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, [
            "--return-home",
            "--intermediate-settle-time-s", "0.5",
            "--final-settle-time-s", "1.5",
        ]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    by_name = dict((segment["segment_name"], segment["settle_time_s"]) for segment in log["segments"])
    assert by_name["current_lift"] == 0.5
    assert by_name["target_high_above"] == 0.5
    assert by_name["target_high_above_return"] == 0.5
    assert by_name["home_high"] == 0.5
    assert by_name["target_normal_above"] == 1.5
    assert by_name["home_pose"] == 1.5

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


def test_stop_at_high_above_runs_path_validation_for_two_segments(tmpdir, monkeypatch):
    calls = {"count": 0}

    def counting_path(*args, **kwargs):
        calls["count"] += 1
        return _passing_path(*args, **kwargs)

    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", counting_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--stop-at", "high_above"]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert [segment["segment_name"] for segment in log["segments"]] == ["current_lift", "target_high_above"]
    assert calls["count"] == 2


def test_route_segments_run_path_validation(tmpdir, monkeypatch):
    calls = {"count": 0}

    def counting_path(*args, **kwargs):
        calls["count"] += 1
        return _passing_path(*args, **kwargs)

    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", counting_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--return-home", "--return-route-squares", "a2,c3", "--route-above-offset-m", "0.140"]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    by_name = dict((segment["segment_name"], segment) for segment in log["segments"])
    assert len(log["segments"]) == 8
    assert calls["count"] == 8
    assert by_name["route_high_a2"]["route_waypoint"] is True
    assert by_name["route_high_a2"]["path_validation"]["passed"] is True
    assert by_name["route_high_c3"]["route_waypoint"] is True
    assert by_name["route_high_c3"]["path_validation"]["passed"] is True


def test_route_segment_abort_stops_later_segments(tmpdir, monkeypatch):
    calls = {"count": 0}

    def fifth_segment_fails(*args, **kwargs):
        calls["count"] += 1
        summary = _passing_path(*args, **kwargs)
        if calls["count"] == 5:
            summary["passed"] = False
            summary["failure_reason"] = "synthetic route path failure"
        return summary

    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", fifth_segment_fails)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--return-home", "--return-route-squares", "a2", "--route-above-offset-m", "0.140"]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert calls["count"] == 5
    assert log["abort_reason"] == "synthetic route path failure"
    assert [segment["segment_name"] for segment in log["segments"]] == [
        "current_lift",
        "target_high_above",
        "target_normal_above",
        "target_high_above_return",
        "route_high_a2",
    ]
    assert log["segments"][-1]["route_waypoint"] is True


def test_reverse_replay_reuses_forward_targets_and_skips_return_ik(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    calls = []

    def recording_solver(*args, **kwargs):
        del kwargs
        call_index = len(calls) + 1
        ticks = _home_ticks()
        ticks["shoulder_pan"] = int(ticks["shoulder_pan"]) + (call_index * 10)
        ticks["shoulder_lift"] = int(ticks["shoulder_lift"]) + (call_index * 20)
        calls.append(call_index)
        return FakeIKResult(args[1], joint_positions_rad=_radians_from_ticks(ticks))

    args = _args(tmpdir, ["--return-home", "--return-strategy", safe_transfer.RETURN_STRATEGY_REVERSE_REPLAY])
    log = safe_transfer.run_safe_square_transfer(
        args,
        ik_solver=recording_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    with open(args.output, "r") as handle:
        saved = json.load(handle)
    by_name = dict((segment["segment_name"], segment) for segment in log["segments"])
    saved_by_name = dict((segment["segment_name"], segment) for segment in saved["segments"])
    assert len(calls) == 3
    assert by_name["target_high_above_return"]["target_ticks"] == by_name["target_high_above"]["target_ticks"]
    assert by_name["home_high"]["target_ticks"] == by_name["current_lift"]["target_ticks"]
    assert by_name["target_high_above_return"]["replay_source_segment"] == "target_high_above"
    assert by_name["home_high"]["replay_source_segment"] == "current_lift"
    assert by_name["target_high_above_return"]["return_strategy"] == safe_transfer.RETURN_STRATEGY_REVERSE_REPLAY
    assert by_name["home_high"]["return_strategy"] == safe_transfer.RETURN_STRATEGY_REVERSE_REPLAY
    assert by_name["target_high_above_return"]["replayed_target_ticks"] is True
    assert by_name["home_high"]["replayed_target_ticks"] is True
    assert by_name["target_high_above_return"]["replay_source"] == "planned_target_ticks"
    assert by_name["home_high"]["replay_source"] == "planned_target_ticks"
    assert by_name["target_high_above_return"]["ik_status"] == "replayed_forward_target"
    assert by_name["home_high"]["ik_status"] == "replayed_forward_target"
    assert by_name["target_high_above_return"]["ik_iterations"] == 0
    assert by_name["home_high"]["ik_iterations"] == 0
    assert by_name["target_high_above_return"]["ik_seed_source"] == "not_applicable"
    assert by_name["home_high"]["ik_seed_source"] == "not_applicable"
    assert by_name["home_pose"]["target_ticks"] == _home_arm_ticks()
    assert saved["return_strategy"] == safe_transfer.RETURN_STRATEGY_REVERSE_REPLAY
    assert saved_by_name["target_high_above_return"]["replay_source_segment"] == "target_high_above"
    assert saved_by_name["home_high"]["return_strategy"] == safe_transfer.RETURN_STRATEGY_REVERSE_REPLAY



def test_resolve_new_preserves_return_ik_solving(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    calls = []

    def recording_solver(*args, **kwargs):
        calls.append(dict(kwargs))
        return FakeIKResult(args[1])

    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--return-home", "--return-strategy", safe_transfer.RETURN_STRATEGY_RESOLVE_NEW]),
        ik_solver=recording_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    by_name = dict((segment["segment_name"], segment) for segment in log["segments"])
    assert len(calls) == 5
    assert by_name["target_high_above_return"]["replayed_target_ticks"] is False
    assert by_name["home_high"]["replayed_target_ticks"] is False
    assert by_name["target_high_above_return"]["ik_status"] != "replayed_forward_target"
    assert by_name["home_high"]["ik_status"] != "replayed_forward_target"



def test_achieved_reverse_replay_execute_uses_achieved_ticks_and_skips_return_ik(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    calls = []

    def recording_solver(*args, **kwargs):
        del kwargs
        call_index = len(calls) + 1
        ticks = _home_ticks()
        ticks["shoulder_pan"] = int(ticks["shoulder_pan"]) + (call_index * 10)
        ticks["shoulder_lift"] = int(ticks["shoulder_lift"]) + (call_index * 20)
        calls.append(call_index)
        return FakeIKResult(args[1], joint_positions_rad=_radians_from_ticks(ticks))

    bus = FakeBus(update_on_write=True)
    args = _args(tmpdir, [
        "--return-home",
        "--execute",
        "--confirm", safe_transfer.CONFIRM_TEXT,
        "--return-strategy", safe_transfer.RETURN_STRATEGY_ACHIEVED_REVERSE_REPLAY,
    ])
    log = safe_transfer.run_safe_square_transfer(
        args,
        bus_factory=_bus_factory(bus),
        ik_solver=recording_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
        sleep_fn=lambda seconds: None,
    )
    by_name = dict((segment["segment_name"], segment) for segment in log["segments"])
    assert len(calls) == 3
    assert by_name["target_high_above"]["achieved_ticks_available"] is True
    assert by_name["current_lift"]["achieved_ticks_available"] is True
    assert by_name["target_high_above_return"]["target_ticks"] == by_name["target_high_above"]["achieved_ticks"]
    assert by_name["home_high"]["target_ticks"] == by_name["current_lift"]["achieved_ticks"]
    assert by_name["target_high_above_return"]["replay_source"] == "achieved_readback_ticks"
    assert by_name["home_high"]["replay_source"] == "achieved_readback_ticks"
    assert by_name["target_high_above_return"]["replayed_target_ticks"] is True
    assert by_name["home_high"]["replayed_target_ticks"] is True
    assert by_name["target_high_above_return"]["ik_status"] == "replayed_forward_target"
    assert by_name["home_high"]["ik_status"] == "replayed_forward_target"



def test_achieved_reverse_replay_dry_run_uses_planned_source_label(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--return-home", "--return-strategy", safe_transfer.RETURN_STRATEGY_ACHIEVED_REVERSE_REPLAY]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    by_name = dict((segment["segment_name"], segment) for segment in log["segments"])
    assert by_name["target_high_above_return"]["replay_source"] == "planned_target_ticks_dry_run"
    assert by_name["home_high"]["replay_source"] == "planned_target_ticks_dry_run"
    assert by_name["target_high_above_return"]["target_ticks"] == by_name["target_high_above"]["planned_target_ticks"]
    assert by_name["home_high"]["target_ticks"] == by_name["current_lift"]["planned_target_ticks"]


def test_achieved_reverse_replay_still_replays_target_high_before_route(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, [
            "--return-home",
            "--return-route-squares", "a2,c3,e4",
            "--route-above-offset-m", "0.140",
            "--return-strategy", safe_transfer.RETURN_STRATEGY_ACHIEVED_REVERSE_REPLAY,
        ]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    by_name = dict((segment["segment_name"], segment) for segment in log["segments"])
    assert by_name["target_high_above_return"]["replay_source"] == "planned_target_ticks_dry_run"
    assert by_name["target_high_above_return"]["replayed_target_ticks"] is True
    assert by_name["route_high_a2"]["replayed_target_ticks"] is False
    assert by_name["route_high_a2"]["route_waypoint"] is True



def test_achieved_reverse_replay_aborts_if_achieved_ticks_are_missing(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    original_execute_segment = safe_transfer.execute_segment

    def execute_without_achieved(segment, current_ticks, bus, servo_ids, args, sleep_fn):
        result = original_execute_segment(segment, current_ticks, bus, servo_ids, args, sleep_fn)
        if segment["segment_name"] in ("current_lift", "target_high_above"):
            segment["achieved_ticks"] = {}
            segment["achieved_ticks_available"] = False
        return result

    monkeypatch.setattr(safe_transfer, "execute_segment", execute_without_achieved)
    bus = FakeBus(update_on_write=True)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, [
            "--return-home",
            "--execute",
            "--confirm", safe_transfer.CONFIRM_TEXT,
            "--return-strategy", safe_transfer.RETURN_STRATEGY_ACHIEVED_REVERSE_REPLAY,
        ]),
        bus_factory=_bus_factory(bus),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
        sleep_fn=lambda seconds: None,
    )
    assert log["segments"][3]["segment_name"] == "target_high_above_return"
    assert "no achieved readback ticks" in log["abort_reason"]
    assert log["segments"][3]["command_sent"] is False



def test_allow_planned_replay_fallback_uses_planned_ticks_when_achieved_missing(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    original_execute_segment = safe_transfer.execute_segment

    def execute_without_achieved(segment, current_ticks, bus, servo_ids, args, sleep_fn):
        result = original_execute_segment(segment, current_ticks, bus, servo_ids, args, sleep_fn)
        if segment["segment_name"] in ("current_lift", "target_high_above"):
            segment["achieved_ticks"] = {}
            segment["achieved_ticks_available"] = False
        return result

    monkeypatch.setattr(safe_transfer, "execute_segment", execute_without_achieved)
    bus = FakeBus(update_on_write=True)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, [
            "--return-home",
            "--execute",
            "--confirm", safe_transfer.CONFIRM_TEXT,
            "--return-strategy", safe_transfer.RETURN_STRATEGY_ACHIEVED_REVERSE_REPLAY,
            "--allow-planned-replay-fallback",
        ]),
        bus_factory=_bus_factory(bus),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
        sleep_fn=lambda seconds: None,
    )
    by_name = dict((segment["segment_name"], segment) for segment in log["segments"])
    assert log["abort_reason"] is None
    assert by_name["target_high_above_return"]["replay_source"] == "planned_target_ticks"
    assert by_name["home_high"]["replay_source"] == "planned_target_ticks"
    assert by_name["target_high_above_return"]["target_ticks"] == by_name["target_high_above"]["planned_target_ticks"]
    assert by_name["home_high"]["target_ticks"] == by_name["current_lift"]["planned_target_ticks"]



def test_achieved_reverse_replay_still_validates_return_paths(tmpdir, monkeypatch):
    calls = {"count": 0}

    def counting_path(*args, **kwargs):
        calls["count"] += 1
        return _passing_path(*args, **kwargs)

    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", counting_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--return-home", "--return-strategy", safe_transfer.RETURN_STRATEGY_ACHIEVED_REVERSE_REPLAY]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    by_name = dict((segment["segment_name"], segment) for segment in log["segments"])
    assert len(log["segments"]) == 6
    assert calls["count"] == 6
    assert by_name["target_high_above_return"]["path_validation"]["passed"] is True
    assert by_name["home_high"]["path_validation"]["passed"] is True



def test_achieved_reverse_replay_path_failure_aborts_before_return_command(tmpdir, monkeypatch):
    calls = {"count": 0}

    def fourth_segment_fails(*args, **kwargs):
        calls["count"] += 1
        summary = _passing_path(*args, **kwargs)
        if calls["count"] == 4:
            summary["passed"] = False
            summary["failure_reason"] = "synthetic replay path failure"
        return summary

    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", fourth_segment_fails)
    bus = FakeBus(update_on_write=True)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--return-home", "--execute", "--confirm", safe_transfer.CONFIRM_TEXT, "--return-strategy", safe_transfer.RETURN_STRATEGY_ACHIEVED_REVERSE_REPLAY]),
        bus_factory=_bus_factory(bus),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
        sleep_fn=lambda seconds: None,
    )
    assert log["abort_reason"] == "synthetic replay path failure"
    assert log["segments"][3]["segment_name"] == "target_high_above_return"
    assert log["segments"][2]["command_sent"] is True
    assert log["segments"][3]["command_sent"] is False



def test_achieved_reverse_replay_still_checks_motion_deltas(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    calls = {"count": 0}

    def recording_motion_deltas(current_ticks, target_ticks, max_joint_delta_ticks, max_total_l1_delta_ticks, include_gripper):
        del max_joint_delta_ticks, max_total_l1_delta_ticks, include_gripper
        calls["count"] += 1
        deltas = dict((joint, abs(int(target_ticks[joint]) - int(current_ticks[joint]))) for joint in safe_transfer.ARM_JOINTS)
        checks = []
        if calls["count"] == 4:
            checks.append(safe_transfer.make_check("motion_delta_replay", False, "synthetic replay delta failure"))
        return deltas, checks

    monkeypatch.setattr(safe_transfer, "validate_motion_deltas", recording_motion_deltas)
    bus = FakeBus(update_on_write=True)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--return-home", "--execute", "--confirm", safe_transfer.CONFIRM_TEXT, "--return-strategy", safe_transfer.RETURN_STRATEGY_ACHIEVED_REVERSE_REPLAY]),
        bus_factory=_bus_factory(bus),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
        sleep_fn=lambda seconds: None,
    )
    assert calls["count"] >= 4
    assert log["abort_reason"] == "synthetic replay delta failure"
    assert log["segments"][3]["segment_name"] == "target_high_above_return"
    assert log["segments"][3]["replayed_target_ticks"] is True


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
    args = _args(tmpdir, ["--return-home", "--return-route-squares", "a2", "--route-above-offset-m", "0.140"])
    log = safe_transfer.run_safe_square_transfer(
        args,
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    with open(args.output, "r") as handle:
        saved = json.load(handle)
    assert saved["return_route_squares"] == ["a2"]
    assert saved["route_above_offset_m"] == 0.14
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
        "planned_target_ticks",
        "achieved_ticks",
        "achieved_ticks_available",
        "route_square",
        "route_waypoint",
        "current_ticks_before",
        "final_ticks_after",
        "motion_deltas_ticks",
        "readback_errors_ticks",
        "safety_checks",
        "path_validation",
        "approach_tilt_deg",
        "return_strategy",
        "replay_source_segment",
        "replay_source",
        "replayed_target_ticks",
        "settle_time_s",
        "command_sent",
        "abort_reason",
    ]
    for key in required:
        assert key in saved["segments"][0]
    route_segment = [segment for segment in saved["segments"] if segment["segment_name"] == "route_high_a2"][0]
    assert route_segment["route_square"] == "a2"
    assert route_segment["route_waypoint"] is True
    assert log["segments"][0]["ik_success"] is True


def test_output_json_contains_stop_at_and_piece_aware_fields(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    args = _args(tmpdir, [
        "--stop-at", "high_above",
        "--enforce-piece-aware-high",
        "--piece-height-m", "0.054",
        "--piece-clearance-margin-m", "0.040",
        "--high-above-offset-m", "0.120",
    ])
    log = safe_transfer.run_safe_square_transfer(
        args,
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    with open(args.output, "r") as handle:
        saved = json.load(handle)
    assert saved["stop_at"] == "high_above"
    assert saved["piece_height_m"] == 0.054
    assert saved["piece_clearance_margin_m"] == 0.04
    assert saved["piece_aware_high_required_m"] == 0.094
    assert saved["piece_aware_high_passed"] is True
    assert log["abort_reason"] is None


def test_piece_aware_high_check_passes_when_offset_is_large_enough(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, [
            "--stop-at", "high_above",
            "--enforce-piece-aware-high",
            "--piece-height-m", "0.054",
            "--piece-clearance-margin-m", "0.040",
            "--high-above-offset-m", "0.120",
        ]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert log["piece_aware_high_required_m"] == 0.094
    assert log["piece_aware_high_passed"] is True
    assert log["abort_reason"] is None


def test_piece_aware_high_check_aborts_when_offset_is_too_small(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, [
            "--stop-at", "high_above",
            "--enforce-piece-aware-high",
            "--piece-height-m", "0.054",
            "--piece-clearance-margin-m", "0.040",
            "--high-above-offset-m", "0.090",
        ]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert log["piece_aware_high_required_m"] == 0.094
    assert log["piece_aware_high_passed"] is False
    assert "requires --high-above-offset-m >= 0.094 m" in log["abort_reason"]
    assert log["segments"] == []


def test_high_above_return_home_skips_normal_above_and_route_waypoints(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, [
            "--stop-at", "high_above",
            "--return-home",
            "--return-route-squares", "a2,c3,e4",
            "--route-above-offset-m", "0.140",
        ]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert [segment["segment_name"] for segment in log["segments"]] == [
        "current_lift",
        "target_high_above",
        "home_high",
        "home_pose",
    ]
    assert "target_normal_above" not in [segment["segment_name"] for segment in log["segments"]]
    assert not [segment for segment in log["segments"] if segment.get("route_waypoint")]
    assert log["return_route_squares"] == []


def _failed_transfer_approach_report(context, args, joint_positions_rad, square=None, prefer_vertical_approach=None, enforce_approach_angle=None):
    del context, args, joint_positions_rad, square
    return {
        "approach_axis_local": [0.0, 0.0, -1.0],
        "approach_axis_name": "minus_z",
        "approach_axis_source": "tool_frame",
        "approach_axis_local_defaulted": False,
        "approach_axis_local_warning": None,
        "approach_axis_world": [1.0, 0.0, 0.0],
        "approach_tilt_deg": 90.0,
        "approach_target_world_axis": [0.0, 0.0, -1.0],
        "approach_weight": 0.05,
        "approach_preferred": bool(prefer_vertical_approach),
        "approach_enforced": bool(enforce_approach_angle),
        "approach_angle_check": {
            "passed": False,
            "tilt_deg": 90.0,
            "max_tilt_deg": 10.0,
            "failure_reason": "synthetic transfer tilt failure",
            "enforced": bool(enforce_approach_angle),
            "preferred": bool(prefer_vertical_approach),
        },
        "selected_approach_tilt_limit_deg": 10.0,
        "max_approach_tilt_deg": 10.0,
        "max_edge_approach_tilt_deg": 20.0,
        "best_candidate_axis_name": "plus_x",
        "best_candidate_axis_local": [1.0, 0.0, 0.0],
        "best_candidate_axis_world": [0.0, 0.0, -1.0],
        "best_candidate_axis_tilt_deg": 0.0,
    }


def test_enforced_target_segment_aborts_before_command(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    monkeypatch.setattr(safe_transfer, "build_approach_report", _failed_transfer_approach_report)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--prefer-vertical-approach", "--enforce-approach-angle"]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert log["abort_reason"] == "synthetic transfer tilt failure"
    assert log["segments"][1]["segment_name"] == "target_high_above"
    assert log["segments"][1]["command_sent"] is False


def test_home_pose_segment_is_not_forced_to_vertical_approach(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--return-home", "--prefer-vertical-approach"]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    home_pose_segment = [segment for segment in log["segments"] if segment["segment_name"] == "home_pose"][0]
    assert home_pose_segment["approach_preferred"] is False
    assert home_pose_segment["approach_enforced"] is False


def test_policy_resolved_json_is_saved_for_safe_transfer(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    args = _args(tmpdir, ["--square", "a1", "--approach-policy", APPROACH_POLICY_PATH])
    log = safe_transfer.run_safe_square_transfer(
        args,
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    with open(args.output, "r") as handle:
        saved = json.load(handle)
    assert saved["approach_policy_path"] == APPROACH_POLICY_PATH
    assert saved["approach_policy_square"] == "a1"
    assert saved["policy_override_applied"] is True
    assert saved["resolved_policy"]["approach_axis_name"] == "plus_z"
    assert saved["resolved_policy"]["approach_weight"] == 0.02
    assert saved["resolved_policy"]["return_route_squares"] == ["a2", "c3", "e4"]
    assert saved["resolved_policy"]["route_above_offset_m"] == 0.14
    assert saved["resolved_policy"]["lock_wrist_roll_home"] is True
    assert log["command_sent_any"] is False


def test_cli_return_route_override_wins_over_policy_for_safe_transfer(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    args = _args(tmpdir, [
        "--square", "a1",
        "--approach-policy", APPROACH_POLICY_PATH,
        "--return-home",
        "--return-route-squares", "h2,f3,e4",
        "--route-above-offset-m", "0.150",
    ])
    log = safe_transfer.run_safe_square_transfer(
        args,
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert log["return_route_squares"] == ["h2", "f3", "e4"]
    assert log["route_above_offset_m"] == 0.15
    assert [segment["segment_name"] for segment in log["segments"] if segment.get("route_waypoint")] == [
        "route_high_h2",
        "route_high_f3",
        "route_high_e4",
    ]


def test_safe_transfer_applies_seed_only_to_square_segments(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    calls = []

    def recording_solver(*args, **kwargs):
        seed = dict(kwargs["home_joint_positions_rad"])
        calls.append(seed)
        return FakeIKResult(args[1], joint_positions_rad=seed)

    seed_path = _seed_path(tmpdir, "a1", {
        "shoulder_pan": 1500,
        "shoulder_lift": 2000,
        "elbow_flex": 2500,
        "wrist_flex": 2200,
        "wrist_roll": 2500,
    })
    args = _args(tmpdir, [
        "--square", "a1",
        "--return-home",
        "--return-strategy", safe_transfer.RETURN_STRATEGY_RESOLVE_NEW,
        "--ik-seed-poses", seed_path,
    ])
    log = safe_transfer.run_safe_square_transfer(
        args,
        ik_solver=recording_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    by_name = dict((segment["segment_name"], segment) for segment in log["segments"])
    assert by_name["current_lift"]["ik_seed_source"] == "current_state"
    assert by_name["target_high_above"]["ik_seed_source"] == "square_seed_pose"
    assert by_name["target_normal_above"]["ik_seed_source"] == "square_seed_pose"
    assert by_name["target_high_above_return"]["ik_seed_source"] == "square_seed_pose"
    assert by_name["home_high"]["ik_seed_source"] == "current_state"
    assert by_name["home_pose"]["ik_seed_source"] == "not_applicable"
    assert by_name["target_high_above"]["ik_seed_ticks_used"]["wrist_roll"] == _home_ticks()["wrist_roll"]
    assert log["ik_seed_applied"] is True
    assert calls[1]["shoulder_pan"] != calls[0]["shoulder_pan"]
    assert calls[2]["shoulder_pan"] == calls[1]["shoulder_pan"]
    assert calls[4]["shoulder_pan"] == calls[3]["shoulder_pan"]


def test_home_pose_does_not_use_square_seed(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    seed_path = _seed_path(tmpdir, "a1", {
        "shoulder_pan": 1500,
        "shoulder_lift": 2000,
        "elbow_flex": 2500,
        "wrist_flex": 2200,
        "wrist_roll": 2500,
    })
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--square", "a1", "--return-home", "--ik-seed-poses", seed_path]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    home_pose_segment = [segment for segment in log["segments"] if segment["segment_name"] == "home_pose"][0]
    assert home_pose_segment["ik_seed_source"] == "not_applicable"
    assert home_pose_segment["ik_seed_ticks_used"] == {}
    assert home_pose_segment["ik_seed_joints_used"] == []


def test_no_seed_preserves_current_behavior(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    seed_path = _seed_path(tmpdir, "a1", {})
    log = safe_transfer.run_safe_square_transfer(
        _args(tmpdir, ["--square", "a1", "--ik-seed-poses", seed_path]),
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    assert log["ik_seed_applied"] is False
    assert [segment["ik_seed_source"] for segment in log["segments"]] == [
        "current_state",
        "current_state",
        "current_state",
    ]


def test_output_json_contains_ik_seed_metadata(tmpdir, monkeypatch):
    monkeypatch.setattr(safe_transfer, "validate_joint_interpolated_tcp_path", _passing_path)
    seed_path = _seed_path(tmpdir, "a1", {
        "shoulder_pan": 1500,
        "shoulder_lift": 2000,
        "elbow_flex": 2500,
        "wrist_flex": 2200,
        "wrist_roll": 2500,
    })
    args = _args(tmpdir, ["--square", "a1", "--ik-seed-poses", seed_path])
    log = safe_transfer.run_safe_square_transfer(
        args,
        ik_solver=_home_solver,
        now_fn=lambda: "2026-05-25T00:00:00Z",
    )
    with open(args.output, "r") as handle:
        saved = json.load(handle)
    assert saved["ik_seed_poses_path"] == seed_path
    assert saved["ik_seed_square"] == "a1"
    assert saved["ik_seed_applied"] is True
    assert saved["ik_seed_notes"]
    assert saved["segments"][1]["ik_seed_source"] == "square_seed_pose"
    assert "wrist_roll" in saved["segments"][1]["ik_seed_joints_used"]
    assert log["segments"][1]["ik_seed_ticks_used"]["wrist_roll"] == _home_ticks()["wrist_roll"]
