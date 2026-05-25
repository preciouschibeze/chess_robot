from __future__ import absolute_import

import json
import importlib.util
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
TOOLS_ROOT = os.path.join(ROOT, "tools")
if TOOLS_ROOT not in sys.path:
    sys.path.insert(0, TOOLS_ROOT)

from chess_robot.robot import ik_validation
from chess_robot.robot.ik_validation import ARM_JOINTS
from chess_robot.robot.ik_validation import CONFIRM_TEXT
from chess_robot.robot.ik_validation import calculate_safety_limit_margins_ticks
from chess_robot.robot.ik_validation import select_target
from chess_robot.robot.ik_validation import validate_motion_deltas
from chess_robot.robot.joint_calibration import load_pose_ticks
from chess_robot.robot.reachability import generate_targets
from chess_robot.robot.workspace import load_scene_geometry


SCENE_PATH = os.path.join(ROOT, "data", "calibration", "robot", "scene_geometry.yaml")
URDF_PATH = os.path.join(ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
JOINT_CALIBRATION_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
JOINT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
JOINT_SAFETY_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")
HOME_POSE_PATH = os.path.join(ROOT, "data", "calibration", "robot", "home_pose.yaml")
TOOL_FRAMES_PATH = os.path.join(ROOT, "data", "calibration", "gripper", "tool_frames.yaml")

CLI_SPEC = importlib.util.spec_from_file_location(
    "single_ik_pose_cli",
    os.path.join(TOOLS_ROOT, "test_single_ik_pose.py"),
)
single_ik_pose_cli = importlib.util.module_from_spec(CLI_SPEC)
CLI_SPEC.loader.exec_module(single_ik_pose_cli)


class FakeIKResult(object):
    def __init__(self, success=True):
        self.success = success
        self.status = "success" if success else "max_iters"
        self.final_xyz_robot = np.asarray((0.01, 0.02, 0.03), dtype=float)
        self.error_m = 0.001 if success else 0.100
        self.iterations = 3
        self.joint_positions_rad = dict((joint, 0.0) for joint in ARM_JOINTS)


class FakeBus(object):
    def __init__(self, positions):
        self.positions = dict(positions)
        self.writes = []

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
        self.positions[int(servo_id)] = int(goal_position)

    def close(self):
        pass


def _args(tmpdir, extra=None):
    output = os.path.join(str(tmpdir), "single_pose.json")
    extra = list(extra or [])
    explicit_target = any(
        value in extra
        for value in (
            "--square",
            "--capture",
            "--target-world",
            "--target-home-pose",
            "--target-world-offset-from-home",
        )
    )
    values = [
        "--urdf", URDF_PATH,
        "--scene", SCENE_PATH,
        "--joint-calibration", JOINT_CALIBRATION_PATH,
        "--joint-limits", JOINT_LIMITS_PATH,
        "--joint-safety-limits", JOINT_SAFETY_LIMITS_PATH,
        "--home-pose", HOME_POSE_PATH,
        "--tool-frames", TOOL_FRAMES_PATH,
        "--tcp-frame", "gripper_frame",
        "--target-type", "above",
        "--workspace-seed-samples", "0",
        "--output", output,
    ]
    if not explicit_target:
        values.extend(["--square", "e4"])
    values.extend(extra)
    return single_ik_pose_cli.build_parser().parse_args(values)


def _fake_bus_factory(bus):
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


def test_dry_run_reports_path_validation_without_hardware(tmpdir):
    def forbidden_bus_factory(args):
        del args
        raise AssertionError("dry-run must not open hardware")

    args = _args(tmpdir)
    log = ik_validation.run_single_pose_validation(
        args,
        bus_factory=forbidden_bus_factory,
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    assert log["mode"] == "dry_run"
    assert log["command_sent"] is False
    assert log["path_validation"] is not None
    assert log["path_validation"]["current_ticks_source"] == "saved_home_pose"
    assert log["approach_angle_check"] is not None


def test_dry_run_never_calls_hardware_command_functions(tmpdir):
    def forbidden_bus_factory(args):
        del args
        raise AssertionError("dry-run must not open hardware")

    args = _args(tmpdir)
    log = ik_validation.run_single_pose_validation(
        args,
        bus_factory=forbidden_bus_factory,
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    assert log["mode"] == "dry_run"
    assert log["command_sent"] is False
    assert log["abort_reason"] is None


def test_execute_aborts_when_board_clearance_path_fails(tmpdir, monkeypatch):
    args = _args(tmpdir, ["--execute", "--confirm", CONFIRM_TEXT, "--allow-large-delta"])
    bus = FakeBus({1: 1000, 2: 1000, 3: 1000, 4: 1000, 5: 1000})

    def failing_path_validation(log, context, args, reference_ticks, reference_source):
        del context, args, reference_ticks
        log["path_validation"] = {
            "xy_delta_m": 0.050,
            "min_z_m": 0.040,
            "low_zone_z_m": 0.086,
            "passed": False,
            "failure_reason": "synthetic unsafe path",
            "samples_count": 25,
            "xy_changing": True,
            "current_tcp_world_xyz_m": [0.0, 0.0, 0.100],
            "target_tcp_world_xyz_m": [0.05, 0.0, 0.100],
            "current_ticks_source": str(reference_source),
        }
        return log["path_validation"]

    monkeypatch.setattr(ik_validation, "attach_path_validation_from_reference_ticks", failing_path_validation)
    log = ik_validation.run_single_pose_validation(
        args,
        bus_factory=_fake_bus_factory(bus),
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
        sleep_fn=lambda seconds: None,
    )
    assert log["abort_reason"] == "synthetic unsafe path"
    assert log["command_sent"] is False
    assert bus.writes == []


def test_execute_mode_without_confirm_aborts_before_hardware(tmpdir):
    def forbidden_bus_factory(args):
        del args
        raise AssertionError("confirmation failure must not open hardware")

    args = _args(tmpdir, ["--execute"])
    log = ik_validation.run_single_pose_validation(
        args,
        bus_factory=forbidden_bus_factory,
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    assert "requires --confirm" in log["abort_reason"]
    assert log["command_sent"] is False


def test_failed_ik_aborts(tmpdir):
    args = _args(tmpdir)
    log = ik_validation.run_single_pose_validation(
        args,
        ik_solver=lambda *a, **k: FakeIKResult(success=False),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    assert log["ik_success"] is False
    assert "IK failed" in log["abort_reason"]


def test_out_of_limit_target_aborts(tmpdir):
    args = _args(tmpdir)

    class OutOfLimitResult(FakeIKResult):
        def __init__(self):
            FakeIKResult.__init__(self, success=True)
            self.joint_positions_rad["shoulder_pan"] = 100.0

    log = ik_validation.run_single_pose_validation(
        args,
        ik_solver=lambda *a, **k: OutOfLimitResult(),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    assert "outside safety limits" in log["abort_reason"]


def test_large_delta_aborts_unless_override_is_supplied():
    current = dict((joint, 1000) for joint in ARM_JOINTS)
    target = dict((joint, 1500) for joint in ARM_JOINTS)
    deltas, checks = validate_motion_deltas(current, target, 350, 1200, False)
    assert deltas["shoulder_pan"] == 500
    assert any(not check["ok"] for check in checks)
    deltas, checks = validate_motion_deltas(current, target, 350, 1200, True)
    assert all(check["ok"] for check in checks)


def test_square_target_generation_matches_analyse_square_ik_convention():
    scene = load_scene_geometry(SCENE_PATH)
    selected = select_target(scene, square="h1", target_type="above")
    generated = generate_targets(scene, above_board_offset_m=0.080, pick_offset_m=0.030)
    h1_above = [target for target in generated if target["target_name"] == "h1_above"][0]
    assert selected["target_world_xyz_m"] == [h1_above["x_m"], h1_above["y_m"], h1_above["z_m"]]


def test_explicit_target_world_mode_works():
    scene = load_scene_geometry(SCENE_PATH)
    selected = select_target(scene, target_world=[0.1, 0.2, 0.3])
    assert selected["target_mode"] == "explicit_world"
    assert selected["target_world_xyz_m"] == [0.1, 0.2, 0.3]


def test_gripper_is_excluded_from_commanded_joints(tmpdir):
    args = _args(tmpdir, ["--execute", "--confirm", CONFIRM_TEXT, "--allow-large-delta", "--no-enforce-board-clearance"])
    bus = FakeBus({1: 1000, 2: 1000, 3: 1000, 4: 1000, 5: 1000})
    log = ik_validation.run_single_pose_validation(
        args,
        bus_factory=_fake_bus_factory(bus),
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
        sleep_fn=lambda seconds: None,
    )
    written_ids = set(servo_id for servo_id, tick in bus.writes)
    assert log["command_sent"] is True
    assert 6 not in written_ids
    assert written_ids == set([1, 2, 3, 4, 5])


def test_lock_wrist_roll_home_json_contains_locked_joint_metadata(tmpdir):
    args = _args(tmpdir, ["--lock-wrist-roll-home"])
    log = ik_validation.run_single_pose_validation(
        args,
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    assert log["locked_joints_ticks"] == {"wrist_roll": 1091}
    assert log["locked_joint_sources"] == {"wrist_roll": "home_pose"}
    assert log["target_ticks"]["wrist_roll"] == 1091
    assert log["joint_angles_rad"]["wrist_roll"] == log["locked_joints_rad"]["wrist_roll"]



def test_d4_above_lock_wrist_roll_home_avoids_large_wrist_roll_home_delta(tmpdir):
    args = _args(tmpdir, ["--square", "d4", "--lock-wrist-roll-home"])
    log = ik_validation.run_single_pose_validation(
        args,
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    home_pose_ticks = load_pose_ticks(HOME_POSE_PATH)
    assert log["locked_joints_ticks"]["wrist_roll"] == int(home_pose_ticks["wrist_roll"])
    assert abs(int(log["target_ticks"]["wrist_roll"]) - int(home_pose_ticks["wrist_roll"])) <= 1



def test_lock_joint_cli_rad_value_is_preserved_in_target_ticks(tmpdir):
    args = _args(tmpdir, ["--lock-joint", "wrist_roll_rad=0.0"])
    log = ik_validation.run_single_pose_validation(
        args,
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    assert log["locked_joint_sources"]["wrist_roll"] == "cli_rad"
    assert log["target_ticks"]["wrist_roll"] == log["locked_joints_ticks"]["wrist_roll"]



def test_json_output_contains_required_fields(tmpdir):
    args = _args(tmpdir)
    log = ik_validation.run_single_pose_validation(
        args,
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    with open(args.output, "r") as handle:
        saved = json.load(handle)
    required = [
        "mode",
        "target_mode",
        "target_world_xyz_m",
        "target_robot_xyz_m",
        "tcp_frame",
        "ik_success",
        "ik_status",
        "target_ticks",
        "locked_joints_rad",
        "locked_joints_ticks",
        "locked_joint_sources",
        "safety_checks",
        "command_sent",
        "timestamp",
    ]
    for key in required:
        assert key in saved
    assert saved["target_ticks"] == log["target_ticks"]


def test_safety_margin_calculation_works():
    limits = {
        "joints": {
            "shoulder_pan": {"min_tick": 900, "max_tick": 1100},
            "shoulder_lift": {"min_tick": 800, "max_tick": 1300},
        }
    }
    margins = calculate_safety_limit_margins_ticks(
        {"shoulder_pan": 1000, "shoulder_lift": 850},
        limits,
    )
    assert margins["shoulder_pan"]["min"] == 100
    assert margins["shoulder_lift"]["lower"] == 50
    assert margins["shoulder_lift"]["upper"] == 450


def test_target_home_pose_generates_target_from_saved_home_fk(tmpdir):
    args = _args(tmpdir, ["--target-home-pose"])
    log = ik_validation.run_single_pose_validation(
        args,
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    assert log["target_mode"] == "home_pose"
    assert np.allclose(log["target_robot_xyz_m"], log["saved_home_tcp_robot_xyz_m"])
    assert np.allclose(log["target_world_xyz_m"], log["saved_home_tcp_world_xyz_m"])


def test_world_offset_from_home_adds_requested_world_offset(tmpdir):
    args = _args(tmpdir, ["--target-world-offset-from-home", "0.001", "-0.002", "0.010"])
    log = ik_validation.run_single_pose_validation(
        args,
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    home = np.asarray(log["saved_home_tcp_world_xyz_m"], dtype=float)
    target = np.asarray(log["target_world_xyz_m"], dtype=float)
    assert log["target_mode"] == "world_offset_from_home"
    assert np.allclose(target - home, np.asarray((0.001, -0.002, 0.010), dtype=float))


def test_target_mode_exclusivity_rejects_multiple_target_modes(tmpdir):
    with pytest.raises(SystemExit):
        _args(tmpdir, ["--square", "e4", "--target-home-pose"])


def test_target_mode_exclusivity_rejects_no_target_mode(tmpdir):
    output = os.path.join(str(tmpdir), "single_pose.json")
    values = [
        "--urdf", URDF_PATH,
        "--scene", SCENE_PATH,
        "--joint-calibration", JOINT_CALIBRATION_PATH,
        "--joint-safety-limits", JOINT_SAFETY_LIMITS_PATH,
        "--home-pose", HOME_POSE_PATH,
        "--tool-frames", TOOL_FRAMES_PATH,
        "--tcp-frame", "gripper_frame",
        "--output", output,
    ]
    with pytest.raises(SystemExit):
        single_ik_pose_cli.build_parser().parse_args(values)


def test_dry_run_home_pose_mode_does_not_command_hardware(tmpdir):
    def forbidden_bus_factory(args):
        del args
        raise AssertionError("dry-run must not open hardware")

    args = _args(tmpdir, ["--target-home-pose"])
    log = ik_validation.run_single_pose_validation(
        args,
        bus_factory=forbidden_bus_factory,
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    assert log["command_sent"] is False
    assert "gripper" not in log["target_ticks"]


def test_output_json_contains_saved_home_tcp_world_for_home_modes(tmpdir):
    args = _args(tmpdir, ["--target-home-pose"])
    ik_validation.run_single_pose_validation(
        args,
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    with open(args.output, "r") as handle:
        saved = json.load(handle)
    assert "saved_home_tcp_world_xyz_m" in saved

    args = _args(tmpdir, ["--target-world-offset-from-home", "0", "0", "0.010"])
    ik_validation.run_single_pose_validation(
        args,
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    with open(args.output, "r") as handle:
        saved = json.load(handle)
    assert "saved_home_tcp_world_xyz_m" in saved
    assert saved["requested_offset_world_m"] == [0.0, 0.0, 0.01]


def test_zplus10_offset_changes_target_world_z_by_exactly_10mm(tmpdir):
    args = _args(tmpdir, ["--target-world-offset-from-home", "0.000", "0.000", "0.010"])
    log = ik_validation.run_single_pose_validation(
        args,
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    dz = log["target_world_xyz_m"][2] - log["saved_home_tcp_world_xyz_m"][2]
    assert abs(dz - 0.010) < 1.0e-12


def _synthetic_failed_approach_report(*args, **kwargs):
    del args, kwargs
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
        "approach_preferred": True,
        "approach_enforced": True,
        "approach_angle_check": {
            "passed": False,
            "tilt_deg": 90.0,
            "max_tilt_deg": 10.0,
            "failure_reason": "synthetic tilt failure",
            "enforced": True,
            "preferred": True,
        },
        "selected_approach_tilt_limit_deg": 10.0,
        "max_approach_tilt_deg": 10.0,
        "max_edge_approach_tilt_deg": 20.0,
        "best_candidate_axis_name": "plus_x",
        "best_candidate_axis_local": [1.0, 0.0, 0.0],
        "best_candidate_axis_world": [0.0, 0.0, -1.0],
        "best_candidate_axis_tilt_deg": 0.0,
    }


def test_enforce_approach_angle_aborts_when_failed_report_is_returned(tmpdir, monkeypatch):
    monkeypatch.setattr(ik_validation, "build_approach_report", _synthetic_failed_approach_report)
    args = _args(tmpdir, ["--prefer-vertical-approach", "--enforce-approach-angle"])
    log = ik_validation.run_single_pose_validation(
        args,
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    assert log["abort_reason"] == "synthetic tilt failure"


def test_prefer_only_mode_reports_failed_approach_check_without_aborting(tmpdir, monkeypatch):
    monkeypatch.setattr(ik_validation, "build_approach_report", _synthetic_failed_approach_report)
    args = _args(tmpdir, ["--prefer-vertical-approach"])
    log = ik_validation.run_single_pose_validation(
        args,
        ik_solver=lambda *a, **k: FakeIKResult(success=True),
        now_fn=lambda: "2026-05-24T00:00:00Z",
    )
    assert log["approach_angle_check"]["passed"] is False
    assert log["abort_reason"] is None
