from __future__ import absolute_import

import json
import os
import sys
from argparse import Namespace

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map
from chess_robot.robot import safety
import tools.test_square_above_motion as square_above_motion

JOINT_ORDER = list(robot_square_map.DEFAULT_JOINT_ORDER)
MOVEMENT_JOINTS = [joint_name for joint_name in JOINT_ORDER if joint_name != "gripper"]


def _joint_limits(minimum=0, maximum=4095):
    limits = {}
    for joint_name in JOINT_ORDER:
        limits[joint_name] = {
            "provisional_min": int(minimum),
            "provisional_max": int(maximum),
        }
    return limits


def _servo_map():
    return {
        "joints": {
            "shoulder_pan": {"id": 1},
            "shoulder_lift": {"id": 2},
            "elbow_flex": {"id": 3},
            "wrist_flex": {"id": 4},
            "wrist_roll": {"id": 5},
            "gripper": {"id": 6},
        },
        "aliases": {},
    }


def _manual_pose(base):
    joints = {}
    for index, joint_name in enumerate(JOINT_ORDER):
        joints[joint_name] = int(base + (index * 10))
    return {
        "source": "manual",
        "confidence": "taught",
        "joints": joints,
        "recorded_at": "2026-05-21T00:00:00Z",
        "notes": ["manual anchor"],
    }


def _document_with_squares(square_names):
    document = robot_square_map.default_square_targets()
    for offset, square_name in enumerate(square_names):
        document["squares"][square_name] = {
            "above_pose": _manual_pose(1500 + (offset * 100)),
        }
    return document


def _write_yaml(path, data):
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, default_flow_style=False)


def _write_inputs(tmpdir, document):
    targets_path = str(tmpdir.join("square_targets.yaml"))
    limits_path = str(tmpdir.join("joint_limits.yaml"))
    servo_map_path = str(tmpdir.join("servo_map.yaml"))
    config_path = str(tmpdir.join("robot.yaml"))
    robot_square_map.save_yaml_file(targets_path, document)
    _write_yaml(limits_path, {"limits": _joint_limits(1000, 3000)})
    _write_yaml(servo_map_path, _servo_map())
    _write_yaml(config_path, {
        "servo_bus": {
            "backend": "mock",
            "dry_run_default": True,
            "feetech": {"port": "/dev/null", "baudrate": 1000000},
            "mock": {"servo_ids": [1, 2, 3, 4, 5, 6], "positions": {}},
        },
        "joints": {
            "shoulder_pan": {"servo_id": 1},
            "shoulder_lift": {"servo_id": 2},
            "elbow_flex": {"servo_id": 3},
            "wrist_flex": {"servo_id": 4},
            "wrist_roll": {"servo_id": 5},
            "gripper": {"servo_id": 6},
        },
    })
    return targets_path, limits_path, servo_map_path, config_path


def _make_args(targets_path, limits_path, servo_map_path, config_path, **overrides):
    values = {
        "targets": targets_path,
        "joint_limits": limits_path,
        "servo_map": servo_map_path,
        "robot_config": config_path,
        "path": "corner-loop",
        "squares": None,
        "real": False,
        "confirm_text": None,
        "pause_each": False,
        "pause_seconds": 1.0,
        "step_size_ticks": 20,
        "step_delay": 0.05,
        "settle_time": 0.5,
        "include_gripper": False,
        "stop_on_warning": True,
        "log": None,
        "output_json": None,
        "max_squares": None,
        "start_square": None,
        "end_square": None,
    }
    values.update(overrides)
    return Namespace(**values)


class FakeBus(object):
    def __init__(self, positions):
        self.positions = dict(positions)
        self.goal_positions = dict(positions)
        self.torque_events = []
        self.closed = False

    def read_position(self, servo_id):
        return self.positions.get(servo_id)

    def read_register(self, servo_id, address, length):
        return 0

    def write_goal_position(self, servo_id, goal_position):
        self.goal_positions[int(servo_id)] = int(goal_position)
        self.positions[int(servo_id)] = int(goal_position)

    def torque_enable(self, servo_id, enabled):
        self.torque_events.append((int(servo_id), bool(enabled)))

    def close(self):
        self.closed = True


def test_corner_loop_path_returns_expected_sequence():
    assert square_above_motion.build_corner_loop_path() == ["a1", "h1", "h8", "a8", "a1"]


def test_manual_anchors_path_returns_only_manual_in_preferred_order():
    document = robot_square_map.default_square_targets()
    for offset, square_name in enumerate(["h3", "a1", "c1", "h8"]):
        document["squares"][square_name] = {"above_pose": _manual_pose(1500 + (offset * 100))}
    document["squares"]["f1"] = {
        "above_pose": {
            "source": "generated",
            "confidence": "high",
            "joints": _manual_pose(1900)["joints"],
            "generated_at": "2026-05-21T00:00:00Z",
            "notes": ["generated"],
        }
    }
    squares, warnings = square_above_motion.build_manual_anchor_path(document)
    assert squares == ["a1", "c1", "h3", "h8"]
    assert warnings


def test_all_squares_path_returns_serpentine_64():
    squares = square_above_motion.build_all_squares_path()
    assert len(squares) == 64
    assert squares[:8] == ["a1", "b1", "c1", "d1", "e1", "f1", "g1", "h1"]
    assert squares[8:16] == ["h2", "g2", "f2", "e2", "d2", "c2", "b2", "a2"]
    assert squares[-8:] == ["h8", "g8", "f8", "e8", "d8", "c8", "b8", "a8"]


def test_explicit_square_path_parsing_works():
    assert square_above_motion.parse_explicit_square_path("a1,h1,h8,a8,a1") == ["a1", "h1", "h8", "a8", "a1"]


def test_invalid_square_is_rejected():
    with pytest.raises(ValueError):
        square_above_motion.parse_explicit_square_path("z9")


def test_missing_above_pose_is_rejected():
    document = robot_square_map.default_square_targets()
    with pytest.raises(square_above_motion.SquareAboveMotionError):
        square_above_motion.select_square_targets(document, _joint_limits(1000, 3000), _servo_map(), ["a1"])


def test_target_outside_joint_limit_is_rejected():
    document = _document_with_squares(["a1"])
    document["squares"]["a1"]["above_pose"]["joints"]["shoulder_pan"] = 3500
    with pytest.raises(square_above_motion.SquareAboveMotionError):
        square_above_motion.select_square_targets(document, _joint_limits(1000, 3000), _servo_map(), ["a1"])


def test_gripper_is_excluded_by_default():
    document = _document_with_squares(["a1"])
    plan = square_above_motion.select_square_targets(document, _joint_limits(1000, 3000), _servo_map(), ["a1"])
    assert "gripper" not in plan["movement_joints"]
    assert "gripper" not in plan["selected"][0]["target_joints"]


def test_gripper_included_only_with_flag():
    document = _document_with_squares(["a1"])
    plan = square_above_motion.select_square_targets(
        document,
        _joint_limits(1000, 3000),
        _servo_map(),
        ["a1"],
        include_gripper=True,
    )
    assert "gripper" in plan["movement_joints"]
    assert "gripper" in plan["selected"][0]["target_joints"]


def test_intermediate_step_generation_respects_step_size():
    current = {"shoulder_pan": 1000, "elbow_flex": 1000}
    target = {"shoulder_pan": 1065, "elbow_flex": 1035}
    poses = square_above_motion.build_intermediate_poses(current, target, 20)
    previous = dict(current)
    for pose in poses:
        for joint_name in pose:
            assert abs(int(pose[joint_name]) - int(previous[joint_name])) <= 20
        previous = pose
    assert poses[-1] == target


def test_dry_run_does_not_touch_hardware(tmpdir):
    document = _document_with_squares(["a1", "h1", "h8", "a8"])
    targets_path, limits_path, servo_map_path, config_path = _write_inputs(tmpdir, document)
    args = _make_args(targets_path, limits_path, servo_map_path, config_path)
    calls = {"count": 0}

    def fake_bus_factory(**kwargs):
        calls["count"] += 1
        raise AssertionError("dry-run should not build a hardware bus")

    exit_code, result = square_above_motion.run(args, bus_factory=fake_bus_factory)
    assert exit_code == 0
    assert result["dry_run"] is True
    assert calls["count"] == 0


def test_typed_confirmation_is_required_for_real(tmpdir):
    document = _document_with_squares(["a1", "h1", "h8", "a8"])
    targets_path, limits_path, servo_map_path, config_path = _write_inputs(tmpdir, document)
    args = _make_args(targets_path, limits_path, servo_map_path, config_path, real=True, confirm_text=None)
    calls = {"count": 0}

    def fake_bus_factory(**kwargs):
        calls["count"] += 1
        return None

    with pytest.raises(safety.SafetyError):
        square_above_motion.run(args, bus_factory=fake_bus_factory)
    assert calls["count"] == 0


def test_json_result_writer_works_for_dry_run(tmpdir):
    document = _document_with_squares(["a1", "h1", "h8", "a8"])
    targets_path, limits_path, servo_map_path, config_path = _write_inputs(tmpdir, document)
    output_json = str(tmpdir.join("result.json"))
    args = _make_args(
        targets_path,
        limits_path,
        servo_map_path,
        config_path,
        output_json=output_json,
        log=str(tmpdir.join("motion.log")),
    )
    exit_code, result = square_above_motion.run(args)
    assert exit_code == 0
    with open(output_json, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    assert payload["dry_run"] is True
    assert payload["path_name"] == "corner-loop"
    assert len(payload["per_square_results"]) == len(result["per_square_results"])


def test_abort_result_structure_is_valid(tmpdir):
    document = _document_with_squares(["a1", "h1"])
    targets_path, limits_path, servo_map_path, config_path = _write_inputs(tmpdir, document)
    args = _make_args(
        targets_path,
        limits_path,
        servo_map_path,
        config_path,
        squares="a1,h1",
        real=True,
        confirm_text="MOVE ABOVE SQUARES",
        pause_each=True,
    )
    initial_positions = {1: 1500, 2: 1510, 3: 1520, 4: 1530, 5: 1540}
    fake_bus = FakeBus(initial_positions)

    def fake_bus_factory(**kwargs):
        return fake_bus

    def fake_config_loader(path):
        with open(path, "r", encoding="utf-8") as handle:
            return yaml.safe_load(handle)

    exit_code, result = square_above_motion.run(
        args,
        bus_factory=fake_bus_factory,
        config_loader=fake_config_loader,
        sleep_fn=lambda seconds: None,
        pause_input_fn=lambda prompt="": "q",
    )
    assert exit_code == 1
    assert result["aborted"] is True
    assert result["abort_reason"] == "operator requested abort during pause"
    assert result["final_torque_disable_attempted"] is True
    assert result["final_torque_disable_success"] is True
