from __future__ import absolute_import

import json
import os
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.calibration import robot_square_map
import tools.test_open_loop_pick_place as open_loop_pick_place


JOINT_ORDER = list(robot_square_map.DEFAULT_JOINT_ORDER)
ARM_JOINTS = [joint_name for joint_name in JOINT_ORDER if joint_name != "gripper"]


def _joint_limits(minimum=900, maximum=3000):
    limits = {}
    for joint_name in ARM_JOINTS:
        limits[joint_name] = {"provisional_min": minimum, "provisional_max": maximum}
    limits["gripper"] = {"provisional_min": 1463, "provisional_max": 1738}
    return {"limits": limits}


def _servo_map():
    return {
        "joints": dict((joint_name, {"id": index + 1}) for index, joint_name in enumerate(JOINT_ORDER))
    }


def _robot_config():
    return {
        "servo_bus": {
            "backend": "mock",
            "dry_run_default": True,
            "feetech": {"port": "/dev/null", "baudrate": 1000000},
        },
        "joints": dict((joint_name, {"servo_id": index + 1}) for index, joint_name in enumerate(JOINT_ORDER)),
    }


def _pose(seed, source="manual", gripper_value=1613):
    joints = {
        "shoulder_pan": seed + 1,
        "shoulder_lift": seed + 2,
        "elbow_flex": seed + 3,
        "wrist_flex": seed + 4,
        "wrist_roll": seed + 5,
        "gripper": gripper_value,
    }
    entry = {
        "source": source,
        "confidence": "taught" if source == "manual" else "high",
        "joints": joints,
    }
    if source == "manual":
        entry["recorded_at"] = "2026-05-22T00:00:00Z"
    else:
        entry["generated_at"] = "2026-05-22T00:00:00Z"
        entry["notes"] = ["generated"]
    return entry


def _targets(include_dest_place=True, source_pick_source="manual", dest_place_source="manual",
             source_above_source="manual", dest_above_source="manual"):
    document = robot_square_map.default_square_targets()
    document["squares"] = {
        "c3": {
            "above_pose": _pose(2000, source=source_above_source, gripper_value=1613),
            "pick_pose": _pose(2100, source=source_pick_source, gripper_value=1111),
            "place_pose": _pose(2200, source="manual", gripper_value=1112),
        },
        "d4": {
            "above_pose": _pose(2300, source=dest_above_source, gripper_value=1614),
            "pick_pose": _pose(2400, source="manual", gripper_value=1113),
        },
    }
    if include_dest_place:
        document["squares"]["d4"]["place_pose"] = _pose(2500, source=dest_place_source, gripper_value=1114)
    return document


def _home_pose(seed=2600):
    joints = {}
    for index, joint_name in enumerate(JOINT_ORDER):
        position = 1600 if joint_name == "gripper" else seed + index + 1
        joints[joint_name] = {"id": index + 1, "position": position}
    return {
        "source": "test",
        "joints": joints,
    }


def _gripper_profile(open_position=1704, grasp_position=1536, release_position=1652,
                     neutral_position=1616, pre_grasp_position=1596):
    profile = {
        "gripper": {
            "joint": "gripper",
            "servo_id": 6,
            "limits": {"min": 1463, "max": 1738},
            "open_position": open_position,
            "grasp_position": grasp_position,
            "release_position": release_position,
            "neutral_position": neutral_position,
            "pre_grasp_position": pre_grasp_position,
        }
    }
    return profile


def _write_yaml(tmpdir, name, data):
    path = str(tmpdir.join(name))
    with open(path, "w") as handle:
        yaml.safe_dump(data, handle, default_flow_style=False)
    return path


def _paths(tmpdir, targets=None, joint_limits=None, servo_map=None, gripper_profile=None, robot_config=None, home_pose=None):
    return {
        "targets": _write_yaml(tmpdir, "square_targets.yaml", targets if targets is not None else _targets()),
        "joint_limits": _write_yaml(tmpdir, "joint_limits.yaml", joint_limits if joint_limits is not None else _joint_limits()),
        "servo_map": _write_yaml(tmpdir, "servo_map.yaml", servo_map if servo_map is not None else _servo_map()),
        "gripper_profile": _write_yaml(tmpdir, "gripper_profile.yaml", gripper_profile if gripper_profile is not None else _gripper_profile()),
        "robot_config": _write_yaml(tmpdir, "robot.yaml", robot_config if robot_config is not None else _robot_config()),
        "home_pose": _write_yaml(tmpdir, "home_pose.yaml", home_pose if home_pose is not None else _home_pose()),
    }


def _make_args(tmpdir, paths=None, **overrides):
    if paths is None:
        paths = _paths(tmpdir)
    values = {
        "source": "c3",
        "dest": "c3",
        "targets": paths["targets"],
        "joint_limits": paths["joint_limits"],
        "servo_map": paths["servo_map"],
        "gripper_profile": paths["gripper_profile"],
        "robot_config": paths["robot_config"],
        "home_pose": paths["home_pose"],
        "real": False,
        "confirm_text": None,
        "pause_each": None,
        "step_size_ticks": 5,
        "step_delay": 0.15,
        "settle_time": 1.0,
        "gripper_step_size_ticks": 5,
        "gripper_step_delay": 0.08,
        "log": str(tmpdir.join("open_loop_pick_place.log")),
        "output_json": str(tmpdir.join("result.json")),
        "piece": "rook",
        "allow_same_square": False,
        "allow_place_uses_pick": False,
        "return_home_after": False,
    }
    values.update(overrides)
    return type("Args", (object,), values)()


class FakeBus(object):
    def __init__(self):
        self.positions = {1: 2001, 2: 2002, 3: 2003, 4: 2004, 5: 2005, 6: 1704}
        self.torque_calls = []
        self.write_calls = []
        self.closed = False

    def read_position(self, servo_id):
        return self.positions.get(int(servo_id))

    def read_register(self, servo_id, address, length):
        if int(address) == 65 and int(length) == 1:
            return 0
        return 0

    def write_goal_position(self, servo_id, goal_position):
        self.positions[int(servo_id)] = int(goal_position)
        self.write_calls.append((int(servo_id), int(goal_position)))

    def torque_enable(self, servo_id, enabled):
        self.torque_calls.append((int(servo_id), bool(enabled)))

    def close(self):
        self.closed = True


def test_c3_to_c3_dry_run_sequence_builds_expected_stages(tmpdir):
    args = _make_args(tmpdir, allow_same_square=True)
    validation = open_loop_pick_place.validate_inputs(args)
    validation["step_size_ticks"] = args.step_size_ticks
    validation["gripper_step_size_ticks"] = args.gripper_step_size_ticks
    stages = open_loop_pick_place.build_stage_sequence(validation, pause_each=False)
    assert [stage["name"] for stage in stages] == [
        "open_gripper",
        "move_source_above",
        "move_pre_grasp",
        "move_source_pick",
        "close_gripper",
        "move_source_above_after_pick",
        "move_dest_above",
        "move_dest_place",
        "release_gripper",
        "move_dest_above_after_place",
        "move_gripper_neutral",
    ]


def test_same_square_real_mode_requires_allow_same_square(tmpdir):
    args = _make_args(
        tmpdir,
        real=True,
        confirm_text=open_loop_pick_place.EXPECTED_CONFIRM_TEXT,
        allow_same_square=False,
    )
    with pytest.raises(open_loop_pick_place.OpenLoopPickPlaceError):
        open_loop_pick_place.validate_inputs(args)


def test_missing_source_pick_pose_is_rejected(tmpdir):
    document = _targets()
    del document["squares"]["c3"]["pick_pose"]
    paths = _paths(tmpdir, targets=document)
    args = _make_args(tmpdir, paths=paths)
    with pytest.raises(open_loop_pick_place.OpenLoopPickPlaceError):
        open_loop_pick_place.validate_inputs(args)


def test_missing_dest_place_pose_is_rejected_without_allow_place_uses_pick(tmpdir):
    document = _targets(include_dest_place=False)
    paths = _paths(tmpdir, targets=document)
    args = _make_args(tmpdir, paths=paths, dest="d4")
    with pytest.raises(open_loop_pick_place.OpenLoopPickPlaceError):
        open_loop_pick_place.validate_inputs(args)


def test_place_pose_fallback_works_with_allow_place_uses_pick(tmpdir):
    document = _targets(include_dest_place=False)
    paths = _paths(tmpdir, targets=document)
    args = _make_args(tmpdir, paths=paths, dest="d4", allow_place_uses_pick=True)
    validation = open_loop_pick_place.validate_inputs(args)
    assert validation["dest_place"]["pose_name"] == "pick_pose"
    assert any("fell back to pick_pose" in warning for warning in validation["warnings"])


def test_pick_pose_and_place_pose_must_be_manual(tmpdir):
    document = _targets(source_pick_source="generated")
    paths = _paths(tmpdir, targets=document)
    args = _make_args(tmpdir, paths=paths)
    with pytest.raises(open_loop_pick_place.OpenLoopPickPlaceError):
        open_loop_pick_place.validate_inputs(args)

    document = _targets(dest_place_source="generated")
    paths = _paths(tmpdir, targets=document)
    args = _make_args(tmpdir, paths=paths, dest="d4", allow_place_uses_pick=False)
    with pytest.raises(open_loop_pick_place.OpenLoopPickPlaceError):
        open_loop_pick_place.validate_inputs(args)


def test_gripper_commands_use_profile_not_pose_ticks(tmpdir):
    args = _make_args(tmpdir, allow_same_square=True)
    validation = open_loop_pick_place.validate_inputs(args)
    validation["step_size_ticks"] = args.step_size_ticks
    validation["gripper_step_size_ticks"] = args.gripper_step_size_ticks
    stages = open_loop_pick_place.build_stage_sequence(validation, pause_each=False)
    stage_map = dict((stage["name"], stage) for stage in stages)
    assert stage_map["open_gripper"]["target_gripper_value"] == 1704
    assert stage_map["close_gripper"]["target_gripper_value"] == 1536
    assert stage_map["release_gripper"]["target_gripper_value"] == 1652
    assert stage_map["open_gripper"]["target_gripper_value"] != 1111
    assert stage_map["close_gripper"]["target_gripper_value"] != 1111


def test_gripper_targets_validate_against_limits(tmpdir):
    bad_profile = _gripper_profile(open_position=1800)
    paths = _paths(tmpdir, gripper_profile=bad_profile)
    args = _make_args(tmpdir, paths=paths)
    with pytest.raises(open_loop_pick_place.OpenLoopPickPlaceError):
        open_loop_pick_place.validate_inputs(args)


def test_arm_intermediate_step_generation_respects_step_size():
    current = {"shoulder_pan": 1000, "elbow_flex": 1000}
    target = {"shoulder_pan": 1065, "elbow_flex": 1035}
    poses = open_loop_pick_place.build_intermediate_poses(current, target, 20)
    previous = dict(current)
    for pose in poses:
        for joint_name in pose:
            assert abs(int(pose[joint_name]) - int(previous[joint_name])) <= 20
        previous = pose
    assert poses[-1] == target


def test_gripper_intermediate_step_generation_respects_step_size():
    positions = open_loop_pick_place.build_intermediate_positions(1704, 1536, 5)
    previous = 1704
    for position in positions:
        assert abs(int(position) - int(previous)) <= 5
        previous = position
    assert positions[-1] == 1536


def test_dry_run_does_not_touch_hardware(tmpdir):
    args = _make_args(tmpdir, allow_same_square=True)
    calls = {"count": 0}

    def fake_bus_factory(**kwargs):
        calls["count"] += 1
        raise AssertionError("dry-run should not build a hardware bus")

    exit_code, result = open_loop_pick_place.run(args, bus_factory=fake_bus_factory)
    assert exit_code == 0
    assert result["dry_run"] is True
    assert calls["count"] == 0


def test_real_mode_requires_exact_confirmation_text(tmpdir):
    args = _make_args(tmpdir, real=True, allow_same_square=True, confirm_text="WRONG")
    with pytest.raises(open_loop_pick_place.OpenLoopPickPlaceError):
        open_loop_pick_place.validate_inputs(args)


def test_json_output_writer_works_in_dry_run(tmpdir):
    args = _make_args(tmpdir, allow_same_square=True)
    exit_code, result = open_loop_pick_place.run(args)
    assert exit_code == 0
    with open(args.output_json, "r") as handle:
        payload = json.load(handle)
    assert payload["source"] == "c3"
    assert payload["dest"] == "c3"
    assert payload["real"] is False
    assert payload["per_stage_results"]
    assert result["completed_at"] is not None


def test_abort_result_structure_is_valid(tmpdir):
    args = _make_args(
        tmpdir,
        real=True,
        allow_same_square=True,
        confirm_text=open_loop_pick_place.EXPECTED_CONFIRM_TEXT,
        pause_each=True,
    )
    bus = FakeBus()

    def fake_bus_factory(**kwargs):
        return bus

    def fake_config_loader(path):
        with open(path, "r") as handle:
            return yaml.safe_load(handle)

    def fake_pause_input(prompt):
        return "q"

    exit_code, result = open_loop_pick_place.run(
        args,
        bus_factory=fake_bus_factory,
        config_loader=fake_config_loader,
        sleep_fn=lambda seconds: None,
        pause_input_fn=fake_pause_input,
    )
    assert exit_code == 1
    assert result["aborted"] is True
    assert result["abort_reason"] == "operator_abort_at_pause_before_pick_descent"
    assert result["final_torque_disable_attempted"] is True
    assert result["final_torque_disable_success"] is True
    assert bus.closed is True


def test_generated_above_pose_causes_warning_but_not_failure(tmpdir):
    document = _targets(source_above_source="generated", dest_above_source="generated")
    paths = _paths(tmpdir, targets=document)
    args = _make_args(tmpdir, paths=paths, allow_same_square=True)
    validation = open_loop_pick_place.validate_inputs(args)
    assert validation["warnings"]
    assert any("above_pose source is generated" in warning for warning in validation["warnings"])


def test_generated_pick_or_place_pose_causes_refusal(tmpdir):
    document = _targets(source_pick_source="generated")
    paths = _paths(tmpdir, targets=document)
    args = _make_args(tmpdir, paths=paths)
    with pytest.raises(open_loop_pick_place.OpenLoopPickPlaceError):
        open_loop_pick_place.validate_inputs(args)


def test_return_home_after_adds_final_home_stage(tmpdir):
    args = _make_args(tmpdir, allow_same_square=True, return_home_after=True)
    validation = open_loop_pick_place.validate_inputs(args)
    validation["step_size_ticks"] = args.step_size_ticks
    validation["gripper_step_size_ticks"] = args.gripper_step_size_ticks
    stages = open_loop_pick_place.build_stage_sequence(validation, pause_each=False)
    assert stages[-1]["name"] == "move_home_after_place"
    assert stages[-1]["kind"] == "arm"
    assert stages[-1]["target_pose_name"] == "home_pose"


def test_home_pose_is_validated_before_motion(tmpdir):
    bad_home = _home_pose(seed=5000)
    paths = _paths(tmpdir, home_pose=bad_home)
    args = _make_args(tmpdir, paths=paths, allow_same_square=True, return_home_after=True)
    with pytest.raises(open_loop_pick_place.OpenLoopPickPlaceError) as excinfo:
        open_loop_pick_place.validate_inputs(args)
    assert "home_pose" in str(excinfo.value)


def test_missing_home_pose_fails_clearly(tmpdir):
    paths = _paths(tmpdir)
    args = _make_args(
        tmpdir,
        paths=paths,
        allow_same_square=True,
        return_home_after=True,
        home_pose=str(tmpdir.join("missing_home_pose.yaml")),
    )
    with pytest.raises(open_loop_pick_place.OpenLoopPickPlaceError) as excinfo:
        open_loop_pick_place.validate_inputs(args)
    assert "home pose file does not exist" in str(excinfo.value)


def test_return_home_dry_run_does_not_touch_hardware(tmpdir):
    args = _make_args(tmpdir, allow_same_square=True, return_home_after=True)
    calls = {"count": 0}

    def fake_bus_factory(**kwargs):
        calls["count"] += 1
        raise AssertionError("dry-run should not build a hardware bus")

    exit_code, result = open_loop_pick_place.run(args, bus_factory=fake_bus_factory)
    assert exit_code == 0
    assert calls["count"] == 0
    assert result["return_home_after"] is True
    assert result["sequence_stages"][-1] == "move_home_after_place"
