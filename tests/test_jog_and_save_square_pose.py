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
import tools.jog_and_save_square_pose as jog_and_save_square_pose


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



def _gripper_profile():
    return {
        "gripper": {
            "joint": "gripper",
            "servo_id": 6,
            "limits": {"min": 1463, "max": 1738},
            "open_position": 1704,
            "grasp_position": 1536,
        }
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
        "notes": ["seed {}".format(seed)],
    }
    if source == "manual":
        entry["recorded_at"] = "2026-05-22T00:00:00Z"
    else:
        entry["generated_at"] = "2026-05-22T00:00:00Z"
    return entry



def _targets(existing_source="generated", include_pose=True, include_above=True):
    document = robot_square_map.default_square_targets()
    square = {}
    if include_above:
        square["above_pose"] = _pose(2000, source="manual", gripper_value=1609)
    if include_pose:
        square["pick_pose"] = _pose(2100, source=existing_source, gripper_value=1610)
        square["place_pose"] = _pose(2200, source=existing_source, gripper_value=1611)
    document["squares"] = {
        "c3": square,
        "d4": {"above_pose": _pose(2300, source="manual", gripper_value=1612)},
    }
    return document



def _write_yaml(tmpdir, name, data):
    path = str(tmpdir.join(name))
    with open(path, "w") as handle:
        yaml.safe_dump(data, handle, default_flow_style=False)
    return path



def _paths(tmpdir, targets=None, joint_limits=None, servo_map=None, robot_config=None, gripper_profile=None):
    gripper_profile_path = _write_yaml(
        tmpdir,
        "gripper_profile.yaml",
        gripper_profile if gripper_profile is not None else _gripper_profile(),
    )
    return {
        "targets": _write_yaml(tmpdir, "square_targets.yaml", targets if targets is not None else _targets()),
        "joint_limits": _write_yaml(tmpdir, "joint_limits.yaml", joint_limits if joint_limits is not None else _joint_limits()),
        "servo_map": _write_yaml(tmpdir, "servo_map.yaml", servo_map if servo_map is not None else _servo_map()),
        "robot_config": _write_yaml(tmpdir, "robot.yaml", robot_config if robot_config is not None else _robot_config()),
        "gripper_profile": gripper_profile_path,
    }



def _make_args(tmpdir, paths=None, **overrides):
    if paths is None:
        paths = _paths(tmpdir)
    values = {
        "square": "c3",
        "pose_name": "pick_pose",
        "targets": paths["targets"],
        "joint_limits": paths["joint_limits"],
        "servo_map": paths["servo_map"],
        "robot_config": paths["robot_config"],
        "real": False,
        "confirm_text": None,
        "move_to_existing_pose": False,
        "approach_through_above": None,
        "write": False,
        "force": False,
        "jog_step": 5,
        "max_single_jog": 10,
        "max_total_delta": 100,
        "step_size_ticks": 5,
        "step_delay": 0.12,
        "settle_time": 0.75,
        "allow_gripper_jog": False,
        "log": str(tmpdir.join("jog_and_save_square_pose.log")),
        "output_json": str(tmpdir.join("result.json")),
    }
    values.update(overrides)
    return type("Args", (object,), values)()



def _load_targets(path):
    return robot_square_map.load_square_targets(path)



def test_allowed_pose_names_are_defined():
    assert robot_square_map.ALLOWED_POSE_NAMES == ["above_pose", "pick_pose", "place_pose"]
    for pose_name in robot_square_map.ALLOWED_POSE_NAMES:
        assert robot_square_map.validate_pose_name(pose_name) == pose_name



def test_invalid_pose_name_is_rejected(tmpdir):
    args = _make_args(tmpdir, pose_name="drop_pose")
    with pytest.raises(jog_and_save_square_pose.JogAndSaveSquarePoseError):
        jog_and_save_square_pose.validate_inputs(args)



def test_dry_run_opens_no_hardware_bus(tmpdir):
    args = _make_args(tmpdir)
    calls = {"count": 0}

    def fake_bus_factory(**kwargs):
        calls["count"] += 1
        raise AssertionError("dry-run should not build a hardware bus")

    exit_code, result = jog_and_save_square_pose.run(args, bus_factory=fake_bus_factory)
    assert exit_code == 0
    assert result["dry_run"] is True
    assert calls["count"] == 0



def test_real_mode_requires_exact_confirmation_text(tmpdir):
    args = _make_args(tmpdir, real=True, confirm_text="WRONG")
    with pytest.raises(jog_and_save_square_pose.JogAndSaveSquarePoseError):
        jog_and_save_square_pose.validate_inputs(args)



def test_gripper_is_excluded_from_joggable_joints_by_default(tmpdir):
    args = _make_args(tmpdir)
    validation = jog_and_save_square_pose.validate_inputs(args)
    assert "gripper" not in validation["joggable_joints"]



def test_gripper_becomes_joggable_only_with_flag(tmpdir):
    args = _make_args(tmpdir, allow_gripper_jog=True)
    validation = jog_and_save_square_pose.validate_inputs(args)
    assert "gripper" in validation["joggable_joints"]



def test_jog_command_parser_accepts_explicit_form():
    command = jog_and_save_square_pose.parse_operator_command(
        "jog shoulder_pan 5",
        current_step=5,
        allowed_joints=list(jog_and_save_square_pose.DEFAULT_MOVEMENT_JOINTS),
        max_single_jog=10,
    )
    assert command == {"action": "jog", "joint": "shoulder_pan", "delta": 5}



def test_jog_command_parser_rejects_delta_above_max_single_jog():
    with pytest.raises(jog_and_save_square_pose.CommandRejected):
        jog_and_save_square_pose.parse_operator_command(
            "jog shoulder_pan 11",
            current_step=5,
            allowed_joints=list(jog_and_save_square_pose.DEFAULT_MOVEMENT_JOINTS),
            max_single_jog=10,
        )



def test_jog_target_outside_joint_limits_is_rejected():
    current = {"shoulder_pan": 2998}
    initial = {"shoulder_pan": 2998}
    limits = {"shoulder_pan": {"provisional_min": 1000, "provisional_max": 3000}}
    with pytest.raises(jog_and_save_square_pose.JogAndSaveSquarePoseError):
        jog_and_save_square_pose.validate_jog_request(
            current,
            initial,
            "shoulder_pan",
            5,
            limits,
            max_total_delta=100,
        )



def test_total_delta_above_max_total_delta_is_rejected():
    current = {"shoulder_pan": 1050}
    initial = {"shoulder_pan": 1000}
    limits = {"shoulder_pan": {"provisional_min": 900, "provisional_max": 3000}}
    with pytest.raises(jog_and_save_square_pose.JogAndSaveSquarePoseError):
        jog_and_save_square_pose.validate_jog_request(
            current,
            initial,
            "shoulder_pan",
            60,
            limits,
            max_total_delta=100,
        )



def test_save_refuses_overwriting_manual_pose_without_force(tmpdir):
    document = _targets(existing_source="manual")
    current = _pose(2600, source="manual", gripper_value=1617)["joints"]
    with pytest.raises(robot_square_map.SquareTargetError):
        jog_and_save_square_pose.prepare_save_document(
            document,
            "c3",
            "pick_pose",
            current,
            write_enabled=True,
            force=False,
        )



def test_save_overwrites_manual_pose_with_force(tmpdir):
    document = _targets(existing_source="manual")
    current = _pose(2600, source="manual", gripper_value=1617)["joints"]
    prepared = jog_and_save_square_pose.prepare_save_document(
        document,
        "c3",
        "pick_pose",
        current,
        write_enabled=True,
        force=True,
    )
    assert prepared["entry"]["joints"] == current
    assert prepared["saved"] is True



def test_save_preserves_unrelated_poses_and_squares(tmpdir):
    document = _targets(existing_source="generated")
    current = _pose(2600, source="manual", gripper_value=1617)["joints"]
    prepared = jog_and_save_square_pose.prepare_save_document(
        document,
        "c3",
        "pick_pose",
        current,
        write_enabled=True,
        force=False,
    )
    updated = prepared["document"]
    assert updated["squares"]["c3"]["above_pose"] == document["squares"]["c3"]["above_pose"]
    assert updated["squares"]["d4"] == document["squares"]["d4"]



def test_save_includes_gripper_readback_even_when_gripper_not_joggable(tmpdir):
    args = _make_args(tmpdir, write=True)
    validation = jog_and_save_square_pose.validate_inputs(args)
    state = jog_and_save_square_pose.SessionState(args.jog_step)
    current = _pose(2600, source="manual", gripper_value=1677)["joints"]
    result = jog_and_save_square_pose.build_result_template(args, validation)
    prepared = jog_and_save_square_pose._perform_save(
        args,
        validation,
        state,
        validation["document"],
        current,
        result,
    )
    assert prepared["entry"]["joints"]["gripper"] == 1677



def test_move_to_existing_pose_builds_startup_movement_sequence(tmpdir):
    args = _make_args(tmpdir, move_to_existing_pose=True, approach_through_above=True)
    validation = jog_and_save_square_pose.validate_inputs(args)
    current = {
        "shoulder_pan": 1500,
        "shoulder_lift": 1500,
        "elbow_flex": 1500,
        "wrist_flex": 1500,
        "wrist_roll": 1500,
    }
    stages = jog_and_save_square_pose.build_startup_plan(
        current,
        validation["startup_targets"],
        args.step_size_ticks,
        validation["joint_limits"],
    )
    assert len(stages) == 2
    assert stages[0]["pose_name"] == "above_pose"
    assert stages[1]["pose_name"] == "pick_pose"



def test_approach_through_above_requires_above_pose(tmpdir):
    paths = _paths(tmpdir, targets=_targets(existing_source="generated", include_pose=True, include_above=False))
    args = _make_args(tmpdir, paths=paths, move_to_existing_pose=True, approach_through_above=True)
    with pytest.raises(jog_and_save_square_pose.JogAndSaveSquarePoseError):
        jog_and_save_square_pose.validate_inputs(args)



def test_json_writer_works_in_dry_run(tmpdir):
    args = _make_args(tmpdir, output_json=str(tmpdir.join("dry_run.json")))
    exit_code, result = jog_and_save_square_pose.run(args)
    assert exit_code == 0
    with open(args.output_json, "r") as handle:
        payload = json.load(handle)
    assert payload["dry_run"] is True
    assert payload["output_written"] is True
    assert payload["square"] == "c3"
    assert result["completed_at"] is not None



def test_abort_quit_result_structure_is_valid(tmpdir):
    args = _make_args(tmpdir, real=True, confirm_text="POWERED JOG SAVE")

    class FakeBus(object):
        def __init__(self):
            self.positions = {1: 2001, 2: 2002, 3: 2003, 4: 2004, 5: 2005, 6: 1613}
            self.torque_calls = []
            self.closed = False

        def read_position(self, servo_id):
            return self.positions.get(int(servo_id))

        def read_register(self, servo_id, address, length):
            if int(address) == 65 and int(length) == 1:
                return 0
            return 0

        def write_goal_position(self, servo_id, goal_position):
            self.positions[int(servo_id)] = int(goal_position)

        def torque_enable(self, servo_id, enabled):
            self.torque_calls.append((int(servo_id), bool(enabled)))

        def close(self):
            self.closed = True

    bus = FakeBus()

    def fake_bus_factory(**kwargs):
        return bus

    exit_code, result = jog_and_save_square_pose.run(
        args,
        bus_factory=fake_bus_factory,
        input_fn=lambda prompt="": "q",
        sleep_fn=lambda seconds: None,
    )
    assert exit_code == 1
    assert result["aborted"] is True
    assert result["abort_reason"] == "operator_quit"
    assert result["final_torque_disable_attempted"] is True
    assert result["final_torque_disable_success"] is True
    assert bus.closed is True



def test_startup_intermediate_stepping_respects_step_size_ticks():
    current = {"shoulder_pan": 1000, "elbow_flex": 1000}
    targets = [{"label": "move", "pose_name": "pick_pose", "target_joints": {"shoulder_pan": 1065, "elbow_flex": 1035}}]
    stages = jog_and_save_square_pose.build_startup_plan(
        current,
        targets,
        step_size_ticks=20,
        joint_limits={
            "shoulder_pan": {"provisional_min": 900, "provisional_max": 3000},
            "elbow_flex": {"provisional_min": 900, "provisional_max": 3000},
        },
    )
    previous = dict(current)
    for pose in stages[0]["steps"]:
        for joint_name in pose:
            assert abs(int(pose[joint_name]) - int(previous[joint_name])) <= 20
        previous = pose
    assert stages[0]["steps"][-1] == targets[0]["target_joints"]



def test_current_readback_validation_rejects_out_of_limit_current_positions(tmpdir):
    args = _make_args(tmpdir)
    validation = jog_and_save_square_pose.validate_inputs(args)
    bad_current = {
        "shoulder_pan": 9999,
        "shoulder_lift": 2000,
        "elbow_flex": 2000,
        "wrist_flex": 2000,
        "wrist_roll": 2000,
        "gripper": 1600,
    }
    with pytest.raises(jog_and_save_square_pose.JogAndSaveSquarePoseError):
        jog_and_save_square_pose._validate_current_positions(
            bad_current,
            validation["joint_limits"],
            validation["gripper_limits"],
        )
