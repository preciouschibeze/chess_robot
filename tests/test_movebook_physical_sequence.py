from __future__ import absolute_import

import os
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import tools.run_movebook_physical_sequence as sequence


class FakeOpenLoopModule(object):
    EXPECTED_CONFIRM_TEXT = "PICK PLACE BASELINE"

    def __init__(self, outcomes=None):
        self.outcomes = list(outcomes or [(0, {"aborted": False})])
        self.validate_calls = []
        self.run_calls = []

    def validate_inputs(self, args):
        self.validate_calls.append(args)

    def run(self, args):
        self.run_calls.append(args)
        if self.outcomes:
            return self.outcomes.pop(0)
        return 0, {"aborted": False}


def _pose(seed):
    return {
        "source": "manual",
        "joints": {
            "shoulder_pan": seed + 1,
            "shoulder_lift": seed + 2,
            "elbow_flex": seed + 3,
            "wrist_flex": seed + 4,
            "wrist_roll": seed + 5,
            "gripper": seed + 6,
        },
    }


def _targets(robot_moves):
    squares = {}
    seed = 1000
    for move in robot_moves:
        source = move[:2]
        dest = move[2:4]
        squares[source] = {
            "above_pose": _pose(seed),
            "pick_pose": _pose(seed + 10),
        }
        seed += 100
        squares[dest] = {
            "above_pose": _pose(seed),
            "place_pose": _pose(seed + 10),
        }
        seed += 100
    return {"squares": squares}


def _write_yaml(tmpdir, name, data):
    path = str(tmpdir.join(name))
    with open(path, "w") as handle:
        yaml.safe_dump(data, handle, default_flow_style=False)
    return path


def _write_movebook(tmpdir):
    path = str(tmpdir.join("demo_movebook.yaml"))
    with open(path, "w") as handle:
        handle.write("demo_movebook:\n")
        handle.write("  e2e4: a1a2\n")
        handle.write("  g1f3: b1b2\n")
        handle.write("  f1c4: c1c2\n")
    return path


def _write_required_files(tmpdir, targets=None):
    movebook = _write_movebook(tmpdir)
    paths = {
        "movebook": movebook,
        "square_targets": _write_yaml(
            tmpdir,
            "square_targets.yaml",
            targets if targets is not None else _targets(["a1a2", "b1b2", "c1c2"]),
        ),
        "joint_limits": _write_yaml(tmpdir, "joint_limits.yaml", {"limits": {}}),
        "servo_map": _write_yaml(tmpdir, "servo_map.yaml", {"joints": {}}),
        "gripper_profile": _write_yaml(tmpdir, "gripper_profile.yaml", {"gripper": {}}),
        "robot_config": _write_yaml(tmpdir, "robot.yaml", {"servo_bus": {}}),
        "home_pose": _write_yaml(tmpdir, "home_pose.yaml", {"joints": {}}),
        "open_loop_log": str(tmpdir.join("movebook.log")),
        "output_json": str(tmpdir.join("last.json")),
    }
    return paths


def _args(tmpdir, **overrides):
    paths = overrides.pop("paths", None) or _write_required_files(tmpdir)
    args = sequence.build_parser().parse_args([
        "--movebook", paths["movebook"],
        "--square-targets", paths["square_targets"],
        "--joint-limits", paths["joint_limits"],
        "--servo-map", paths["servo_map"],
        "--gripper-profile", paths["gripper_profile"],
        "--robot-config", paths["robot_config"],
        "--home-pose", paths["home_pose"],
        "--open-loop-log", paths["open_loop_log"],
        "--output-json", paths["output_json"],
    ])
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def _input_from(values):
    values = list(values)

    def fake_input(prompt):
        if not values:
            return ""
        return values.pop(0)

    return fake_input


def test_movebook_robot_replies_are_extracted_in_order(tmpdir):
    path = _write_movebook(tmpdir)
    entries = sequence.load_movebook_entries(path)
    assert [entry["robot_move"] for entry in entries] == ["a1a2", "b1b2", "c1c2"]
    assert [entry["expected_human_move"] for entry in entries] == ["e2e4", "g1f3", "f1c4"]


def test_default_wrapper_speed_settings_are_faster():
    args = sequence.build_parser().parse_args([])
    assert args.step_size_ticks == 15
    assert args.gripper_step_size_ticks == 15
    assert args.step_delay == 0.05
    assert args.gripper_step_delay == 0.03
    assert args.settle_time == 0.5


def test_dry_run_does_not_call_hardware_executor(tmpdir):
    fake = FakeOpenLoopModule()
    exit_code, records = sequence.run_sequence(_args(tmpdir), open_loop_module=fake)
    assert exit_code == 0
    assert len(records) == 3
    assert fake.validate_calls == []
    assert fake.run_calls == []


def test_missing_source_pick_pose_fails(tmpdir):
    targets = _targets(["a1a2"])
    del targets["squares"]["a1"]["pick_pose"]
    paths = _write_required_files(tmpdir, targets=targets)
    args = _args(tmpdir, paths=paths, max_moves=1)
    with pytest.raises(sequence.MovebookPhysicalSequenceError) as excinfo:
        sequence.validate_calibration_for_move(args.square_targets, "a1a2")
    assert "missing a1.pick_pose" in str(excinfo.value)


def test_missing_destination_place_pose_fails(tmpdir):
    targets = _targets(["a1a2"])
    del targets["squares"]["a2"]["place_pose"]
    paths = _write_required_files(tmpdir, targets=targets)
    args = _args(tmpdir, paths=paths, max_moves=1)
    with pytest.raises(sequence.MovebookPhysicalSequenceError) as excinfo:
        sequence.validate_calibration_for_move(args.square_targets, "a1a2")
    assert "missing a2.place_pose" in str(excinfo.value)


def test_start_index_works(tmpdir):
    args = _args(tmpdir, start_index=2, max_moves=1)
    exit_code, records = sequence.run_sequence(args, open_loop_module=FakeOpenLoopModule())
    assert exit_code == 0
    assert [record["robot_move"] for record in records] == ["b1b2"]
    assert records[0]["move_index"] == 2


def test_max_moves_works(tmpdir):
    args = _args(tmpdir, max_moves=1)
    exit_code, records = sequence.run_sequence(args, open_loop_module=FakeOpenLoopModule())
    assert exit_code == 0
    assert [record["robot_move"] for record in records] == ["a1a2"]


def test_enter_to_advance_accepts_empty_enter(tmpdir):
    args = _args(tmpdir, real=True, enter_to_advance=True, max_moves=1)
    fake = FakeOpenLoopModule()
    exit_code, records = sequence.run_sequence(
        args,
        input_fn=_input_from(["", ""]),
        open_loop_module=fake,
    )
    assert exit_code == 0
    assert records[0]["execution_success"] is True
    assert len(fake.run_calls) == 1


def test_enter_to_advance_aborts_on_q(tmpdir):
    args = _args(tmpdir, real=True, enter_to_advance=True, max_moves=1)
    fake = FakeOpenLoopModule()
    exit_code, records = sequence.run_sequence(
        args,
        input_fn=_input_from(["", "q"]),
        open_loop_module=fake,
    )
    assert exit_code == 1
    assert records[0]["execution_attempted"] is False
    assert fake.run_calls == []
    assert "operator aborted" in records[0]["failure_reason"]


def test_old_typed_phrase_confirmation_still_works(tmpdir):
    args = _args(tmpdir, real=True, max_moves=1)
    fake = FakeOpenLoopModule()
    exit_code, records = sequence.run_sequence(
        args,
        input_fn=_input_from(["HUMAN_DONE e2e4", "EXECUTE a1a2"]),
        open_loop_module=fake,
    )
    assert exit_code == 0
    assert records[0]["execution_success"] is True
    assert len(fake.run_calls) == 1


def test_real_mode_rejects_wrong_typed_confirmation(tmpdir):
    args = _args(tmpdir, real=True, max_moves=1)
    fake = FakeOpenLoopModule()
    exit_code, records = sequence.run_sequence(
        args,
        input_fn=_input_from(["WRONG"]),
        open_loop_module=fake,
    )
    assert exit_code == 1
    assert records[0]["execution_attempted"] is False
    assert fake.run_calls == []
    assert "expected confirmation HUMAN_DONE e2e4" in records[0]["failure_reason"]


def test_wrapper_passes_return_home_after_to_open_loop(tmpdir):
    args = _args(tmpdir, real=True, max_moves=1)
    fake = FakeOpenLoopModule()
    exit_code, records = sequence.run_sequence(
        args,
        input_fn=_input_from(["HUMAN_DONE e2e4", "EXECUTE a1a2"]),
        open_loop_module=fake,
    )
    assert exit_code == 0
    assert records[0]["execution_success"] is True
    assert fake.validate_calls[0].return_home_after is True
    assert fake.run_calls[0].return_home_after is True
    assert fake.run_calls[0].step_size_ticks == 15
    assert fake.run_calls[0].gripper_step_size_ticks == 15


def test_failure_stops_sequence_unless_continue_on_failure_is_set(tmpdir):
    args = _args(tmpdir, real=True)
    fake = FakeOpenLoopModule(outcomes=[
        (1, {"aborted": True, "abort_reason": "failed_first"}),
        (0, {"aborted": False}),
    ])
    exit_code, records = sequence.run_sequence(
        args,
        input_fn=_input_from(["HUMAN_DONE e2e4", "EXECUTE a1a2"]),
        open_loop_module=fake,
    )
    assert exit_code == 1
    assert len(records) == 1
    assert len(fake.run_calls) == 1

    args = _args(tmpdir, real=True, continue_on_failure=True)
    fake = FakeOpenLoopModule(outcomes=[
        (1, {"aborted": True, "abort_reason": "failed_first"}),
        (0, {"aborted": False}),
    ])
    exit_code, records = sequence.run_sequence(
        args,
        input_fn=_input_from([
            "HUMAN_DONE e2e4",
            "EXECUTE a1a2",
            "HUMAN_DONE g1f3",
            "EXECUTE b1b2",
            "HUMAN_DONE f1c4",
            "EXECUTE c1c2",
        ]),
        open_loop_module=fake,
    )
    assert exit_code == 1
    assert len(records) == 3
    assert len(fake.run_calls) == 3
    assert records[0]["execution_success"] is False
    assert records[1]["execution_success"] is True
