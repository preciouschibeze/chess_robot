import io
import os
import tempfile

import yaml

from tools.audit_demo_movebook_calibration import run_audit


def _write_yaml(path, payload):
    with open(path, "w") as handle:
        yaml.safe_dump(payload, handle, default_flow_style=False)


def _movebook_payload():
    return {
        "demo_movebook": {
            "e2e4": "e7e5",
            "d2d4": "d7d5",
        }
    }


def _pose_entry():
    return {"source": "manual", "joints": {"shoulder_pan": 1}}


def _square_targets_payload(include_missing_source_pick=False, include_missing_dest_place=False):
    squares = {
        "e7": {"above_pose": _pose_entry(), "pick_pose": _pose_entry()},
        "e5": {"above_pose": _pose_entry(), "place_pose": _pose_entry()},
        "d7": {"above_pose": _pose_entry(), "pick_pose": _pose_entry()},
        "d5": {"above_pose": _pose_entry(), "place_pose": _pose_entry()},
    }
    if include_missing_source_pick:
        del squares["e7"]["pick_pose"]
    if include_missing_dest_place:
        del squares["d5"]["place_pose"]
    return {
        "board_orientation": "robot_black_side",
        "joint_order": [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ],
        "pose_type": "joint_ticks",
        "squares": squares,
    }


def test_all_required_poses_present_returns_success():
    with tempfile.TemporaryDirectory() as tmp:
        movebook_path = os.path.join(tmp, "movebook.yaml")
        targets_path = os.path.join(tmp, "square_targets.yaml")
        _write_yaml(movebook_path, _movebook_payload())
        _write_yaml(targets_path, _square_targets_payload())

        stream = io.StringIO()
        code, report = run_audit(movebook_path, targets_path, stream=stream)

        assert code == 0
        assert report is not None
        assert report["ok"] is True
        assert report["missing"] == []


def test_missing_source_pick_pose_returns_failure():
    with tempfile.TemporaryDirectory() as tmp:
        movebook_path = os.path.join(tmp, "movebook.yaml")
        targets_path = os.path.join(tmp, "square_targets.yaml")
        _write_yaml(movebook_path, _movebook_payload())
        _write_yaml(targets_path, _square_targets_payload(include_missing_source_pick=True))

        stream = io.StringIO()
        code, report = run_audit(movebook_path, targets_path, stream=stream)

        assert code == 1
        assert report is not None
        assert "e7.pick_pose" in report["missing"]


def test_missing_destination_place_pose_returns_failure():
    with tempfile.TemporaryDirectory() as tmp:
        movebook_path = os.path.join(tmp, "movebook.yaml")
        targets_path = os.path.join(tmp, "square_targets.yaml")
        _write_yaml(movebook_path, _movebook_payload())
        _write_yaml(targets_path, _square_targets_payload(include_missing_dest_place=True))

        stream = io.StringIO()
        code, report = run_audit(movebook_path, targets_path, stream=stream)

        assert code == 1
        assert report is not None
        assert "d5.place_pose" in report["missing"]


def test_malformed_movebook_move_returns_failure():
    with tempfile.TemporaryDirectory() as tmp:
        movebook_path = os.path.join(tmp, "movebook.yaml")
        targets_path = os.path.join(tmp, "square_targets.yaml")
        _write_yaml(movebook_path, {"demo_movebook": {"e2e4": "bad_move"}})
        _write_yaml(targets_path, _square_targets_payload())

        stream = io.StringIO()
        code, report = run_audit(movebook_path, targets_path, stream=stream)

        assert code != 0
        assert report is None
