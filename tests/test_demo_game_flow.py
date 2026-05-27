import os
import tempfile

import pytest
import yaml

from chess_robot.chess_logic.board_state import ChessBoardState
from chess_robot.calibration import robot_square_map
from tools.run_demo_game import (
    DemoRunError,
    resolve_mode,
    validate_square_targets_for_move,
    verify_move_is_supported_quiet,
)


class Args(object):
    def __init__(self, dry_run=False, vision_only=False, real=False):
        self.dry_run = dry_run
        self.vision_only = vision_only
        self.real = real


def _write_yaml(path, payload):
    with open(path, "w") as handle:
        yaml.safe_dump(payload, handle, default_flow_style=False)


def test_resolve_mode_defaults_to_dry_run():
    assert resolve_mode(Args()) == "dry_run"


def test_resolve_mode_rejects_multiple_flags():
    with pytest.raises(DemoRunError):
        resolve_mode(Args(dry_run=True, real=True))


def test_verify_move_rejects_capture_for_demo_scope():
    board = ChessBoardState("8/8/8/3p4/4P3/8/8/4K2k w - - 0 1")
    with pytest.raises(DemoRunError):
        verify_move_is_supported_quiet(board, "e4d5", "Human")


def test_validate_square_targets_requires_source_and_destination_poses():
    with tempfile.TemporaryDirectory() as tmp:
        targets = robot_square_map.default_square_targets()
        targets["squares"] = {
            "e7": {
                "above_pose": {"joints": {}, "source": "manual"},
                "pick_pose": {"joints": {}, "source": "manual"},
            },
            "e5": {
                "above_pose": {"joints": {}, "source": "manual"},
                "place_pose": {"joints": {}, "source": "manual"},
            },
        }
        path = os.path.join(tmp, "square_targets.yaml")
        _write_yaml(path, targets)

        result = validate_square_targets_for_move(path, "e7e5")

        assert result["source"] == "e7"
        assert result["destination"] == "e5"


def test_validate_square_targets_rejects_missing_destination_place_pick():
    with tempfile.TemporaryDirectory() as tmp:
        targets = robot_square_map.default_square_targets()
        targets["squares"] = {
            "e7": {
                "above_pose": {"joints": {}, "source": "manual"},
                "pick_pose": {"joints": {}, "source": "manual"},
            },
            "e5": {
                "above_pose": {"joints": {}, "source": "manual"},
            },
        }
        path = os.path.join(tmp, "square_targets.yaml")
        _write_yaml(path, targets)

        with pytest.raises(DemoRunError):
            validate_square_targets_for_move(path, "e7e5")
