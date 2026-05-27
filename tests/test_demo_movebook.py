import os
import tempfile

import pytest
import yaml

from chess_robot.chess_logic.demo_movebook import DemoMovebook, DemoMovebookError


def _write_yaml(path, payload):
    with open(path, "w") as handle:
        yaml.safe_dump(payload, handle, default_flow_style=False)


def test_load_valid_movebook_and_lookup_reply():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "movebook.yaml")
        _write_yaml(path, {"demo_movebook": {"e2e4": "e7e5", "g1f3": "b8c6"}})

        movebook = DemoMovebook.from_path(path)

        assert movebook.robot_reply("e2e4") == "e7e5"
        assert movebook.robot_reply("G1F3") == "b8c6"


def test_invalid_uci_key_rejected():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "movebook.yaml")
        _write_yaml(path, {"demo_movebook": {"bad_move": "e7e5"}})

        with pytest.raises(DemoMovebookError):
            DemoMovebook.from_path(path)


def test_missing_reply_raises_clear_error():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "movebook.yaml")
        _write_yaml(path, {"demo_movebook": {"e2e4": "e7e5"}})

        movebook = DemoMovebook.from_path(path)

        with pytest.raises(DemoMovebookError):
            movebook.robot_reply("d2d4")
