import json
import os
import subprocess
import tempfile

from chess_robot.chess_logic.board_state import ChessBoardState


SCRIPT = os.path.join(os.path.dirname(__file__), "..", "tools", "match_human_move.py")


def _write_json(path, payload):
    with open(path, "w") as f:
        json.dump(payload, f)


def _run_cli(args, stdin_text=None):
    cmd = ["python", SCRIPT] + args
    proc = subprocess.Popen(
        cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
    )
    out, err = proc.communicate(stdin_text)
    return proc.returncode, out, err


def _transition(changed_squares):
    return {
        "status": "success",
        "transition_type": "move",
        "added_squares": [],
        "removed_squares": [],
        "uncertain_squares": [],
        "changed_squares": changed_squares,
        "summary": {},
        "notes": [],
    }


def test_fen_save_and_load_roundtrip():
    with tempfile.TemporaryDirectory() as tmp:
        fen_path = os.path.join(tmp, "current_fen.txt")
        state = ChessBoardState("startpos")
        state.push_uci("e2e4")
        state.save_fen_file(fen_path)

        loaded = ChessBoardState()
        loaded.load_fen_file(fen_path)
        assert loaded.fen() == state.fen()


def test_unique_apply_preview_does_not_commit_or_write_state():
    with tempfile.TemporaryDirectory() as tmp:
        transition_path = os.path.join(tmp, "transition.json")
        output_path = os.path.join(tmp, "output.json")
        fen_path = os.path.join(tmp, "current_fen.txt")
        history_path = os.path.join(tmp, "history.jsonl")

        _write_json(transition_path, _transition([
            {"square": "e2", "change_type": "removed"},
            {"square": "e4", "change_type": "added"},
        ]))

        rc, _, _ = _run_cli([
            "--transition-result", transition_path,
            "--fen", "startpos",
            "--output", output_path,
            "--history-file", history_path,
            "--save-fen-file", fen_path,
            "--apply",
        ])
        assert rc == 0

        with open(output_path, "r") as f:
            result = json.load(f)

        assert result["status"] == "unique"
        assert result["accepted_move"] == "e2e4"
        assert result["committed"] is False
        assert result["fen_after_if_applied"]
        assert not os.path.exists(fen_path)
        assert not os.path.exists(history_path)


def test_commit_unique_writes_fen_and_history():
    with tempfile.TemporaryDirectory() as tmp:
        transition_path = os.path.join(tmp, "transition.json")
        output_path = os.path.join(tmp, "output.json")
        fen_path = os.path.join(tmp, "current_fen.txt")
        history_path = os.path.join(tmp, "history.jsonl")

        _write_json(transition_path, _transition([
            {"square": "e2", "change_type": "removed"},
            {"square": "e4", "change_type": "added"},
        ]))
        with open(fen_path, "w") as f:
            f.write("startpos\n")

        rc, _, _ = _run_cli([
            "--transition-result", transition_path,
            "--fen-file", fen_path,
            "--save-fen-file", fen_path,
            "--history-file", history_path,
            "--output", output_path,
            "--commit",
        ])
        assert rc == 0

        with open(output_path, "r") as f:
            result = json.load(f)
        with open(fen_path, "r") as f:
            fen_text = f.read().strip()
        with open(history_path, "r") as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]

        assert result["committed"] is True
        assert result["accepted_move"] == "e2e4"
        assert fen_text != "startpos"
        assert len(lines) == 1
        entry = json.loads(lines[0])
        assert entry["move_uci"] == "e2e4"
        assert entry["match_status"] == "unique"


def test_invalid_evidence_never_writes_fen_or_history():
    with tempfile.TemporaryDirectory() as tmp:
        transition_path = os.path.join(tmp, "transition.json")
        output_path = os.path.join(tmp, "output.json")
        fen_path = os.path.join(tmp, "current_fen.txt")
        history_path = os.path.join(tmp, "history.jsonl")

        _write_json(transition_path, _transition([
            {"square": "e4", "change_type": "added"},
        ]))
        with open(fen_path, "w") as f:
            f.write("startpos\n")

        rc, _, _ = _run_cli([
            "--transition-result", transition_path,
            "--fen-file", fen_path,
            "--save-fen-file", fen_path,
            "--history-file", history_path,
            "--output", output_path,
            "--commit",
        ])
        assert rc != 0

        with open(fen_path, "r") as f:
            fen_text = f.read().strip()
        assert fen_text == "startpos"
        assert not os.path.exists(history_path)


def test_ambiguous_without_confirmation_does_not_commit():
    with tempfile.TemporaryDirectory() as tmp:
        transition_path = os.path.join(tmp, "transition.json")
        output_path = os.path.join(tmp, "output.json")
        fen_path = os.path.join(tmp, "current_fen.txt")
        history_path = os.path.join(tmp, "history.jsonl")

        _write_json(transition_path, {
            "status": "success",
            "changed_squares": [{"square": "e4", "change_type": "removed"}],
        })
        with open(fen_path, "w") as f:
            f.write("8/8/3p1p2/8/4N3/8/8/4K2k w - - 0 1\n")

        rc, _, _ = _run_cli([
            "--transition-result", transition_path,
            "--fen-file", fen_path,
            "--save-fen-file", fen_path,
            "--history-file", history_path,
            "--output", output_path,
            "--commit",
        ])
        assert rc != 0

        with open(output_path, "r") as f:
            result = json.load(f)
        assert result["status"] == "ambiguous"
        assert result["committed"] is False
        assert result["selected_move"] is None
        assert not os.path.exists(history_path)


def test_ambiguous_with_confirmation_can_commit_selected_candidate():
    with tempfile.TemporaryDirectory() as tmp:
        transition_path = os.path.join(tmp, "transition.json")
        output_path = os.path.join(tmp, "output.json")
        fen_path = os.path.join(tmp, "current_fen.txt")
        history_path = os.path.join(tmp, "history.jsonl")

        _write_json(transition_path, {
            "status": "success",
            "changed_squares": [{"square": "e4", "change_type": "removed"}],
        })
        with open(fen_path, "w") as f:
            f.write("8/8/3p1p2/8/4N3/8/8/4K2k w - - 0 1\n")

        rc, _, _ = _run_cli([
            "--transition-result", transition_path,
            "--fen-file", fen_path,
            "--save-fen-file", fen_path,
            "--history-file", history_path,
            "--output", output_path,
            "--confirm",
            "--commit",
        ], stdin_text="1\n")
        assert rc == 0

        with open(output_path, "r") as f:
            result = json.load(f)
        with open(history_path, "r") as f:
            entry = json.loads(f.readline())

        assert result["status"] == "ambiguous"
        assert result["selected_move"] in ("e4d6", "e4f6")
        assert result["committed"] is True
        assert entry["manual_confirmation_used"] is True
