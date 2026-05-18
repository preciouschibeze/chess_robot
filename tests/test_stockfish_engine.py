import os

import chess

from chess_robot.chess_logic.stockfish_engine import StockfishConfig, StockfishEngine
from tools.select_robot_move import maybe_apply_move


class _FakeStream(object):
    def __init__(self, lines=None):
        self._lines = list(lines or [])
        self.writes = []

    def readline(self):
        if self._lines:
            return self._lines.pop(0)
        return ""

    def write(self, text):
        self.writes.append(text)

    def flush(self):
        return None


class _FakePopen(object):
    def __init__(self, lines):
        self.stdin = _FakeStream()
        self.stdout = _FakeStream(lines)

    def wait(self, timeout=None):
        return 0


def test_parse_bestmove_line():
    assert StockfishEngine.parse_bestmove("bestmove e7e5") == "e7e5"


def test_select_move_legal_from_black_after_e2e4(monkeypatch):
    fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1"

    def fake_popen(*args, **kwargs):
        lines = ["id name Mockfish\n", "uciok\n", "readyok\n", "bestmove e7e5\n"]
        return _FakePopen(lines)

    monkeypatch.setattr("chess_robot.chess_logic.stockfish_engine.subprocess.Popen", fake_popen)

    engine = StockfishEngine(StockfishConfig(engine_path="mockfish", depth=6, movetime_ms=None))
    result = engine.select_move(fen)
    assert result.move_uci == "e7e5"
    assert result.legal is True
    assert result.error is None


def test_select_move_rejects_illegal_for_black_to_move(monkeypatch):
    fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1"

    def fake_popen(*args, **kwargs):
        lines = ["uciok\n", "readyok\n", "bestmove e2e4\n"]
        return _FakePopen(lines)

    monkeypatch.setattr("chess_robot.chess_logic.stockfish_engine.subprocess.Popen", fake_popen)

    engine = StockfishEngine(StockfishConfig(engine_path="mockfish"))
    result = engine.select_move(fen)
    assert result.move_uci == "e2e4"
    assert result.legal is False
    assert "illegal move" in result.error.lower()


def test_missing_engine_path_returns_clear_error(monkeypatch):
    fen = chess.Board().fen()

    def missing_popen(*args, **kwargs):
        raise OSError("No such file or directory")

    monkeypatch.setattr("chess_robot.chess_logic.stockfish_engine.subprocess.Popen", missing_popen)

    engine = StockfishEngine(StockfishConfig(engine_path="missing-engine"))
    result = engine.select_move(fen)
    assert result.legal is False
    assert result.error == "Stockfish binary not found. Install stockfish or pass --engine-path."


def test_apply_helper_does_not_save_when_apply_false(tmp_path):
    fen = chess.Board().fen()
    out_file = tmp_path / "fen.txt"
    fen_after, applied, saved_to = maybe_apply_move(fen, "e2e4", False, str(out_file), None)
    assert fen_after is None
    assert applied is False
    assert saved_to is None
    assert os.path.exists(str(out_file)) is False
