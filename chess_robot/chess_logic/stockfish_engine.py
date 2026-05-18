"""Minimal Stockfish UCI wrapper for robot move selection dry-runs."""

from __future__ import print_function

import subprocess

import chess


class EngineError(Exception):
    """Raised when the UCI engine cannot be started or queried."""


class StockfishConfig(object):
    def __init__(self, engine_path="stockfish", depth=6, movetime_ms=None):
        self.engine_path = engine_path or "stockfish"
        self.depth = depth if depth is not None else 6
        self.movetime_ms = movetime_ms


class EngineMoveResult(object):
    def __init__(
        self,
        move_uci=None,
        fen_before=None,
        depth=6,
        movetime_ms=None,
        engine_path="stockfish",
        raw_bestmove_line=None,
        legal=False,
        error=None,
    ):
        self.move_uci = move_uci
        self.fen_before = fen_before
        self.depth = depth
        self.movetime_ms = movetime_ms
        self.engine_path = engine_path
        self.raw_bestmove_line = raw_bestmove_line
        self.legal = legal
        self.error = error


class StockfishEngine(object):
    def __init__(self, config=None):
        self.config = config or StockfishConfig()

    @staticmethod
    def parse_bestmove(line):
        if not line:
            return None
        text = line.strip()
        if not text.startswith("bestmove "):
            return None
        parts = text.split()
        if len(parts) < 2:
            return None
        move_uci = parts[1].strip()
        if not move_uci or move_uci == "(none)":
            return None
        return move_uci

    def _send(self, proc, text):
        proc.stdin.write(text + "\n")
        proc.stdin.flush()

    def _read_until(self, proc, token):
        while True:
            line = proc.stdout.readline()
            if not line:
                raise EngineError("Engine exited before '{}'".format(token))
            stripped = line.strip()
            if stripped == token:
                return stripped

    def _read_bestmove(self, proc):
        while True:
            line = proc.stdout.readline()
            if not line:
                return None
            stripped = line.strip()
            if stripped.startswith("bestmove "):
                return stripped

    def select_move(self, fen):
        result = EngineMoveResult(
            fen_before=fen,
            depth=self.config.depth,
            movetime_ms=self.config.movetime_ms,
            engine_path=self.config.engine_path,
        )

        proc = None
        try:
            proc = subprocess.Popen(
                [self.config.engine_path],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
            )

            self._send(proc, "uci")
            self._read_until(proc, "uciok")
            self._send(proc, "isready")
            self._read_until(proc, "readyok")

            self._send(proc, "position fen {}".format(fen))
            if self.config.movetime_ms is not None:
                self._send(proc, "go movetime {}".format(int(self.config.movetime_ms)))
            else:
                self._send(proc, "go depth {}".format(int(self.config.depth)))

            bestmove_line = self._read_bestmove(proc)
            result.raw_bestmove_line = bestmove_line
            result.move_uci = self.parse_bestmove(bestmove_line)

            if not result.move_uci:
                result.error = "Engine did not return a bestmove"
                result.legal = False
                return result

            board = chess.Board(fen)
            try:
                move_obj = chess.Move.from_uci(result.move_uci)
            except ValueError:
                result.error = "Engine returned invalid UCI move: {}".format(result.move_uci)
                result.legal = False
                return result

            if move_obj in board.legal_moves:
                result.legal = True
                return result

            result.legal = False
            result.error = "Engine returned illegal move for position: {}".format(result.move_uci)
            return result

        except OSError:
            result.error = "Stockfish binary not found. Install stockfish or pass --engine-path."
            result.legal = False
            return result
        except Exception as exc:
            result.error = str(exc)
            result.legal = False
            return result
        finally:
            if proc is not None:
                try:
                    self._send(proc, "quit")
                except Exception:
                    pass
                try:
                    proc.wait(timeout=1)
                except Exception:
                    pass
