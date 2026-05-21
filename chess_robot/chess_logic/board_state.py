"""Small wrapper around python-chess board state."""

import os

import chess


class ChessBoardState(object):
    """Convenience wrapper for a chess.Board instance."""

    def __init__(self, fen=None):
        if fen and fen != "startpos":
            self._board = chess.Board(fen)
        else:
            self._board = chess.Board()

    @property
    def board(self):
        return self._board

    def get_board(self):
        return self._board

    def fen(self):
        return self._board.fen()

    def legal_moves(self):
        return list(self._board.legal_moves)

    def load_fen_file(self, path, default_startpos=True):
        """Load board state from a FEN file path."""
        if not path:
            raise ValueError("FEN file path is required")
        if not os.path.exists(path):
            if default_startpos:
                self._board = chess.Board()
                return self
            raise IOError("FEN file does not exist: {}".format(path))

        with open(path, "r") as f:
            text = f.read().strip()

        if not text and default_startpos:
            self._board = chess.Board()
        elif text == "startpos":
            self._board = chess.Board()
        else:
            self._board = chess.Board(text)
        return self

    def save_fen_file(self, path):
        """Persist current board FEN to disk."""
        if not path:
            raise ValueError("FEN file path is required")
        directory = os.path.dirname(path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        with open(path, "w") as f:
            f.write(self.fen())
            f.write("\n")
        return path

    def push_uci(self, uci):
        move = chess.Move.from_uci(uci)
        if move not in self._board.legal_moves:
            raise ValueError("Illegal move for current board: {}".format(uci))
        self._board.push(move)
        return move

    def san(self, move):
        return self._board.san(move)

    def piece_at(self, square):
        idx = chess.SQUARE_NAMES.index(square.lower())
        return self._board.piece_at(idx)
