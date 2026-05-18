#!/usr/bin/env python
"""Select a dry-run robot move from current symbolic FEN using Stockfish."""

from __future__ import print_function

import argparse
import json
import os
import sys

import chess

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.chess_logic.stockfish_engine import StockfishConfig, StockfishEngine


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fen", default=None, help="FEN string or startpos")
    parser.add_argument("--fen-file", default="data/game/current_fen.txt", help="Path to load FEN from when --fen is not provided")
    parser.add_argument("--engine-path", default="stockfish", help="Stockfish binary path")
    parser.add_argument("--depth", type=int, default=6, help="Search depth")
    parser.add_argument("--movetime-ms", type=int, default=None, help="Optional movetime in milliseconds")
    parser.add_argument("--output", default="data/debug/latest_robot_move_result.json", help="Path to output JSON result")
    parser.add_argument("--apply", action="store_true", help="Apply selected move to board and save FEN")
    parser.add_argument("--save-fen-file", default=None, help="Path to save updated FEN when --apply is used")
    return parser.parse_args()


def load_fen(args):
    if args.fen:
        if args.fen == "startpos":
            return chess.Board().fen()
        return args.fen.strip()

    with open(args.fen_file, "r") as f:
        text = f.read().strip()
    if not text or text == "startpos":
        return chess.Board().fen()
    return text


def write_json(path, payload):
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    with open(path, "w") as f:
        json.dump(payload, f, indent=2, sort_keys=True)


def build_payload(result, move_san=None, fen_after=None, applied=False, fen_saved_to=None):
    status = "ok" if result.legal and not result.error else "error"
    return {
        "status": status,
        "move_uci": result.move_uci,
        "move_san": move_san,
        "legal": bool(result.legal),
        "fen_before": result.fen_before,
        "fen_after_if_applied": fen_after,
        "applied": bool(applied),
        "fen_saved_to": fen_saved_to,
        "engine_path": result.engine_path,
        "depth": result.depth,
        "movetime_ms": result.movetime_ms,
        "error": result.error,
    }


def maybe_apply_move(fen_before, move_uci, apply_requested, fen_file, save_fen_file):
    if not apply_requested:
        return None, False, None

    board = chess.Board(fen_before)
    move_obj = chess.Move.from_uci(move_uci)
    if move_obj not in board.legal_moves:
        raise ValueError("Illegal move for apply: {}".format(move_uci))

    board.push(move_obj)
    fen_after = board.fen()
    target = save_fen_file or fen_file
    if not target:
        raise ValueError("--apply requires --fen-file or --save-fen-file")

    directory = os.path.dirname(target)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    with open(target, "w") as f:
        f.write(fen_after)
        f.write("\n")
    return fen_after, True, target


def main():
    args = parse_args()

    fen_before = load_fen(args)
    config = StockfishConfig(engine_path=args.engine_path, depth=args.depth, movetime_ms=args.movetime_ms)
    engine = StockfishEngine(config=config)
    result = engine.select_move(fen_before)

    move_san = None
    fen_after = None
    applied = False
    fen_saved_to = None

    if result.move_uci and result.legal:
        board = chess.Board(fen_before)
        move_san = board.san(chess.Move.from_uci(result.move_uci))

    if result.legal and args.apply:
        try:
            fen_after, applied, fen_saved_to = maybe_apply_move(
                fen_before,
                result.move_uci,
                True,
                args.fen_file,
                args.save_fen_file,
            )
        except Exception as exc:
            result.error = str(exc)
            result.legal = False

    payload = build_payload(
        result=result,
        move_san=move_san,
        fen_after=fen_after,
        applied=applied,
        fen_saved_to=fen_saved_to,
    )
    write_json(args.output, payload)

    side_to_move = chess.Board(fen_before).turn
    print("fen_before: {}".format(fen_before))
    print("side_to_move: {}".format("white" if side_to_move else "black"))
    print("selected_move_uci: {}".format(result.move_uci))
    print("selected_move_san: {}".format(move_san))
    print("legal: {}".format(result.legal))
    print("output_path: {}".format(args.output))

    if payload["status"] != "ok":
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
