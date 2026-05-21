#!/usr/bin/env python
"""CLI bridge for matching transition evidence to legal human moves."""

import argparse
import json
import os
import sys
from datetime import datetime

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.chess_logic.board_state import ChessBoardState
from chess_robot.chess_logic.legal_move_matcher import match_legal_moves, normalise_transition_result_json


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transition-result", required=True, help="Path to transition_result.json")
    parser.add_argument("--fen", default=None, help="FEN string or startpos")
    parser.add_argument("--fen-file", default=None, help="Load current board FEN from this path")
    parser.add_argument("--save-fen-file", default=None, help="Save committed FEN to this path")
    parser.add_argument("--history-file", default="data/game/move_history.jsonl", help="Path for committed move history JSONL")
    parser.add_argument("--output", required=True, help="Path to JSON output report")
    parser.add_argument("--confirm", action="store_true", help="Interactively choose candidate when ambiguous")
    parser.add_argument("--apply", action="store_true", help="Include fen_after_if_applied in output only")
    parser.add_argument("--commit", action="store_true", help="Persist selected move to FEN/history files")
    return parser.parse_args()


def _load_transition(path):
    with open(path, "r") as f:
        raw = json.load(f)
    return raw, normalise_transition_result_json(raw, source_path=path)


def _print_summary(result):
    print("status: {}".format(result.status))
    print("message: {}".format(result.message))
    if result.accepted_move:
        print("accepted move: {}".format(result.accepted_move))
    elif result.candidates:
        print("candidates:")
        for index, candidate in enumerate(result.candidates, start=1):
            print("  {}. {} ({}, {})".format(index, candidate.uci, candidate.san, candidate.move_type))
    else:
        print("accepted move: None")


def _read_input(prompt):
    try:
        return raw_input(prompt)
    except NameError:
        return input(prompt)


def _maybe_confirm_ambiguous(result):
    if result.status != "ambiguous" or not result.candidates:
        return None
    print("Ambiguous candidates:")
    for index, candidate in enumerate(result.candidates, start=1):
        print("  {}. {} ({})".format(index, candidate.uci, candidate.san))
    while True:
        response = _read_input("Choose candidate number (or Enter to skip): ").strip()
        if response == "":
            return None
        if not response.isdigit():
            print("Please enter a number.")
            continue
        choice = int(response)
        if 1 <= choice <= len(result.candidates):
            return result.candidates[choice - 1].uci
        print("Choice out of range.")


def _board_from_args(args):
    if args.fen:
        return ChessBoardState(args.fen)

    state = ChessBoardState()
    if args.fen_file and os.path.exists(args.fen_file):
        state.load_fen_file(args.fen_file, default_startpos=True)
    return state


def _candidate_map(result):
    return [
        {
            "uci": c.uci,
            "san": c.san,
            "move_type": c.move_type,
            "expected_removed": sorted(c.expected_removed),
            "expected_added": sorted(c.expected_added),
            "score": c.score,
            "notes": list(c.notes),
        }
        for c in result.candidates
    ]


def _append_history(path, entry):
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    with open(path, "a") as f:
        f.write(json.dumps(entry, sort_keys=True))
        f.write("\n")


def main():
    args = _parse_args()

    raw_transition, evidence = _load_transition(args.transition_result)
    state = _board_from_args(args)
    result = match_legal_moves(state.board, evidence)

    selected_move = None
    manual_confirmation_used = False
    if result.status == "ambiguous" and args.confirm:
        selected_move = _maybe_confirm_ambiguous(result)
        manual_confirmation_used = selected_move is not None

    chosen_move = result.accepted_move or selected_move
    committed = False
    fen_saved_to = None
    history_file = None
    fen_after_if_applied = None
    commit_error = None

    if args.apply and chosen_move:
        preview = ChessBoardState(result.fen_before)
        preview.push_uci(chosen_move)
        fen_after_if_applied = preview.fen()

    if args.commit:
        if not chosen_move:
            commit_error = "Commit requested but no unique or confirmed move selected"
        else:
            save_path = args.save_fen_file or args.fen_file
            if not save_path:
                commit_error = "Commit requested but no FEN save path provided"
            else:
                before_state = ChessBoardState(result.fen_before)
                move_uci = chosen_move
                try:
                    move_obj = before_state.push_uci(move_uci)
                    fen_after_if_applied = before_state.fen()
                    move_san = ChessBoardState(result.fen_before).san(move_obj)
                    before_state.save_fen_file(save_path)
                    history_entry = {
                        "timestamp": datetime.utcnow().isoformat() + "Z",
                        "source_transition_result": args.transition_result,
                        "fen_before": result.fen_before,
                        "move_uci": move_uci,
                        "move_san": move_san,
                        "fen_after": fen_after_if_applied,
                        "match_status": result.status,
                        "manual_confirmation_used": manual_confirmation_used,
                        "candidates": _candidate_map(result),
                        "evidence": {
                            "removed": sorted(result.evidence.removed),
                            "added": sorted(result.evidence.added),
                            "uncertain": sorted(result.evidence.uncertain),
                        },
                    }
                    _append_history(args.history_file, history_entry)
                    committed = True
                    fen_saved_to = save_path
                    history_file = args.history_file
                except Exception as exc:
                    commit_error = str(exc)

    payload = {
        "status": result.status,
        "message": result.message,
        "accepted_move": result.accepted_move,
        "selected_move": selected_move,
        "committed": committed,
        "fen_before": result.fen_before,
        "fen_after_if_applied": fen_after_if_applied,
        "fen_saved_to": fen_saved_to,
        "history_file": history_file,
        "evidence": {
            "source_path": result.evidence.source_path,
            "removed": sorted(result.evidence.removed),
            "added": sorted(result.evidence.added),
            "uncertain": sorted(result.evidence.uncertain),
        },
        "candidates": _candidate_map(result),
        "raw_transition": raw_transition,
    }

    if commit_error:
        payload["commit_error"] = commit_error

    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    with open(args.output, "w") as f:
        json.dump(payload, f, indent=2, sort_keys=True)

    _print_summary(result)
    if selected_move:
        print("selected move: {}".format(selected_move))
    print("committed: {}".format(committed))
    print("wrote: {}".format(args.output))

    if args.commit and not committed:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
