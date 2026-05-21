import chess

from chess_robot.chess_logic.board_state import ChessBoardState
from chess_robot.chess_logic.legal_move_matcher import evidence_from_sets, match_legal_moves, normalise_transition_result_json


def test_starting_position_e2e4_unique():
    board = chess.Board()
    evidence = evidence_from_sets(removed=["e2"], added=["e4"])
    result = match_legal_moves(board, evidence)
    assert result.status == "unique"
    assert result.accepted_move == "e2e4"


def test_starting_position_g1f3_unique_knight():
    board = chess.Board()
    evidence = evidence_from_sets(removed=["g1"], added=["f3"])
    result = match_legal_moves(board, evidence)
    assert result.status == "unique"
    assert result.accepted_move == "g1f3"


def test_starting_position_e2e5_no_match():
    board = chess.Board()
    evidence = evidence_from_sets(removed=["e2"], added=["e5"])
    result = match_legal_moves(board, evidence)
    assert result.status == "no_match"
    assert result.accepted_move is None


def test_two_move_like_transition_invalid_evidence():
    board = chess.Board()
    evidence = evidence_from_sets(removed=["e2", "g1"], added=["e4", "f3"])
    result = match_legal_moves(board, evidence)
    assert result.status == "invalid_evidence"


def test_capture_like_ambiguity_two_candidates():
    board = chess.Board("8/8/8/3p1p2/4P3/8/8/4K2k w - - 0 1")
    evidence = evidence_from_sets(removed=["e4"], added=[])
    result = match_legal_moves(board, evidence)
    assert result.status == "ambiguous"
    uci_set = set([candidate.uci for candidate in result.candidates])
    assert "e4d5" in uci_set
    assert "e4f5" in uci_set


def test_capture_like_unique_single_candidate():
    board = chess.Board("8/8/8/3p4/4P3/8/8/4K2k w - - 0 1")
    evidence = evidence_from_sets(removed=["e4"], added=[])
    result = match_legal_moves(board, evidence)
    assert result.status == "unique"
    assert result.accepted_move == "e4d5"


def test_board_state_update_push_move():
    state = ChessBoardState()
    before = state.fen()
    state.push_uci("e2e4")
    after = state.fen()
    assert before != after
    assert state.piece_at("e4") is not None
    assert state.piece_at("e2") is None
    assert state.board.turn == chess.BLACK


def _real_schema_fixture(changed_squares):
    return {
        "schema_version": "1.0",
        "type": "occupancy_transition_result",
        "board_orientation": "black",
        "added_squares": [],
        "removed_squares": [],
        "uncertain_squares": [],
        "changed_squares": changed_squares,
        "summary": {},
        "notes": [],
    }


def test_real_schema_e2e4_parses_removed_and_added():
    raw = _real_schema_fixture([
        {"square": "e2", "change_type": "removed", "previous_state": "occupied", "current_state": "empty"},
        {"square": "e4", "change_type": "added", "previous_state": "empty", "current_state": "occupied"},
    ])
    evidence = normalise_transition_result_json(raw)
    assert evidence.removed == frozenset(["e2"])
    assert evidence.added == frozenset(["e4"])
    assert evidence.uncertain == frozenset()


def test_real_schema_empty_to_e4_add_parses_added_only():
    raw = _real_schema_fixture([
        {"square": "e4", "change_type": "added", "previous_state": "empty", "current_state": "occupied"},
    ])
    evidence = normalise_transition_result_json(raw)
    assert evidence.removed == frozenset()
    assert evidence.added == frozenset(["e4"])


def test_real_schema_e4_to_empty_remove_parses_removed_only():
    raw = _real_schema_fixture([
        {"square": "e4", "change_type": "removed", "previous_state": "occupied", "current_state": "empty"},
    ])
    evidence = normalise_transition_result_json(raw)
    assert evidence.removed == frozenset(["e4"])
    assert evidence.added == frozenset()


def test_real_schema_uncertain_stays_uncertain_only():
    raw = _real_schema_fixture([
        {"square": "e4", "change_type": "uncertain", "previous_state": "occupied", "current_state": "empty"},
    ])
    evidence = normalise_transition_result_json(raw)
    assert evidence.uncertain == frozenset(["e4"])
    assert evidence.removed == frozenset()
    assert evidence.added == frozenset()


def test_real_schema_e2e4_matches_unique_from_startpos():
    board = chess.Board()
    raw = _real_schema_fixture([
        {"square": "e2", "change_type": "removed"},
        {"square": "e4", "change_type": "added"},
    ])
    evidence = normalise_transition_result_json(raw)
    result = match_legal_moves(board, evidence)
    assert result.status == "unique"
    assert result.accepted_move == "e2e4"


def test_add_only_evidence_from_startpos_does_not_false_match():
    board = chess.Board()
    raw = _real_schema_fixture([
        {"square": "e4", "change_type": "added"},
    ])
    evidence = normalise_transition_result_json(raw)
    result = match_legal_moves(board, evidence)
    assert result.status in ("invalid_evidence", "no_match")
    assert result.accepted_move is None
