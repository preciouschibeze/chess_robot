"""Chess logic primitives for board state and move matching."""

from .board_state import ChessBoardState
from .legal_move_matcher import (
    MatchResult,
    MoveCandidate,
    TransitionEvidence,
    evidence_from_sets,
    expected_occupancy_delta,
    match_legal_moves,
    normalise_transition_result_json,
)

__all__ = [
    "ChessBoardState",
    "TransitionEvidence",
    "MoveCandidate",
    "MatchResult",
    "evidence_from_sets",
    "normalise_transition_result_json",
    "expected_occupancy_delta",
    "match_legal_moves",
]
